# SPDX-License-Identifier: AGPL-3.0-only
"""Attitude tracking check (the crown-jewel check).

Reads the ``ATT`` message, which logs both the *demanded* attitude
(DesRoll/DesPitch/DesYaw) and the *achieved* attitude (Roll/Pitch/Yaw) in
degrees. For each axis the tracking error is ``actual - desired``. Yaw is
circular, so its error is wrapped into [-180, 180].

A large, *sustained* gap between demand and achievement means the autopilot
asked for an attitude the airframe never reached: loss of control, a mechanical
failure (broken/slipping motor, control surface), wrong tuning, or an
underpowered/overweight craft. A brief spike is far less alarming (a gust or a
single aggressive stick input) than a multi-second divergence.

Copter and heli are judged the way the firmware judges them. Their crash and parachute
checks look at the thrust-vector ("lean") error, the angle between the demanded and the
achieved thrust direction, and call 30 deg of it held for 1 s a loss of control; heading
is not part of it (ArduCopter/crash_check.cpp, AC_AttitudeControl get_att_error_angle_deg).
So roll and pitch are judged together as that lean error, named after the axis that
carried most of it, and a yaw divergence is at most a warning: the motor mixer fits yaw
into whatever room roll and pitch leave (AP_MotorsMatrix output_armed_stabilizing), so
heading is the first thing a saturated or failing motor gives up.

Enrichment: if ``RCOU`` motor outputs were saturated (any of C1..C4 averaging
above ~1950 us) during the worst divergence window, the craft was likely simply
underpowered for the demand, and we say so. This is deterministic and additive.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import Interval, intervals_above, make_finding, safe_max

# A divergence held this long is "sustained" rather than a transient spike.
SUSTAINED_S = 1.0
# Sustained error above this is a loss-of-tracking emergency.
CRIT_DEG = 25.0
# Sustained error in this band is concerning but not (yet) critical.
WARN_DEG = 15.0
# A brief error above this still warrants a warning even if it self-corrects.
BRIEF_WARN_DEG = 30.0
# RCOU PWM (us) at or above which a motor output is treated as saturated/maxed-out,
# the same level as the motors check.
MOTOR_SATURATION_US = 1950.0
# Design choice: the saturation window extends this far either side of the worst
# divergence, so a brief one still holds RCOU samples (RCOU logs at ~10 Hz and up).
SATURATION_PAD_S = 0.5

# Copter's own loss-of-control rule: a thrust-vector error over 30 deg ("30 degrees off from
# target indicates a loss of control") held for 1 s (PARACHUTE_CHECK_ANGLE_DEVIATION_DEG,
# PARACHUTE_CHECK_TRIGGER_SEC; the crash check uses the same 30 deg). ArduCopter/crash_check.cpp.
LEAN_CRIT_DEG = 30.0
LEAN_CRIT_S = 1.0
# Above this lean error the attitude controller scales its yaw correction down, and above twice
# it stops correcting heading (AC_ATTITUDE_THRUST_ERROR_ANGLE, AC_AttitudeControl.h/.cpp).
THRUST_ERROR_YAW_LIMIT_DEG = 30.0
# Vehicles judged by that rule (Copter's crash_check runs for multirotors and helis alike).
LEAN_KINDS = {"copter", "heli"}
# Design choice: the airspeed note uses the last CTUN sample at most this old at the onset.
SPEED_NOTE_MAX_AGE_S = 1.0

# ArduPlane modes in which ATT.DesRoll/DesPitch (nav_roll_cd/nav_pitch_cd) is not the
# attitude the pilot or autopilot asked for: MANUAL writes the current attitude, STABILIZE
# writes 0 while the pilot banks with stick mixing, TRAINING writes 0 inside its limits, and
# ACRO writes a locked error or the current attitude (ArduPlane/mode_manual.cpp,
# mode_stabilize.cpp, mode_training.cpp, mode_acro.cpp, Plane-4.6). Mode numbers: PLANE_MODES.
PLANE_NO_TARGET_MODES = {0, 2, 3, 4}  # MANUAL, STABILIZE, TRAINING, ACRO
# Plane's roll and pitch controllers follow their demand with a first-order time constant
# (RLL2SRV_TCONST / PTCH2SRV_TCONST, "time constant in seconds from demanded to achieved" angle,
# default 0.5 s; AP_RollController.cpp, AP_PitchController.cpp), so a demand that jumps
# (a turn reversal in AUTO, a waypoint change) is followed by an error that closes over a few
# time constants. Design choice: a Plane divergence that starts within STEP_LOOKBACK_S of the
# demand moving more than CRIT_DEG, and is over within STEP_LAG_MAX_S, is that lag. Validation:
# in completed flights of the forum landing logs all five such criticals were turn reversals in
# AUTO (demand steps of 26-59 deg, 1.0-1.4 s); in the crash pools it changes no labelled loss.
STEP_LOOKBACK_S = 1.0
STEP_LAG_MAX_S = 1.5
# A demand that keeps moving after the divergence starts (a 90 deg turn reversal in RTL) is the
# same lag seen later. Design choice: a Plane divergence is also that lag when the demand moved
# more than CRIT_DEG from STEP_LOOKBACK_S before it to its end, the aircraft was rotating toward
# the demand in at least TOWARD_MIN_FRACTION of the samples where it rotated faster than
# MOVING_RATE_DEG_S, and it was over within LAG_TCONSTS time constants (4 x 0.5 s by default:
# a first-order lag has closed 98% of a step by then). A loss of control rotates away from the
# demand. Validation on the 100 Plane attitude criticals in the 215 real logs: in the crash logs
# no fixed-wing critical rotated toward the demand more than 79% of the time, and no crash log's
# findings change; every critical it turns into a warning is in a flight that ended normally
# (turn reversals in FBWA, AUTO, RTL and TAKEOFF, AUTOTUNE twitches). The windy FBWA upset in forum 99954 rotated toward the demand 51% of the
# time, with the aileron and elevator at their limits, and stays critical.
TOWARD_MIN_FRACTION = 0.90
MOVING_RATE_DEG_S = 5.0
LAG_TCONSTS = 4.0
TCONST_DEFAULT_S = 0.5  # RLL2SRV_TCONST / PTCH2SRV_TCONST default
# ArduPlane VTOL modes run the multicopter attitude controller, not the fixed-wing one.
PLANE_Q_MODES = {17, 18, 19, 20, 21, 22, 23, 25}  # QSTABILIZE..QACRO, LOITER_ALT_QLAND

# (desired field, actual field, axis label, is-yaw-circular).
_AXES = (
    ("DesRoll", "Roll", "roll", False),
    ("DesPitch", "Pitch", "pitch", False),
    ("DesYaw", "Yaw", "yaw", True),
)


@register_check
class AttitudeTrackingCheck(Check):
    id = "attitude"
    title = "Attitude tracking"
    category = "attitude"
    requires = {"ATT"}
    # Vehicles that actively command roll/pitch attitude. A rover does NOT — its
    # roll/pitch just follow the terrain while DesRoll/DesPitch stay ~0, so the
    # "error" is meaningless and would false-positive on rough ground.
    vehicles = {"copter", "heli", "plane"}
    description = "Compares demanded vs achieved roll/pitch/yaw from ATT to catch loss of control."

    def run(self, log: FlightLog) -> list[Finding]:
        if log.meta.vehicle_kind in LEAN_KINDS:
            return self._multirotor_findings(log)
        findings: list[Finding] = []
        for des_field, act_field, axis, is_yaw in _AXES:
            # ArduPlane logs DesYaw as a constant 0 placeholder ("Plane does not have
            # the concept of navyaw", ArduPlane/Log.cpp), so its yaw "error" is just
            # the heading.
            if is_yaw and log.meta.vehicle_kind == "plane":
                continue
            finding = self._axis_finding(log, des_field, act_field, axis, is_yaw)
            if finding is not None:
                findings.append(finding)
        return findings

    def _multirotor_findings(self, log: FlightLog) -> list[Finding]:
        """Copter/heli: one lean (thrust-vector) finding and one heading finding."""
        errors = self._lean_heading_error(log)
        if errors is None:
            return []
        times, lean_err, heading_err, axis_errs = errors
        findings = [
            self._judge(log, "lean", times, lean_err, rule="lean", axis_errs=axis_errs),
            self._judge(log, "yaw", times, heading_err, rule="heading"),
        ]
        return [f for f in findings if f is not None]

    def _axis_finding(
        self,
        log: FlightLog,
        des_field: str,
        act_field: str,
        axis: str,
        is_yaw: bool,
    ) -> Finding | None:
        err = self._axis_error(log, des_field, act_field, is_yaw=is_yaw)
        if err is None:
            return None
        demand = None
        if log.meta.vehicle_kind == "plane" and not is_yaw:
            td, (dv, av) = log.series_many("ATT", [des_field, act_field])
            demand = (td, dv, av)
        return self._judge(log, axis, *err, rule="axis", demand=demand)

    def _axis_error(
        self, log: FlightLog, des_field: str, act_field: str, *, is_yaw: bool
    ) -> tuple[np.ndarray, np.ndarray] | None:
        # Both fields from the same ATT records, so demand and actual stay paired.
        times, (des_v, act_v) = log.series_many("ATT", [des_field, act_field])
        if times.size == 0:
            return None
        err = act_v - des_v
        if is_yaw:
            # Yaw is circular: wrap error into [-180, 180].
            err = ((err + 180.0) % 360.0) - 180.0
        return times, self._in_flight(log, times, np.abs(err))

    def _lean_heading_error(
        self, log: FlightLog
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray]] | None:
        """Thrust-direction and heading error per ATT record, as AC_AttitudeControl splits them.

        The lean error is the angle between the demanded and the achieved thrust direction
        (the body z axes in the earth frame). The heading error is what remains once the
        achieved frame is turned the shortest way onto the demanded thrust direction: the
        angle between the two x axes about it (thrust_heading_rotation_angles). Both stay
        exact near 90 deg of pitch, where Euler roll and yaw differences stop meaning anything.
        The heading is not judged while the lean error is over AC_ATTITUDE_THRUST_ERROR_ANGLE,
        where the firmware scales its yaw correction down (and drops it above twice that).
        """
        fields = ["DesRoll", "Roll", "DesPitch", "Pitch", "DesYaw", "Yaw"]
        times, (dr, r, dp, p, dy, y) = log.series_many("ATT", fields)
        if times.size == 0:
            return None
        x_d, z_d = _body_axes(dr, dp, dy)
        x_a, z_a = _body_axes(r, p, y)
        cos_t = np.clip(np.sum(z_a * z_d, axis=0), -1.0, 1.0)
        lean = np.degrees(np.arccos(cos_t))
        # Rodrigues: turn x_a about k = z_a x z_d by the lean angle.
        k = np.cross(z_a, z_d, axis=0)
        sin_t = np.linalg.norm(k, axis=0)
        with np.errstate(invalid="ignore", divide="ignore"):
            k_hat = np.where(sin_t > 1e-9, k / sin_t, 0.0)
        x_rot = (
            x_a * cos_t
            + np.cross(k_hat, x_a, axis=0) * sin_t
            + k_hat * np.sum(k_hat * x_a, axis=0) * (1.0 - cos_t)
        )
        heading = np.degrees(
            np.abs(
                np.arctan2(np.sum(np.cross(x_rot, x_d, axis=0) * z_d, axis=0), np.sum(x_rot * x_d, axis=0))
            )
        )
        heading = np.where(lean > THRUST_ERROR_YAW_LIMIT_DEG, 0.0, heading)
        axis_errs = {
            "roll": self._in_flight(log, times, np.abs(r - dr)),
            "pitch": self._in_flight(log, times, np.abs(p - dp)),
        }
        return times, self._in_flight(log, times, lean), self._in_flight(log, times, heading), axis_errs

    def _in_flight(self, log: FlightLog, times: np.ndarray, abs_err: np.ndarray) -> np.ndarray:
        # Judge flight only: on the ground (before take-off, after landing, a crashed
        # airframe lying on its side, disarmed) the targets are relaxed or frozen.
        # Zero the error there instead of dropping samples, so no interval spans a gap.
        flying = log.flying_mask(times)
        if flying is not None:
            abs_err = np.where(flying, abs_err, 0.0)
        if log.meta.vehicle_kind == "plane":
            # No attitude target in these modes, so there is nothing to track.
            abs_err = np.where(log.mode_mask(times, PLANE_NO_TARGET_MODES), 0.0, abs_err)
        return abs_err

    def _judge(
        self,
        log: FlightLog,
        axis: str,
        times: np.ndarray,
        abs_err: np.ndarray,
        *,
        rule: str,
        axis_errs: dict[str, np.ndarray] | None = None,
        demand: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
    ) -> Finding | None:
        """Severity and text for one error series.

        rule "axis": a sustained error over CRIT_DEG is critical (Plane, unknown vehicles).
        rule "lean": Copter's own loss-of-control rule, LEAN_CRIT_DEG held for LEAN_CRIT_S.
        rule "heading": a multirotor's heading, at most a warning.
        """
        peak_err = safe_max(abs_err) or 0.0
        if peak_err <= WARN_DEG:
            return None

        # Sustained excursions above the WARN floor; their durations/peaks drive
        # the severity decision.
        sustained = intervals_above(times, abs_err, WARN_DEG, min_duration_s=SUSTAINED_S)
        if rule == "lean":
            # Counted like the firmware's check: each sample above holds for one period.
            dt = float(np.median(np.diff(times))) if times.size > 1 else 0.0
            over = intervals_above(times, abs_err, LEAN_CRIT_DEG)
            sustained_crit = [iv for iv in over if round(iv.duration_s + dt, 6) >= LEAN_CRIT_S]
            crit_deg, crit_s = LEAN_CRIT_DEG, LEAN_CRIT_S
        else:
            sustained_crit = [iv for iv in sustained if iv.peak > CRIT_DEG]
            crit_deg, crit_s = CRIT_DEG, SUSTAINED_S
        lagged: list[tuple[Interval, tuple[float, float | None]]] = []
        if demand is not None:
            for iv in sustained_crit:
                lag = _demand_step(log, demand, iv, axis)
                if lag is not None:
                    lagged.append((iv, lag))
            sustained_crit = [iv for iv in sustained_crit if all(iv is not lg for lg, _ in lagged)]

        worst: Interval | None
        if sustained_crit and rule == "heading":
            worst = max(sustained_crit, key=lambda iv: (iv.peak, iv.duration_s))
            sev = Severity.WARN
            title = "Heading not held (yaw)"
            msg = (
                f"Yaw failed to track the demand for {worst.duration_s:.1f} s, error up to "
                f"{worst.peak:.0f} deg. A multirotor gives heading the lowest priority: the motor "
                "mixer fits yaw into whatever room roll and pitch leave, so heading is the first "
                "thing given up when the motors saturate or one of them fails, and the firmware's "
                "crash check does not count it. On its own this is not a loss of control."
            )
            rec = (
                "Look at the motor outputs over the same window: one motor pinned high or low while "
                "the heading drifts points at a failing motor or ESC, a damaged prop or a twisted arm; "
                "all of them high points at an underpowered or overweight craft."
            )
        elif sustained_crit:
            worst = max(sustained_crit, key=lambda iv: (iv.peak, iv.duration_s))
            sev = Severity.CRITICAL
            if rule == "lean" and axis_errs is not None:
                axis = _dominant_axis(times, axis_errs, worst)
                what = (
                    f"The thrust direction was {worst.peak:.0f} deg off the demand, mostly in {axis}, "
                    f"for {worst.duration_s:.1f} s (over {crit_deg:.0f} deg for {crit_s:.0f} s is the "
                    "firmware's own loss-of-control line)."
                )
            else:
                what = (
                    f"{axis.capitalize()} failed to track the demand for {worst.duration_s:.1f} s, "
                    f"error up to {worst.peak:.0f} deg (sustained >{crit_deg:.0f} deg)."
                )
            title = f"Attitude tracking failure ({axis})"
            msg = (
                f"{what} The aircraft could not achieve the commanded attitude — consistent with "
                "loss of control, a mechanical failure, or an underpowered/overweight craft."
            )
            rec = (
                "Inspect the airframe for mechanical failure (motor/ESC/prop, control linkage) and "
                "review whether the craft is overweight or under-powered for the demanded maneuvers."
            )
        elif lagged:
            worst, (step, toward) = max(lagged, key=lambda x: (x[0].peak, x[0].duration_s))
            sev = Severity.WARN
            title = f"Attitude lagged a step in the demand ({axis})"
            when = (
                f"right after the demand itself moved {step:.0f} deg within {STEP_LOOKBACK_S:.0f} s, and the error "
                "then closed."
                if toward is None
                else f"while the demand itself moved {step:.0f} deg, and the aircraft kept rotating toward it "
                f"({toward * 100:.0f}% of the time it was rotating), so the error closed."
            )
            msg = (
                f"{axis.capitalize()} fell up to {worst.peak:.0f} deg behind the demand for {worst.duration_s:.1f} s, "
                f"{when} Plane's attitude controller follows a demand with a time constant "
                f"({'RLL2SRV_TCONST' if axis == 'roll' else 'PTCH2SRV_TCONST'}), so a sudden change such as a turn "
                "reversal is followed by a lag like this; it is not a loss of control."
            )
            rec = (
                "Nothing to do unless the lag is larger than the airframe should show: then check the time constant "
                "and the rate limit for this axis, and the tune."
            )
        elif sustained:
            # Sustained above WARN_DEG, short of the critical rule.
            worst = max(sustained, key=lambda iv: (iv.peak, iv.duration_s))
            if axis_errs is not None:
                axis = _dominant_axis(times, axis_errs, worst)
            sev = Severity.WARN
            title = f"Sustained attitude tracking error ({axis})"
            msg = (
                f"{axis.capitalize()} tracking error stayed above {WARN_DEG:.0f} deg for "
                f"{worst.duration_s:.1f} s, peaking at {worst.peak:.0f} deg. The craft lagged the "
                "demand noticeably — worth checking tuning and control authority."
            )
            rec = "Review PID tuning and control authority; confirm no partial mechanical loss."
        elif peak_err > BRIEF_WARN_DEG:
            # Large but brief (no interval held >= SUSTAINED_S): a transient.
            brief = intervals_above(times, abs_err, BRIEF_WARN_DEG)
            worst = max(brief, key=lambda iv: iv.peak) if brief else None
            if axis_errs is not None and worst is not None:
                axis = _dominant_axis(times, axis_errs, worst)
            sev = Severity.WARN
            title = f"Brief attitude tracking spike ({axis})"
            dur = worst.duration_s if worst else 0.0
            msg = (
                f"{axis.capitalize()} tracking error briefly spiked to {peak_err:.0f} deg "
                f"(>{BRIEF_WARN_DEG:.0f} deg) for under {SUSTAINED_S:.1f} s ({dur:.2f} s). A short "
                "excursion like this is usually a gust or aggressive input rather than a failure, "
                "but it is worth a glance."
            )
            rec = "Likely a transient (gust/aggressive input); review if it recurs across flights."
        else:
            # Peak is 25-30 deg but only briefly, and nothing sustained: not
            # severe enough to flag.
            return None

        # Worst-interval window for the evidence timestamps.
        t0 = worst.start_s if worst is not None else None
        t1 = worst.end_s if worst is not None else None
        worst_dur = round(worst.duration_s, 2) if worst is not None else 0.0

        # Enrichment: if motors were maxed out while it diverged, the craft was likely underpowered.
        motors_saturated = self._motors_saturated(log, t0, t1)
        if sev == Severity.CRITICAL and motors_saturated:
            msg += (
                " Motor outputs (RCOU C1-C4) were saturated near maximum while the attitude diverged, "
                "so the craft was likely underpowered/overweight for the demand."
            )

        key = "lean" if rule == "lean" else axis
        samples: dict[str, float] = {
            f"peak_err_deg_{key}": round(peak_err, 1),
            f"worst_duration_s_{key}": worst_dur,
        }
        if sev == Severity.CRITICAL and log.meta.vehicle_kind == "plane":
            note = _plane_speed_note(log, t0)
            if note is not None:
                msg += note[0]
                samples.update(note[1])
        if axis_errs is not None and worst is not None:
            win = (times >= worst.start_s) & (times <= worst.end_s)
            for name, e in axis_errs.items():
                if win.any():
                    samples[f"peak_err_deg_{name}"] = round(float(np.max(e[win])), 1)
        if motors_saturated:
            samples["motors_saturated"] = 1.0

        return make_finding(
            self.id,
            sev,
            title,
            msg,
            recommendation=rec,
            time_start_s=t0,
            time_end_s=t1,
            message_types=["ATT", "RCOU"] if motors_saturated else ["ATT"],
            samples=samples,
        )

    def _motors_saturated(self, log: FlightLog, t0: float | None, t1: float | None) -> bool:
        """True if any RCOU motor channel (C1..C4) averaged above saturation in [t0, t1]."""
        if t0 is None or t1 is None or not log.has("RCOU"):
            return False
        for chan in ("C1", "C2", "C3", "C4"):
            times, vals = log.series("RCOU", chan)
            vals = vals[(times >= t0 - SATURATION_PAD_S) & (times <= t1 + SATURATION_PAD_S)]
            if vals.size and float(np.mean(vals)) >= MOTOR_SATURATION_US:
                return True
        return False


def _body_axes(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Body x and z axes in the earth frame for ZYX Euler angles in degrees (3 x n each)."""
    r, p, y = np.radians(roll), np.radians(pitch), np.radians(yaw)
    x = np.stack([np.cos(p) * np.cos(y), np.cos(p) * np.sin(y), -np.sin(p)])
    z = np.stack(
        [
            np.cos(r) * np.sin(p) * np.cos(y) + np.sin(r) * np.sin(y),
            np.cos(r) * np.sin(p) * np.sin(y) - np.sin(r) * np.cos(y),
            np.cos(r) * np.cos(p),
        ]
    )
    return x, z


def _plane_speed_note(log: FlightLog, t0: float | None) -> tuple[str, dict[str, float]] | None:
    """The airspeed a plane's loss of control began at, when the flight ran above AIRSPEED_MAX.

    Only for a flight that spent much of its time above AIRSPEED_MAX (the param_audit finding),
    so a dive after the loss of control does not count; the note states what was measured and
    leaves the cause to the reader.
    """
    # Imported here so this module does not register param_audit ahead of the checks list order.
    from .param_audit import flew_above_airspeed_max, speed_scaler

    envelope = flew_above_airspeed_max(log)
    if envelope is None or t0 is None:
        return None
    max_name, vmax, frac, times, aspd = envelope
    k = int(np.searchsorted(times, t0, side="right")) - 1
    if k < 0 or times[k] < t0 - SPEED_NOTE_MAX_AGE_S or aspd[k] <= vmax:
        return None
    speed = float(aspd[k])
    text = (
        f" It began at {speed:.0f} m/s, above {max_name} ({vmax:.0f} m/s), where this flight spent "
        f"{frac * 100:.0f}% of its time"
    )
    samples = {"airspeed_at_onset": round(speed, 1)}
    scaling = log.params.get("SCALING_SPEED")
    if scaling and scaling > 0:
        scaler = float(speed_scaler(log, np.array([speed]))[0])
        text += f": with SCALING_SPEED {scaling:.0f} m/s the surfaces had {scaler:.2f}x the deflection they were tuned at"
        samples["speed_scaler_at_onset"] = round(scaler, 2)
    return text + f" (see 'Flew far above {max_name}').", samples


def _demand_step(
    log: FlightLog, demand: tuple[np.ndarray, np.ndarray, np.ndarray], iv: Interval, axis: str
) -> tuple[float, float | None] | None:
    """(demand move in deg, share of rotation toward it) when a Plane divergence is the controller's lag.

    The share is None when the move came before the divergence started (the step rule).
    """
    if log.mode_mask(np.array([iv.start_s]), PLANE_Q_MODES)[0]:
        return None
    td, dv, av = demand
    before = (td >= iv.start_s - STEP_LOOKBACK_S) & (td <= iv.start_s) & np.isfinite(dv)
    if iv.duration_s < STEP_LAG_MAX_S and before.any():
        step = float(np.ptp(dv[before]))
        if step > CRIT_DEG:
            return step, None
    tconst = log.params.get("RLL2SRV_TCONST" if axis == "roll" else "PTCH2SRV_TCONST", TCONST_DEFAULT_S)
    if iv.duration_s >= LAG_TCONSTS * tconst:
        return None
    around = (td >= iv.start_s - STEP_LOOKBACK_S) & (td <= iv.end_s) & np.isfinite(dv)
    inside = (td >= iv.start_s) & (td <= iv.end_s) & np.isfinite(dv) & np.isfinite(av)
    if not around.any() or np.count_nonzero(inside) < 4:
        return None
    step = float(np.ptp(dv[around]))
    if step <= CRIT_DEG:
        return None
    rate = np.gradient(av[inside], td[inside])
    moving = np.abs(rate) > MOVING_RATE_DEG_S
    if np.count_nonzero(moving) < 3:
        return None
    toward = float(np.mean(np.sign(rate[moving]) == np.sign((dv[inside] - av[inside])[moving])))
    return (step, toward) if toward >= TOWARD_MIN_FRACTION else None


def _dominant_axis(times: np.ndarray, axis_errs: dict[str, np.ndarray], iv: Interval) -> str:
    """The axis (roll or pitch) with the larger error inside the interval."""
    win = (times >= iv.start_s) & (times <= iv.end_s)
    if not win.any():
        return "roll"
    return max(axis_errs, key=lambda k: float(np.max(axis_errs[k][win])))
