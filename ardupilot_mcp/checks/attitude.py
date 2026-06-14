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

Enrichment: if ``RCOU`` motor outputs were saturated (any of C1..C4 averaging
above ~1950 us) during the divergence window, the craft was likely simply
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
# RCOU PWM (us) above which a motor output is treated as saturated/maxed-out.
MOTOR_SATURATION_US = 1950.0

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
        findings: list[Finding] = []
        motors_saturated = self._motors_saturated(log)
        for des_field, act_field, axis, is_yaw in _AXES:
            finding = self._axis_finding(log, des_field, act_field, axis, is_yaw, motors_saturated)
            if finding is not None:
                findings.append(finding)
        return findings

    def _axis_finding(
        self,
        log: FlightLog,
        des_field: str,
        act_field: str,
        axis: str,
        is_yaw: bool,
        motors_saturated: bool,
    ) -> Finding | None:
        # Both fields live on the same ATT records; read each as an aligned
        # series and intersect on the common length so they line up by index.
        des_t, des_v = log.series("ATT", des_field)
        act_t, act_v = log.series("ATT", act_field)
        n = min(des_v.size, act_v.size)
        if n == 0:
            return None
        times = act_t[:n]
        err = act_v[:n] - des_v[:n]
        if is_yaw:
            # Yaw is circular: wrap error into [-180, 180].
            err = ((err + 180.0) % 360.0) - 180.0
        abs_err = np.abs(err)

        peak_err = safe_max(abs_err) or 0.0
        if peak_err <= WARN_DEG:
            return None

        # Sustained excursions above the WARN floor; their durations/peaks drive
        # the severity decision.
        sustained = intervals_above(times, abs_err, WARN_DEG, min_duration_s=SUSTAINED_S)
        sustained_crit = [iv for iv in sustained if iv.peak > CRIT_DEG]

        worst: Interval | None
        if sustained_crit:
            worst = max(sustained_crit, key=lambda iv: (iv.peak, iv.duration_s))
            sev = Severity.CRITICAL
            title = f"Attitude tracking failure ({axis})"
            msg = (
                f"{axis.capitalize()} failed to track the demand for {worst.duration_s:.1f} s, "
                f"error up to {worst.peak:.0f} deg (sustained >{CRIT_DEG:.0f} deg). The aircraft "
                "could not achieve the commanded attitude — consistent with loss of control, a "
                "mechanical failure, or an underpowered/overweight craft."
            )
            rec = (
                "Inspect the airframe for mechanical failure (motor/ESC/prop, control linkage) and "
                "review whether the craft is overweight or under-powered for the demanded maneuvers."
            )
        elif sustained:
            # Sustained, in the 15-25 deg band (no sustained interval broke 25).
            worst = max(sustained, key=lambda iv: (iv.peak, iv.duration_s))
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

        # Enrichment: if motors were maxed out, the craft was likely underpowered.
        if sev == Severity.CRITICAL and motors_saturated:
            msg += (
                " Motor outputs (RCOU C1-C4) were saturated near maximum during the flight, so the "
                "craft was likely underpowered/overweight for the demand."
            )

        samples: dict[str, float] = {
            f"peak_err_deg_{axis}": round(peak_err, 1),
            f"worst_duration_s_{axis}": worst_dur,
        }
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

    def _motors_saturated(self, log: FlightLog) -> bool:
        """True if any RCOU motor channel (C1..C4) averaged above saturation."""
        if not log.has("RCOU"):
            return False
        for chan in ("C1", "C2", "C3", "C4"):
            vals = log.field("RCOU", chan)
            if vals.size and float(np.mean(vals)) > MOTOR_SATURATION_US:
                return True
        return False
