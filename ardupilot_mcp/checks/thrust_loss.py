"""Thrust loss on one motor, judged with the frame's own motor geometry.

When a motor, ESC or propeller stops producing its share of thrust, the attitude
controller drives that motor to its maximum output and the vehicle still sinks. Copter
detects this on board (``Copter::thrust_loss_check``) and logs three traces of it:
the message "Potential Thrust Loss (n)", an ERR with Subsys THRUST_LOSS_CHECK, and
bit 0 (thrust boost) of ``MOTB.FailFlags``. Any of those is the firmware's own
verdict and is reported CRITICAL.

Independently of the firmware, the log can show the same signature in the motor
outputs. It is reported WARN, and only when every condition the firmware itself
requires holds for at least ``THRUST_LOSS_CHECK_TRIGGER_SEC``:

* one motor (or one coaxial group) is at its maximum output while every other motor
  still has headroom;
* the motor geometrically opposite it, from the frame's roll/pitch factors
  (AP_MotorsMatrix), is the lowest of the rest: the controller is pulling the far side
  down to hold attitude against the weak side;
* throttle is at least ``THRUST_LOSS_CHECK_MINIMUM_THROTTLE`` (input or output),
  the vehicle is descending, the requested lean is within
  ``THRUST_LOSS_CHECK_ANGLE_DEVIATION_DEG`` and the attitude error is under
  ``CRASH_CHECK_ANGLE_DEVIATION_DEG`` (the vehicle is still under control);
* the vehicle is flying (land detector / armed), so spool-up and a crashed airframe
  are not judged.

Sources (Copter-4.6.3): ArduCopter/crash_check.cpp (thrust_loss_check and its
constants), libraries/AP_Motors/AP_MotorsMatrix.cpp (frame geometry,
check_for_failed_motor), libraries/AP_Motors/AP_MotorsMulticopter.cpp (MOTB
FailFlags = thrust_boost | thrust_balanced << 1; output = PWM_MIN + (PWM_MAX -
PWM_MIN) * actuator), libraries/AP_Logger/AP_Logger.h (LogErrorSubsystem
THRUST_LOSS_CHECK = 25), libraries/SRV_Channel/SRV_Channel.h (Motor1-8 = 33-40,
Motor9-12 = 82-85).
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from ..motor_geometry import FRAME_DEFAULT, FRAME_GEOMETRY
from .base import Check, register_check
from .firmware import VERDICTS
from .util import intervals_above, make_finding

# ArduCopter/crash_check.cpp
THRUST_LOSS_CHECK_TRIGGER_SEC = 1.0
THRUST_LOSS_CHECK_ANGLE_DEVIATION_DEG = 15.0  # THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD 1500
THRUST_LOSS_CHECK_MINIMUM_THROTTLE = 0.9
THRUST_LOSS_CHECK_LOW_THROTTLE = 0.25  # "prevent checks triggering ... low commanded throttle"
CRASH_CHECK_ANGLE_DEVIATION_DEG = 30.0
ERR_THRUST_LOSS_CHECK = 25  # LogErrorSubsystem::THRUST_LOSS_CHECK
MOTB_THRUST_BOOST = 0x1  # FailFlags bit 0
# The firmware-verdicts entry this check reports (one pattern, so its dedupe and this match agree).
THRUST_LOSS_MSG = next(v.pattern for v in VERDICTS if v.reported_by == "thrust_loss")

# AP_MotorsMulticopter.cpp parameter defaults.
PWM_MIN_DEFAULT, PWM_MAX_DEFAULT = 1000.0, 2000.0
SPIN_MIN_DEFAULT, SPIN_MAX_DEFAULT = 0.15, 0.95

# Design choice: a motor is "at maximum" within this much (0..1 actuator) of MOT_SPIN_MAX,
# and the other motors "have headroom" when all of them are at least HEADROOM below it.
SATURATED_MARGIN = 0.03
HEADROOM = 0.08
# Design choice: motors whose roll/pitch direction is within this cosine of the most
# opposite one share its side (coaxial pairs); a side counts as opposite only when its
# direction points at least this far away (cosine <= OPPOSITE_MAX_COS).
SAME_SIDE_COS = 0.05
OPPOSITE_MAX_COS = -0.7
# Design choice: an output-signature episode corroborates the firmware's detection when it
# overlaps the detection time within this slack (the firmware fires after 1 s of the pattern).
DETECTION_SLACK_S = 2.0

# SERVOn_FUNCTION exists for up to 32 outputs (SRV_Channel.h: NUM_SERVO_CHANNELS <= 32).
# A motor on an output RCOU does not log (C15 and up) cannot be judged.
SERVO_OUTPUTS = 32
_MOTOR_FUNCTION = {k: 32 + k for k in range(1, 9)} | {k: 73 + k for k in range(9, 13)}


def _hold(src_t: np.ndarray, src_v: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Sample-and-hold ``src_v`` at ``t`` (NaN before the first sample)."""
    if src_t.size == 0:
        return np.full(t.shape, np.nan)
    i = np.searchsorted(src_t, t, side="right") - 1
    out = src_v[np.clip(i, 0, None)].astype(float)
    out[i < 0] = np.nan
    return out


def _onset(detected: float | None, sig: dict | None) -> float | None:
    """When the thrust loss began: the firmware fires only after THRUST_LOSS_CHECK_TRIGGER_SEC of
    the pattern, so it started at least that long before the detection (or when the corroborating
    output pattern began, if earlier)."""
    if detected is None:
        return sig["start"] if sig else None
    start = detected - THRUST_LOSS_CHECK_TRIGGER_SEC
    return min(start, sig["start"]) if sig else start


def frame_geometry(log: FlightLog) -> tuple[tuple[float, float], ...] | None:
    """Per-motor (roll, pitch) factors for this frame, or None if the frame is not a known matrix."""
    fc, ft = log.params.get("FRAME_CLASS"), log.params.get("FRAME_TYPE")
    if fc is None:
        return None
    geo = FRAME_GEOMETRY.get((int(fc), int(ft if ft is not None else 1)))
    return geo if geo is not None else FRAME_DEFAULT.get(int(fc))


def opposite_side(geo: tuple[tuple[float, float], ...], motor: int) -> list[int]:
    """0-based motors on the far side from ``motor`` (several for coaxial frames; [] if none)."""
    v = np.array(geo[motor], dtype=float)
    vn = np.linalg.norm(v)
    if vn == 0:
        return []
    cos = []
    for j, w in enumerate(geo):
        wn = np.linalg.norm(w)
        cos.append(float(np.dot(v, w) / (vn * wn)) if wn > 0 and j != motor else np.inf)
    best = min(cos)
    if best > OPPOSITE_MAX_COS:
        return []
    return [j for j, c in enumerate(cos) if c <= best + SAME_SIDE_COS]


def same_side(geo: tuple[tuple[float, float], ...], motor: int) -> list[int]:
    """0-based motors at the same position as ``motor`` (itself plus any coaxial partner)."""
    v = np.array(geo[motor], dtype=float)
    return [j for j, w in enumerate(geo) if np.allclose(w, v, atol=1e-3)]


def pwm_range(log: FlightLog) -> tuple[float, float]:
    """PWM the motors output at actuator 0 and 1.

    Copter 4.6 refuses to arm unless 1 <= MOT_PWM_MIN < MOT_PWM_MAX ("Check MOT_PWM_MIN and
    MOT_PWM_MAX"), so an armed flight with 0 here comes from older firmware, where 0 meant
    "use the throttle channel's RC3_MIN/RC3_MAX".
    """
    pmin = float(log.params.get("MOT_PWM_MIN", PWM_MIN_DEFAULT))
    pmax = float(log.params.get("MOT_PWM_MAX", PWM_MAX_DEFAULT))
    if 1 <= pmin < pmax:
        return pmin, pmax
    rmin, rmax = log.params.get("RC3_MIN"), log.params.get("RC3_MAX")
    if rmin is not None and rmax is not None and 1 <= float(rmin) < float(rmax):
        return float(rmin), float(rmax)
    return PWM_MIN_DEFAULT, PWM_MAX_DEFAULT


def configured_channel(log: FlightLog, motor: int) -> str | None:
    """RCOU channel SERVOn_FUNCTION assigns to 1-based ``motor``, or None if no param maps it."""
    fn = _MOTOR_FUNCTION.get(motor)
    for n in range(1, SERVO_OUTPUTS + 1):
        if fn is not None and int(log.params.get(f"SERVO{n}_FUNCTION", -1)) == fn:
            return f"C{n}"
    return None


def motor_channel(log: FlightLog, motor: int) -> str:
    """RCOU channel driving 1-based ``motor`` (SERVOn_FUNCTION; channel n by default)."""
    return configured_channel(log, motor) or f"C{motor}"


@register_check
class ThrustLossCheck(Check):
    id = "thrust_loss"
    title = "Thrust loss on a motor"
    category = "motors"
    requires: set[str] = set()  # MSG / ERR / MOTB / RCOU are each optional evidence
    # Heli too: Copter runs thrust_loss_check on every frame, and the firmware-verdicts check
    # leaves this message to this check. The output signature needs a matrix frame, so on a
    # heli only the firmware's own detection is reported.
    vehicles = {"copter", "heli"}
    description = (
        "Firmware thrust-loss detection (message, ERR, MOTB thrust boost) and the same signature in the "
        "motor outputs: one motor at maximum, its geometric opposite lowest, sinking at high throttle."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        firmware = self._firmware_evidence(log)
        episodes = self._signature_episodes(log)
        if firmware is not None:
            # Only an episode around the detection corroborates it; one elsewhere in the
            # flight (e.g. later, while thrust boost was already on) is a different event.
            t0 = firmware["start"]
            near = [
                e
                for e in episodes
                if t0 is not None
                and e["start"] <= t0 + DETECTION_SLACK_S
                and e["end"] >= t0 - DETECTION_SLACK_S
            ]
            sig = max(near, key=lambda e: e["duration"]) if near else None
            return [self._firmware_finding(log, firmware, sig)]
        if not episodes:
            return []
        return [self._signature_finding(max(episodes, key=lambda e: e["duration"]))]

    # -- the firmware's own detection -----------------------------------------

    @staticmethod
    def _firmware_evidence(log: FlightLog) -> dict | None:
        times: list[float] = []
        motors: list[int] = []
        texts: list[str] = []
        for rec in log.get("MSG"):
            text = rec.get("Message")
            if not isinstance(text, str):
                continue
            m = THRUST_LOSS_MSG.match(text.strip())
            if m:
                motors.append(int(m.group(1)))
                texts.append(text.strip())
                if isinstance(rec.get("timestamp"), (int, float)):
                    times.append(float(rec["timestamp"]))
        errs = [
            float(t)
            for r in log.get("ERR")
            if r.get("Subsys") == ERR_THRUST_LOSS_CHECK
            and r.get("ECode")
            and isinstance(t := r.get("timestamp"), (int, float))
        ]
        t_mb, flags = log.series("MOTB", "FailFlags")
        boost = (flags.astype(int) & MOTB_THRUST_BOOST) > 0
        armed = log.armed_mask(t_mb)
        if armed is not None:
            boost &= armed
        boost_s = float(np.sum(np.diff(t_mb)[boost[:-1]])) if t_mb.size > 1 else 0.0
        if not texts and not errs and not boost.any():
            return None
        all_t = times + errs + ([float(t_mb[boost][0])] if boost.any() else [])
        return {
            "start": min(all_t) if all_t else None,
            "end": float(t_mb[boost][-1]) if boost.any() else (max(all_t) if all_t else None),
            "motors": sorted(set(motors)),
            "text": texts[0] if texts else None,
            "messages": len(texts),
            "errors": len(errs),
            "boost_s": boost_s,
        }

    def _firmware_finding(self, log: FlightLog, fw: dict, sig: dict | None) -> Finding:
        named = fw["motors"]
        motor_txt = f" on motor {', '.join(str(m) for m in named)}" if named else ""
        wired = {m: configured_channel(log, m) for m in named}
        outputs = ", ".join(f"motor {m} = output {c}" for m, c in wired.items() if c)
        assumed = ", ".join(f"motor {m} = output C{m}" for m, c in wired.items() if not c)
        parts = []
        if fw["text"]:
            parts.append(
                f'logged "{fw["text"]}"' + (f" ({fw['messages']} times)" if fw["messages"] > 1 else "")
            )
        if fw["errors"]:
            parts.append(f"wrote ERR THRUST_LOSS_CHECK {fw['errors']} time(s)")
        if fw["boost_s"] > 0:
            parts.append(f"ran in thrust-boost mode for {fw['boost_s']:.1f} s (MOTB.FailFlags bit 0)")
        when = f" at t={fw['start']:.1f}s" if fw["start"] is not None else ""
        explanation = (
            f"The autopilot's thrust-loss check fired{when}: it {', '.join(parts)}. It fires when the vehicle "
            f"sinks for {THRUST_LOSS_CHECK_TRIGGER_SEC:.0f} s with the throttle at "
            f"{THRUST_LOSS_CHECK_MINIMUM_THROTTLE:.0%} or more, near level and still under control: one motor "
            "is no longer producing its share of thrust."
            + (f" Wiring from SERVOn_FUNCTION: {outputs}." if outputs else "")
            + (
                f" No SERVOn_FUNCTION maps it, so the default output is assumed: {assumed}."
                if assumed
                else ""
            )
        )
        samples: dict[str, float] = {
            "messages": fw["messages"],
            "errors": fw["errors"],
            "thrust_boost_s": round(fw["boost_s"], 2),
        }
        if named:
            samples["motor"] = named[0]
        if sig is not None:
            agree = not named or sig["motor"] in named
            explanation += (
                f" The motor outputs show the same pattern on motor {sig['motor']} ({sig['channel']} at maximum, "
                f"opposite motor {sig['opposite_txt']} lowest)"
                + ("." if agree else f", not on the motor the firmware named ({motor_txt.strip()}).")
            )
            samples["signature_motor"] = sig["motor"]
            samples["signature_s"] = round(sig["duration"], 2)
        return make_finding(
            self.id,
            Severity.CRITICAL,
            f"Thrust loss{motor_txt}",
            explanation,
            recommendation=(
                "Inspect that motor, its ESC, propeller (cracked, loose, wrong pitch) and wiring, and check the "
                "craft is not overweight for its motors. Confirm the motor order matches the frame type."
            ),
            time_start_s=_onset(fw["start"], sig),
            time_end_s=fw["end"],
            message_types=["MSG", "ERR", "MOTB"] + (["RCOU", "CTUN", "ATT"] if sig else []),
            samples=samples,
        )

    # -- the signature in the outputs -----------------------------------------

    def _signature_episodes(self, log: FlightLog) -> list[dict]:
        """Every window where one motor shows the thrust-loss output pattern for at least 1 s."""
        geo = frame_geometry(log)
        if geo is None or not log.has("RCOU") or not log.has("CTUN") or not log.has("ATT"):
            return []
        n = len(geo)
        chans = [motor_channel(log, k + 1) for k in range(n)]
        if not set(chans) <= set(log.fields("RCOU")):
            return []  # a motor's output is not logged: no motor can be judged against the rest
        t, cols = log.aligned("RCOU", chans)
        if t.size < 2:
            return []
        pmin, pmax = pwm_range(log)
        spin_max = float(log.params.get("MOT_SPIN_MAX", SPIN_MAX_DEFAULT))
        act = np.vstack([(cols[c] - pmin) / (pmax - pmin) for c in chans])  # motors x samples

        tc, cc = log.aligned("CTUN", ["ThI", "ThO", "CRt"])
        ta, aa = log.aligned("ATT", ["DesRoll", "Roll", "DesPitch", "Pitch"])
        thi, tho, crt = (_hold(tc, cc[k], t) for k in ("ThI", "ThO", "CRt"))
        dr, r, dp, p = (_hold(ta, aa[k], t) for k in ("DesRoll", "Roll", "DesPitch", "Pitch"))
        with np.errstate(invalid="ignore"):
            thr = np.fmax(thi, tho)
            gate = (
                (thr >= THRUST_LOSS_CHECK_MINIMUM_THROTTLE)
                & (thi >= THRUST_LOSS_CHECK_LOW_THROTTLE)
                & (crt < 0)
                & (np.hypot(dr, dp) <= THRUST_LOSS_CHECK_ANGLE_DEVIATION_DEG)
                & (np.hypot(r - dr, p - dp) < CRASH_CHECK_ANGLE_DEVIATION_DEG)
            )
        flying = log.flying_mask(t)
        if flying is not None:
            gate &= flying

        episodes: list[dict] = []
        for k in range(n):
            group = same_side(geo, k)
            if group[0] != k:
                continue  # each coaxial group is judged once, from its first motor
            opp = opposite_side(geo, k)
            if not opp:
                continue
            rest = [j for j in range(n) if j not in group]
            others = [j for j in rest if j not in opp]
            with np.errstate(invalid="ignore"):
                sat = act[group].max(axis=0) >= spin_max - SATURATED_MARGIN
                room = act[rest].max(axis=0) <= spin_max - HEADROOM
                opp_mean = act[opp].mean(axis=0)
                lowest = (
                    opp_mean <= np.median(act[others], axis=0) if others else np.ones(t.shape, dtype=bool)
                )
            mask = gate & sat & room & lowest
            for iv in intervals_above(t, mask.astype(float), 0.5, THRUST_LOSS_CHECK_TRIGGER_SEC):
                episodes.append(
                    {
                        "motor": k + 1,
                        "channel": chans[k],
                        "opposite_txt": "/".join(str(j + 1) for j in opp),
                        "start": iv.start_s,
                        "end": iv.end_s,
                        "duration": iv.duration_s,
                    }
                )
        return episodes

    def _signature_finding(self, sig: dict) -> Finding:
        return make_finding(
            self.id,
            Severity.WARN,
            f"Thrust-loss pattern on motor {sig['motor']}",
            (
                f"For {sig['duration']:.1f} s from t={sig['start']:.1f}s motor {sig['motor']} ({sig['channel']}) was at "
                f"maximum output while the other motors had headroom and the opposite motor "
                f"({sig['opposite_txt']}) was the lowest, and the vehicle sank with the throttle at "
                f"{THRUST_LOSS_CHECK_MINIMUM_THROTTLE:.0%} or more, near level and under control. These are the "
                "firmware's own thrust-loss conditions; the firmware did not log a detection (the check may be "
                "disabled by FLIGHT_OPTIONS, or this firmware predates it)."
            ),
            recommendation=(
                f"Inspect motor {sig['motor']}, its ESC and propeller; confirm the motor order matches the frame "
                "type and the craft is not overweight."
            ),
            time_start_s=sig["start"],
            time_end_s=sig["end"],
            message_types=["RCOU", "CTUN", "ATT"],
            samples={"motor": sig["motor"], "signature_s": round(sig["duration"], 2)},
        )
