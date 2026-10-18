"""The firmware's own in-flight verdicts.

ArduPilot already runs detectors in flight and writes their conclusions into the
log: Copter's crash check announces a thrust loss on a specific motor, a yaw
imbalance, or a crash disarm; the EKF check announces variance, a failsafe or
vibration compensation; and every mode change records *why* it happened
(``MODE.Rsn``, a ModeReason). These are the most precise signals a log carries,
computed on board at full rate with the firmware's own thresholds, so this check
reports them as they are instead of re-deriving them. The thrust-loss message is
matched here (timeline, prearm dedupe) but reported by the ``thrust_loss`` check,
which adds the motor's output channel and the motor-output evidence.

Sources (Copter-4.6.3):
- ArduCopter/crash_check.cpp: "Potential Thrust Loss (%d)" (motor index + 1),
  "Yaw Imbalance %0.0f%%", "Crash: Disarming: ..."
- ArduCopter/ekf_check.cpp: "EKF variance", "EKF Failsafe: changed to %s Mode",
  "Vibration compensation ON"
- ArduCopter/events.cpp: "GPS Glitch or Compass error", "<prefix> started"/"timeout"
  for dead reckoning
- libraries/AP_Vehicle/ModeReason.h: the ModeReason values decoded below
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from ..ardupilot_meta import mode_name
from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding


@dataclass(frozen=True)
class Verdict:
    pattern: re.Pattern[str]
    severity: Severity
    title: str
    meaning: str
    # Check that reports this verdict instead of this one (it adds evidence the
    # message alone lacks); the message still feeds the timeline and prearm dedupe.
    reported_by: str | None = None


# Matched against the start of each MSG line (case-sensitive, as the firmware writes it).
VERDICTS: tuple[Verdict, ...] = (
    Verdict(
        re.compile(r"Potential Thrust Loss \((\d+)\)"),
        Severity.CRITICAL,
        "Firmware detected thrust loss",
        "the crash check saw a motor at full output while the vehicle still could not hold attitude "
        "or altitude; the number is the motor the firmware suspects (motor index + 1)",
        reported_by="thrust_loss",
    ),
    Verdict(
        re.compile(r"Crash: Disarming"),
        Severity.CRITICAL,
        "Firmware detected a crash and disarmed",
        "the crash check found a large attitude error with almost no acceleration, i.e. the vehicle was "
        "on the ground in the wrong attitude, and cut the motors",
    ),
    Verdict(
        re.compile(r"EKF Failsafe: changed to"),
        Severity.CRITICAL,
        "EKF failsafe changed the flight mode",
        "the EKF check judged the position estimate unusable and forced a non-GPS mode",
    ),
    # Copter::announce_failsafe (ArduCopter/events.cpp): "<type> Failsafe[ - <action>]". The
    # ERR and mode-change records say the same in an onboard log (and are merged with it);
    # in a telemetry log this line is the only record of the failsafe.
    Verdict(
        re.compile(r"Radio(?: \+ Battery)? Failsafe(?! Cleared)"),
        Severity.CRITICAL,
        "Firmware triggered a radio failsafe",
        "the autopilot lost valid RC input for longer than the failsafe timeout and took the action "
        "after the dash (FS_THR_ACTION)",
    ),
    Verdict(
        re.compile(r"Battery Failsafe"),
        Severity.CRITICAL,
        "Firmware triggered a battery failsafe",
        "a battery stayed below its low or critical threshold for BATT_LOW_TIMER and the autopilot "
        "took the action after the dash (BATT_FS_LOW_ACT / BATT_FS_CRT_ACT)",
    ),
    Verdict(
        re.compile(r"GCS(?: \+ Battery)? Failsafe(?! Cleared)"),
        Severity.WARN,
        "Firmware triggered a GCS failsafe",
        "the autopilot stopped hearing the ground station's heartbeat for FS_GCS_TIMEOUT and took the "
        "action after the dash (FS_GCS_ENABLE)",
    ),
    Verdict(
        re.compile(r"Yaw Imbalance (\d+)%"),
        Severity.WARN,
        "Firmware reported a yaw imbalance",
        "the yaw controller's integrator stayed high: motors are working against a constant yaw torque "
        "(twisted motor mount, wrong prop, one motor weaker)",
    ),
    Verdict(
        re.compile(r"Vibration compensation ON"),
        Severity.WARN,
        "Firmware switched on vibration compensation",
        "high vibration made the EKF's vertical estimate unreliable, so the firmware changed its "
        "altitude control to cope",
    ),
    Verdict(
        re.compile(r"EKF variance"),
        Severity.WARN,
        "Firmware reported EKF variance",
        "the EKF's innovations exceeded the EKF check thresholds",
    ),
    Verdict(
        re.compile(r"GPS Glitch or Compass error"),
        Severity.WARN,
        "Firmware reported a GPS glitch or compass error",
        "the position estimate jumped against GPS/compass",
    ),
    Verdict(
        re.compile(r"Dead Reckoning (started|timeout)", re.IGNORECASE),
        Severity.WARN,
        "Firmware fell back to dead reckoning",
        "the EKF lost its position source and estimated position without it",
    ),
)

# ModeReason values that mean "a failsafe or protection changed the mode", not the
# pilot or GCS. libraries/AP_Vehicle/ModeReason.h (Copter-4.6.3).
FAILSAFE_REASONS: dict[int, str] = {
    3: "radio failsafe",
    4: "battery failsafe",
    5: "GCS failsafe",
    6: "EKF failsafe",
    7: "GPS glitch",
    10: "fence breach",
    11: "terrain failsafe",
    17: "terminate",
    19: "crash failsafe",
    25: "failsafe",
    29: "leak failsafe",
    50: "dead-reckoning failsafe",
    51: "takeoff failsafe",
}


def verdict_for(text: str) -> Verdict | None:
    """The firmware verdict a MSG line announces, if any (used to keep checks from double-reporting)."""
    t = text.strip()
    return next((v for v in VERDICTS if v.pattern.match(t)), None)


@register_check
class FirmwareVerdictsCheck(Check):
    id = "firmware"
    title = "Firmware's own verdicts"
    category = "events"
    requires: set[str] = set()  # MSG and MODE are guarded inside run()
    description = (
        "In-flight conclusions the autopilot logged itself: crash disarm, EKF "
        "failsafe, yaw imbalance, vibration compensation, and failsafe-driven mode changes (MODE.Rsn)."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        return self._message_verdicts(log) + self._failsafe_modes(log)

    def _message_verdicts(self, log: FlightLog) -> list[Finding]:
        hits: dict[Verdict, list[tuple[float | None, str]]] = {}
        for rec in log.get("MSG"):
            text = rec.get("Message")
            if not isinstance(text, str):
                continue
            v = verdict_for(text)
            if v is not None:
                ts = rec.get("timestamp")
                hits.setdefault(v, []).append(
                    (float(ts) if isinstance(ts, (int, float)) else None, text.strip())
                )
        out: list[Finding] = []
        for v in VERDICTS:  # stable, severity-ordered output
            seen = hits.get(v)
            if not seen or v.reported_by is not None:
                continue
            times = [t for t, _ in seen if t is not None]
            first_text = seen[0][1]
            samples: dict[str, float] = {"occurrences": len(seen)}
            m = v.pattern.match(first_text)
            if m and m.groups() and m.group(1).isdigit():
                samples["value_pct"] = int(m.group(1))
            when = f" first at t={min(times):.1f}s" if times else ""
            out.append(
                make_finding(
                    self.id,
                    v.severity,
                    v.title,
                    f'The autopilot logged "{first_text}"{when}'
                    + (f" ({len(seen)} times)" if len(seen) > 1 else "")
                    + f": {v.meaning}. This is the firmware's own on-board conclusion, not an inference.",
                    recommendation=_recommendation(v),
                    time_start_s=min(times) if times else None,
                    time_end_s=max(times) if times else None,
                    message_types=["MSG"],
                    samples=samples,
                )
            )
        return out

    def _failsafe_modes(self, log: FlightLog) -> list[Finding]:
        by_reason: dict[int, list[tuple[float | None, str]]] = {}
        for rec in log.get("MODE"):
            rsn = rec.get("Rsn")
            if not isinstance(rsn, (int, float)) or int(rsn) not in FAILSAFE_REASONS:
                continue
            num = rec.get("ModeNum", rec.get("Mode"))
            if isinstance(num, str):
                name = mode_name(num)
            elif isinstance(num, (int, float)):
                name = mode_name(int(num), log.meta.vehicle_kind)
            else:
                name = "?"
            ts = rec.get("timestamp")
            by_reason.setdefault(int(rsn), []).append(
                (float(ts) if isinstance(ts, (int, float)) else None, name)
            )
        out: list[Finding] = []
        for rsn, events in sorted(by_reason.items()):
            label = FAILSAFE_REASONS[rsn]
            times = [t for t, _ in events if t is not None]
            modes = list(dict.fromkeys(m for _, m in events))
            sev = Severity.CRITICAL if rsn in (6, 19) else Severity.WARN
            out.append(
                make_finding(
                    self.id,
                    sev,
                    f"Mode changed by {label}",
                    f"The {label} took control of the flight mode {len(events)} time(s)"
                    + (f", first at t={min(times):.1f}s" if times else "")
                    + f" (to {', '.join(modes)}). MODE.Rsn={rsn} records that the firmware, not the pilot, "
                    "made this change.",
                    recommendation="Find what triggered the failsafe at that time (link, battery, EKF, fence) "
                    "before the next flight.",
                    time_start_s=min(times) if times else None,
                    time_end_s=max(times) if times else None,
                    message_types=["MODE"],
                    samples={"mode_reason": rsn, "occurrences": len(events)},
                )
            )
        return out


def _recommendation(v: Verdict) -> str:
    if "crash" in v.title.lower():
        return "Work backwards from the crash time: what failed first (see the incident timeline)."
    if "Yaw" in v.title:
        return "Check motor mounts are straight and props are on the right motors and the right way up."
    if "Vibration" in v.title:
        return "Reduce vibration (balance props, soft-mount the flight controller) before flying again."
    return "Check the sensors behind the estimate (GPS, compass, vibration) around that time."
