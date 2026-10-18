"""Incident timeline: what went wrong first, and what followed the crash.

A crash log is full of consequences: after an impact the vibration spikes, the
attitude error explodes and the GPS drops out, and a severity-sorted report puts
those at the top. Forum helpers answer one question first: *what went wrong
first, and is the rest just the crash?* This module answers it deterministically
by ordering events in time. It never claims causation; it only says what came
first and what came after.

* **Impact**: the earliest of an ERR CRASH_CHECK onset (Subsys 12), the firmware's
  "Crash: Disarming" message, or the log ending while still armed.
* **Onset**: the earliest anomaly while armed and before the impact: a firmware
  verdict message, a failsafe-driven mode change (MODE.Rsn), an ERR onset, or the
  start of a WARN/CRITICAL finding.
* Findings that start at or after the impact are flagged ``after_impact``.

Sources: ArduCopter/crash_check.cpp, libraries/AP_Vehicle/ModeReason.h,
libraries/AP_Logger/AP_Logger.h (LogErrorSubsystem CRASH_CHECK = 12), Copter-4.6.3.
"""

from __future__ import annotations

from .ardupilot_meta import err_subsystem_name, ev_name, mode_name
from .checks.firmware import FAILSAFE_REASONS, verdict_for
from .flight_log import FlightLog
from .model import Finding, IncidentTimeline, Severity, TimelineEvent

ERR_CRASH_CHECK = 12  # LogErrorSubsystem::CRASH_CHECK
# EV ids worth showing as context on the timeline (AP_Logger.h LogEvent).
_CONTEXT_EVENTS = {10, 11, 15, 17, 18, 28, 54}
# Design choice: findings starting this close after the impact time are still
# "after impact" (the impact sample and its own aftermath share a timestamp).
IMPACT_SLACK_S = 0.0


def _num(v: object) -> float | None:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _in_armed(t: float, armed: list[tuple[float, float]]) -> bool:
    return not armed or any(a <= t <= b for a, b in armed)


# Checks whose findings describe the whole log (a statistic, a parameter, a boot-time
# message), not a moment in the flight. They are reported as conditions, never as onset.
_CONDITION_CHECKS = {"compass", "calibration", "config", "param_audit", "sensors", "prearm"}
# Design choice: a finding present from the start of the flight (window opening
# within the first CONDITION_START_SHARE of armed time) and covering at least
# CONDITION_SHARE of it is a flight-long condition, not an event.
CONDITION_SHARE = 0.5
CONDITION_START_SHARE = 0.05


def _is_condition(f: Finding, armed: list[tuple[float, float]], total: float) -> bool:
    if f.check_id in _CONDITION_CHECKS:
        return True
    t0, t1 = f.evidence.time_start_s, f.evidence.time_end_s
    if total <= 0 or t0 is None or t1 is None:
        return False
    first = armed[0][0] if armed else 0.0
    return t0 <= first + CONDITION_START_SHARE * total and (t1 - t0) >= CONDITION_SHARE * total


def log_ended_armed(log: FlightLog, armed: list[tuple[float, float]] | None = None) -> bool:
    """True when the last armed interval has no disarm: the recording stops mid-flight."""
    armed = log.armed_intervals() if armed is None else armed
    if not armed:
        return False
    start = armed[-1][0]
    disarms = [_num(r.get("timestamp")) for r in log.get("EV") if r.get("Id") == 11]
    disarms += [_num(r.get("timestamp")) for r in log.get("ARM") if r.get("ArmState") == 0]
    return not any(t is not None and t >= start for t in disarms)


def build_timeline(log: FlightLog, findings: list[Finding]) -> IncidentTimeline:
    armed = log.armed_intervals()
    events: list[TimelineEvent] = []
    anomalies: list[tuple[float, str, str]] = []  # (time, source, label)
    impacts: list[tuple[float, str]] = []

    for rec in log.get("MSG"):
        text, t = rec.get("Message"), _num(rec.get("timestamp"))
        if not isinstance(text, str) or t is None:
            continue
        v = verdict_for(text)
        if v is None:
            continue
        label = text.strip()
        events.append(TimelineEvent(time_s=t, source="firmware", label=label, severity=v.severity, phase=""))
        if label.startswith("Crash: Disarming"):
            impacts.append((t, f'firmware "{label}"'))
        elif _in_armed(t, armed):
            anomalies.append((t, "firmware", label))

    for rec in log.get("ERR"):
        sub, code, t = _num(rec.get("Subsys")), _num(rec.get("ECode")), _num(rec.get("timestamp"))
        if sub is None or code is None or t is None or code == 0:
            continue
        label = f"ERR {err_subsystem_name(int(sub))} (ECode {int(code)})"
        events.append(TimelineEvent(time_s=t, source="err", label=label, severity=Severity.WARN, phase=""))
        if int(sub) == ERR_CRASH_CHECK:
            impacts.append((t, label))
        elif _in_armed(t, armed):
            anomalies.append((t, "err", label))

    for rec in log.get("MODE"):
        t = _num(rec.get("timestamp"))
        if t is None:
            continue
        num = rec.get("ModeNum", rec.get("Mode"))
        if isinstance(num, str):
            name = mode_name(num)
        elif isinstance(num, (int, float)):
            name = mode_name(int(num), log.meta.vehicle_kind)
        else:
            name = "?"
        rsn = _num(rec.get("Rsn"))
        reason = FAILSAFE_REASONS.get(int(rsn)) if rsn is not None else None
        label = f"Mode -> {name}" + (f" (by {reason})" if reason else "")
        events.append(
            TimelineEvent(
                time_s=t, source="mode", label=label, severity=Severity.WARN if reason else None, phase=""
            )
        )
        if reason and _in_armed(t, armed):
            anomalies.append((t, "mode", label))

    for rec in log.get("EV"):
        eid, t = _num(rec.get("Id")), _num(rec.get("timestamp"))
        if eid is not None and t is not None and int(eid) in _CONTEXT_EVENTS:
            events.append(TimelineEvent(time_s=t, source="event", label=ev_name(int(eid)), phase=""))

    ended_armed = log_ended_armed(log, armed)
    if ended_armed and log.meta.duration_s is not None:
        impacts.append((float(log.meta.duration_s), "log ended while armed"))

    impact = min(impacts) if impacts else None
    impact_s = impact[0] if impact else None

    armed_total = sum(b - a for a, b in armed) if armed else (log.meta.duration_s or 0.0)
    conditions: list[str] = []
    for f in findings:
        t = f.evidence.time_start_s
        if t is None or f.severity == Severity.INFO:
            continue
        if impact_s is not None and t >= impact_s - IMPACT_SLACK_S and f.check_id != "integrity":
            f.after_impact = True
        if f.check_id != "integrity" and _is_condition(f, armed, armed_total):
            conditions.append(f.title)
            continue
        events.append(
            TimelineEvent(
                time_s=t, source=f"finding:{f.check_id}", label=f.title, severity=f.severity, phase=""
            )
        )
        if not f.after_impact and _in_armed(t, armed) and f.check_id not in ("integrity", "firmware"):
            anomalies.append((t, f.check_id, f.title))

    pre = [a for a in anomalies if impact_s is None or a[0] < impact_s]
    onset = min(pre) if pre else None
    onset_s = onset[0] if onset else None

    impact_label = impact[1] if impact else None
    for e in events:
        t = e.time_s
        if t is None:
            e.phase = "unknown"
        elif impact_s is not None and t == impact_s and impact_label is not None and e.label in impact_label:
            e.phase = "impact"
        elif impact_s is not None and t >= impact_s:
            e.phase = "after_impact"
        elif onset_s is None or t < onset_s:
            e.phase = "before_onset"
        elif t == onset_s:
            e.phase = "onset"
        else:
            e.phase = "after_onset"
    if impact_label == "log ended while armed":
        events.append(TimelineEvent(time_s=impact_s, source="log", label=impact_label, phase="impact"))
    order = {"before_onset": 0, "onset": 1, "after_onset": 2, "impact": 3, "after_impact": 4, "unknown": 5}
    events.sort(key=lambda e: (e.time_s if e.time_s is not None else float("inf"), order[e.phase]))

    caveats = [
        "Order in time is not proof of cause: an event listed first preceded the others, nothing more."
    ]
    if not armed:
        caveats.append(
            "Arming could not be determined from this log, so the whole log was treated as flight."
        )
    if impact is not None and impact[1] == "log ended while armed":
        caveats.append("The impact time is the end of the recording; the real event may be slightly earlier.")

    return IncidentTimeline(
        path=log.path,
        conditions=list(dict.fromkeys(conditions)),
        onset_s=onset_s,
        onset_basis=onset[2] if onset else None,
        impact_s=impact_s,
        impact_basis=impact[1] if impact else None,
        log_ended_armed=ended_armed,
        first_anomaly_check_id=onset[1] if onset else None,
        events=events,
        caveats=caveats,
    )
