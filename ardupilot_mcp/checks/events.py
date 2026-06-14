"""Errors, failsafes & mode/event timeline check.

Reconstructs the safety-relevant event narrative of a flight from three
DataFlash message families:

* ``ERR``  — subsystem errors and failsafes (fields ``Subsys``, ``ECode``). A
  non-zero ``ECode`` is an error *onset*; ``ECode == 0`` is that error
  *clearing*. Each onset becomes one finding, CRITICAL when the subsystem is in
  :data:`CRITICAL_ERR_SUBSYSTEMS` (battery/GPS/EKF/crash failsafes, etc.),
  otherwise WARN.
* ``MODE`` — flight-mode changes (fields ``Mode`` name / ``ModeNum``).
  Summarised as a single INFO timeline with consecutive duplicates collapsed.
* ``EV``   — discrete events (field ``Id``). Summarised as a single INFO note
  counting the key lifecycle events (ARMED / AUTO_ARMED / DISARMED / LAND).

This check always runs (``requires`` is empty) and guards each message type
individually, so it contributes whatever subset of the timeline is present and
returns ``[]`` when the log carries none of ERR/MODE/EV.
"""

from __future__ import annotations

from ..ardupilot_meta import (
    CRITICAL_ERR_SUBSYSTEMS,
    err_subsystem_name,
    ev_name,
    mode_name,
)
from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# EV names that mark the key lifecycle of a flight (decoded via ev_name).
# Verified against ArduPilot's LogEvent enum (AP_Logger.h, Copter-4.5.7): there is
# no TAKEOFF event id, so it is intentionally not listed here.
KEY_EVENT_NAMES = {
    "ARMED",
    "DISARMED",
    "AUTO_ARMED",
    "LAND_COMPLETE",
    "LAND_COMPLETE_MAYBE",
}


@register_check
class EventsCheck(Check):
    id = "events"
    title = "Errors & events"
    category = "events"
    requires: set[str] = set()  # always runs; each message type is guarded inside run()
    description = "Errors, failsafes and the flight mode/event timeline from ERR/MODE/EV."

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        findings.extend(self._err_findings(log))
        findings.extend(self._mode_finding(log))
        findings.extend(self._event_finding(log))
        return findings

    # -- ERR: one finding per error onset (ECode != 0) -----------------------

    def _err_findings(self, log: FlightLog) -> list[Finding]:
        out: list[Finding] = []
        if not log.has("ERR"):
            return out
        for rec in log.get("ERR"):
            if "Subsys" not in rec or "ECode" not in rec:
                continue
            subsys = int(rec["Subsys"])
            ecode = int(rec["ECode"])
            if ecode == 0:
                # ECode 0 clears a previously raised error -- not a finding.
                continue
            name = err_subsystem_name(subsys)
            critical = subsys in CRITICAL_ERR_SUBSYSTEMS
            sev = Severity.CRITICAL if critical else Severity.WARN
            ts = float(rec["timestamp"]) if "timestamp" in rec else None
            kind = "failsafe/critical fault" if critical else "subsystem error"
            out.append(
                make_finding(
                    self.id,
                    sev,
                    f"Error: {name} (ECode {ecode})",
                    f"ERR logged a {kind} on subsystem {name} (Subsys {subsys}) with "
                    f"ECode {ecode}"
                    + (f" at {ts:.1f}s." if ts is not None else ".")
                    + (
                        " This is a safety-critical failsafe/fault that directly affects flight safety."
                        if critical
                        else " Review the surrounding telemetry to understand the cause."
                    ),
                    recommendation=(
                        "Investigate the root cause before flying again; a critical failsafe "
                        "indicates the vehicle hit a hard safety limit."
                        if critical
                        else "Check the relevant subsystem (sensor/radio) for the cause of this error."
                    ),
                    time_start_s=ts,
                    message_types=["ERR"],
                    samples={"Subsys": subsys, "ECode": ecode},
                )
            )
        return out

    # -- MODE: one INFO finding summarising the mode timeline -----------------

    def _mode_finding(self, log: FlightLog) -> list[Finding]:
        if not log.has("MODE"):
            return []
        names: list[str] = []
        first_ts: float | None = None
        last_ts: float | None = None
        for rec in log.get("MODE"):
            label = self._mode_label(rec)
            if label is None:
                continue
            ts = float(rec["timestamp"]) if "timestamp" in rec else None
            if ts is not None:
                if first_ts is None:
                    first_ts = ts
                last_ts = ts
            # Dedupe consecutive duplicates.
            if not names or names[-1] != label:
                names.append(label)
        if not names:
            return []
        timeline = " -> ".join(names)
        return [
            make_finding(
                self.id,
                Severity.INFO,
                "Flight mode timeline",
                f"Flight modes: {timeline} "
                f"({len(names)} distinct consecutive mode(s) across {log.count('MODE')} MODE record(s)).",
                time_start_s=first_ts,
                time_end_s=last_ts,
                message_types=["MODE"],
                samples={"mode_changes": len(names), "mode_records": log.count("MODE")},
            )
        ]

    @staticmethod
    def _mode_label(rec: dict) -> str | None:
        """Decode a MODE record to a human-readable mode name.

        Prefer the numeric ``ModeNum`` decoded via the meta table; fall back to a
        string ``Mode`` field, then to the raw ``Mode`` number.
        """
        if "ModeNum" in rec and _is_number(rec["ModeNum"]):
            return mode_name(int(rec["ModeNum"]))
        if "Mode" in rec:
            val = rec["Mode"]
            if isinstance(val, str) and val:
                return val
            if _is_number(val):
                return mode_name(int(val))
        return None

    # -- EV: one INFO finding counting key lifecycle events -------------------

    def _event_finding(self, log: FlightLog) -> list[Finding]:
        if not log.has("EV"):
            return []
        key_counts: dict[str, int] = {}
        total = 0
        first_ts: float | None = None
        last_ts: float | None = None
        for rec in log.get("EV"):
            if "Id" not in rec or not _is_number(rec["Id"]):
                continue
            total += 1
            ts = float(rec["timestamp"]) if "timestamp" in rec else None
            if ts is not None:
                if first_ts is None:
                    first_ts = ts
                last_ts = ts
            name = ev_name(int(rec["Id"]))
            if name in KEY_EVENT_NAMES:
                key_counts[name] = key_counts.get(name, 0) + 1
        if total == 0:
            return []
        if key_counts:
            parts = ", ".join(f"{k}x{v}" for k, v in sorted(key_counts.items()))
            detail = f"Key lifecycle events: {parts}."
        else:
            detail = "No key lifecycle events (ARMED/DISARMED/LAND) recorded."
        samples: dict[str, float] = {"ev_count": total}
        for k, v in key_counts.items():
            samples[f"ev_{k}"] = v
        return [
            make_finding(
                self.id,
                Severity.INFO,
                "Flight events summary",
                f"EV logged {total} event(s). {detail}",
                time_start_s=first_ts,
                time_end_s=last_ts,
                message_types=["EV"],
                samples=samples,
            )
        ]


def _is_number(v: object) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)
