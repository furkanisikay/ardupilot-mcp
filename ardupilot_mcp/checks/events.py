# SPDX-License-Identifier: AGPL-3.0-only
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

import math

import numpy as np

from ..ardupilot_meta import (
    ARM_METHOD_AUXSWITCH,
    ARM_METHODS,
    ARM_METHODS_COMMANDED,
    ARM_METHODS_GROUND_OR_CRASH,
    CRITICAL_ERR_SUBSYSTEMS,
    ERR_EKF_PRIMARY,
    ERR_FLIGHT_MODE,
    err_subsystem_name,
    ev_name,
    mode_name,
    stick_arm_switch,
)
from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding
from .vibration import trigger_note

# LogErrorSubsystem::FAILSAFE_VIBE (AP_Logger.h): the vibration compensation switching on.
ERR_FAILSAFE_VIBE = 29

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


# The flight state is read this long before the disarm record, since landing and the
# disarm can be written in the same instant (design choice).
INFLIGHT_LOOKBACK_S = 0.3
# A land event (EV LAND_COMPLETE_MAYBE / LAND_COMPLETE) this long before a disarm puts the
# disarm on the ground (design choice; the land detector settles within about a second).
LAND_EVENT_WINDOW_S = 5.0
# Records written by the disarm itself share its timestamp to within a scheduler tick.
SAME_INSTANT_S = 0.05


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
        findings.extend(self._inflight_disarms(log))
        return findings

    # -- ARM: a disarm while still flying -------------------------------------

    def _inflight_disarms(self, log: FlightLog) -> list[Finding]:
        """A disarm the land detector had not cleared: the motors stopped in the air.

        ``ARM.Method`` says who did it (AP_Arming::Method). A commanded one (switch, rudder,
        GCS, script) is reported as INFO: it is most often the pilot's emergency stop after
        control was already lost. One the autopilot made (a failsafe or termination) is a
        WARN, since stopping the motors in flight brings the vehicle down. Crash and normal
        landing disarms are not reported here.
        """
        out: list[Finding] = []
        if not log.has("ARM") or not log.flight_state_logged:
            return out  # without a land detector record, a landing's disarm looks like flight
        land_events = [
            float(r["timestamp"])
            for r in log.get("EV")
            if r.get("Id") in (17, 18) and _is_number(r.get("timestamp"))
        ]
        for rec in log.get("ARM"):
            state, m_raw, ts = rec.get("ArmState"), rec.get("Method"), rec.get("timestamp")
            if (
                not (isinstance(m_raw, (int, float)) and isinstance(ts, (int, float)) and _is_number(state))
                or state != 0
            ):
                continue
            method, t = int(m_raw), float(ts)
            if method in ARM_METHODS_GROUND_OR_CRASH:
                continue
            flying = log.flying_mask(np.array([t - INFLIGHT_LOOKBACK_S]))
            if flying is None or not bool(flying[0]):
                continue
            # Copter writes LAND_COMPLETE_MAYBE (17) as it settles on the ground, so a disarm
            # right after touchdown (the pilot's switch) still reads "flying" a moment before; a
            # land event in the seconds before it means it was on the ground. Every disarm,
            # one in the air included, also writes 17 and 18 at its own instant, because
            # disarming sets the land state (ArduCopter/land_detector.cpp); those are not counted.
            if any(t - LAND_EVENT_WINDOW_S <= lt < t - SAME_INSTANT_S for lt in land_events):
                continue
            name = ARM_METHODS.get(method, f"method {method}")
            on_stick = stick_arm_switch(log.params) if method == ARM_METHOD_AUXSWITCH else None
            commanded = method in ARM_METHODS_COMMANDED and on_stick is None
            msg = (
                f"The vehicle was disarmed by {name} (ARM.Method {method}) at t={t:.1f}s while it was still "
                "flying (the land detector had not reported a landing), so the motors stopped in the air."
            )
            if on_stick is not None:
                ch, stick, func = on_stick
                msg += (
                    f" The arm/disarm switch is the {stick} stick: RC{ch}_OPTION is {func}, so moving that stick "
                    "disarmed the vehicle."
                )
            else:
                msg += (
                    " A commanded disarm in flight is often the pilot's emergency stop after control was lost; if "
                    "nothing had gone wrong before it, check the arm/disarm switch (RCx_OPTION) and rudder-disarm "
                    "(ARMING_RUDDER) settings."
                    if commanded
                    else " The autopilot itself stopped the motors, which brings the vehicle down; check the "
                    "failsafe that chose this action."
                )
            out.append(
                make_finding(
                    self.id,
                    Severity.INFO if commanded else Severity.WARN,
                    f"Disarmed in flight ({name})",
                    msg,
                    time_start_s=t,
                    time_end_s=t,
                    message_types=["ARM"],
                    samples={"arm_method": float(method)},
                )
            )
        return out

    # -- ERR: one finding per error onset (ECode != 0) -----------------------

    def _err_findings(self, log: FlightLog) -> list[Finding]:
        out: list[Finding] = []
        if not log.has("ERR"):
            return out
        for rec in log.get("ERR"):
            if not (_is_finite(rec.get("Subsys")) and _is_finite(rec.get("ECode"))):
                continue
            subsys = int(rec["Subsys"])
            ecode = int(rec["ECode"])
            if subsys == ERR_EKF_PRIMARY:
                out.append(self._lane_switch(rec, ecode))
                continue
            if ecode == 0 and subsys != ERR_FLIGHT_MODE:
                # ECode 0 clears a previously raised error -- not a finding. FLIGHT_MODE is
                # the exception: its ECode is the mode, and mode 0 is STABILIZE / MANUAL.
                continue
            name = err_subsystem_name(subsys)
            critical = subsys in CRITICAL_ERR_SUBSYSTEMS
            sev = Severity.CRITICAL if critical else Severity.WARN
            ts = float(rec["timestamp"]) if "timestamp" in rec else None
            # On the ground and disarmed (the transmitter switched off after landing, a sensor
            # on the bench) a failsafe acts on nothing: kept as a note, not a fault. Only when
            # the log records arming, so an unknown state still counts as flight.
            disarmed = (
                ts is not None
                and subsys != ERR_FLIGHT_MODE
                and bool(log.armed_intervals() or log.arming_complete)
                and not log.was_armed_at(ts)
            )
            if disarmed:
                sev = Severity.INFO
            kind = "failsafe/critical fault" if critical else "subsystem error"
            detail = (
                f" The vehicle could not switch to {mode_name(ecode, log.meta.vehicle_kind)}."
                if subsys == ERR_FLIGHT_MODE
                else ""
            )
            out.append(
                make_finding(
                    self.id,
                    sev,
                    f"Error: {name} (ECode {ecode})",
                    f"ERR logged a {kind} on subsystem {name} (Subsys {subsys}) with "
                    f"ECode {ecode}"
                    + (f" at {ts:.1f}s." if ts is not None else ".")
                    + detail
                    + (trigger_note(log, ts) if subsys == ERR_FAILSAFE_VIBE else "")
                    + (
                        " The vehicle was disarmed at the time, so it acted on nothing in flight."
                        if disarmed
                        else ""
                    )
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

    def _lane_switch(self, rec: dict, core: int) -> Finding:
        ts = float(rec["timestamp"]) if "timestamp" in rec else None
        return make_finding(
            self.id,
            Severity.INFO,
            f"EKF primary core changed to {core}",
            "ERR EKF_PRIMARY records which EKF core became primary"
            + (f" at {ts:.1f}s" if ts is not None else "")
            + ": its ECode is the new core number, not an error. A switch follows the healthier "
            "core; repeated switches are worth reading next to the EKF findings.",
            time_start_s=ts,
            message_types=["ERR"],
            samples={"Subsys": ERR_EKF_PRIMARY, "core": core},
        )

    # -- MODE: one INFO finding summarising the mode timeline -----------------

    def _mode_finding(self, log: FlightLog) -> list[Finding]:
        if not log.has("MODE"):
            return []
        names: list[str] = []
        first_ts: float | None = None
        last_ts: float | None = None
        for rec in log.get("MODE"):
            label = self._mode_label(rec, log.meta.vehicle_kind)
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
    def _mode_label(rec: dict, kind: str | None = None) -> str | None:
        """Decode a MODE record to a human-readable mode name.

        Prefer the numeric ``ModeNum`` decoded via the meta table; fall back to a
        string ``Mode`` field, then to the raw ``Mode`` number.
        """
        if "ModeNum" in rec and _is_number(rec["ModeNum"]):
            return mode_name(int(rec["ModeNum"]), kind)
        if "Mode" in rec:
            val = rec["Mode"]
            if isinstance(val, str) and val:
                return val
            if _is_number(val):
                return mode_name(int(val), kind)
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


def _is_finite(v: object) -> bool:
    return _is_number(v) and math.isfinite(float(v))  # type: ignore[arg-type]
