# SPDX-License-Identifier: AGPL-3.0-only
"""Compare two logs of the same vehicle: what changed since the last good flight?

Most "it flew fine last week" regressions are a configuration change. This lists
parameter differences (configuration separately from values the autopilot learns
or counts by itself), the firmware change, and a few flight metrics side by side.
It reports differences only; it never judges which side is right.
"""

from __future__ import annotations

import numpy as np

from .flight_log import FlightLog
from .model import LogComparison, MetricPair, ParamChange

# Parameters the autopilot writes by itself (statistics, learned values, boot-time
# calibration): a change there is not a configuration change by the user.
# STAT_*: AP_Stats counters; MOT_THST_HOVER: learned when MOT_HOVER_LEARN is on;
# INS_GYR*OFFS*: gyro calibration at every boot; INS_ACC*_ID / INS_GYR*_ID /
# COMPASS_DEV_ID*: detected device ids; BARO*_GND_PRESS: ground pressure at boot;
# COMPASS_DEC: declination learned from GPS. Copter-4.6.3 sources.
_LEARNED_PREFIXES = ("STAT_", "INS_GYROFFS", "INS_GYR2OFFS", "INS_GYR3OFFS", "COMPASS_DEV_ID")
_LEARNED_NAMES = {"MOT_THST_HOVER", "INS_ACC_ID", "INS_ACC2_ID", "INS_ACC3_ID", "INS_GYR_ID", "INS_GYR2_ID",
                  "INS_GYR3_ID", "BARO1_GND_PRESS", "BARO2_GND_PRESS", "BARO3_GND_PRESS",
                  "COMPASS_DEC", "GND_ABS_PRESS"}  # fmt: skip
# Values that differ by less than this are float32 storage noise, not a change.
_REL_TOL = 1e-6


# INS_*_CALTEMP: IMU temperature at calibration time (AP_InertialSensor), written by calibration.
_LEARNED_SUFFIXES = ("_CALTEMP",)


def _is_learned(name: str) -> bool:
    return name in _LEARNED_NAMES or name.startswith(_LEARNED_PREFIXES) or name.endswith(_LEARNED_SUFFIXES)


def _primary_max(log: FlightLog, mtype: str, field: str) -> float | None:
    insts = log.instances(mtype)
    if not insts:
        return None
    _, v = log.series(mtype, field, insts[0])
    return float(np.max(v)) if v.size else None


def _armed_min(log: FlightLog, mtype: str, field: str) -> float | None:
    if not log.has(mtype):
        return None
    t, v = log.series(mtype, field, log.instances(mtype)[0])
    mask = log.armed_mask(t)
    if mask is not None:
        v = v[mask]
    return float(np.min(v)) if v.size else None


def _metrics(log: FlightLog) -> dict[str, float | None]:
    armed = log.armed_intervals()
    return {
        "duration_s": log.meta.duration_s,
        "armed_time_s": sum(b - a for a, b in armed) if armed else None,
        "max_altitude_m": log.meta.max_altitude_m,
        "hover_throttle": log.params.get("MOT_THST_HOVER"),
        "max_VibeX": _primary_max(log, "VIBE", "VibeX"),
        "max_VibeY": _primary_max(log, "VIBE", "VibeY"),
        "max_VibeZ": _primary_max(log, "VIBE", "VibeZ"),
        "min_battery_v_armed": _armed_min(log, "BAT", "Volt"),
        "min_board_vcc_armed": _armed_min(log, "POWR", "Vcc"),
    }


def _same(a: float, b: float) -> bool:
    return abs(a - b) <= _REL_TOL * max(1.0, abs(a), abs(b))


def compare_logs(a: FlightLog, b: FlightLog) -> LogComparison:
    pa, pb = a.params, b.params
    changed: list[ParamChange] = []
    learned: list[ParamChange] = []
    for name in sorted(set(pa) & set(pb)):
        if _same(pa[name], pb[name]):
            continue
        item = ParamChange(name=name, a=pa[name], b=pb[name])
        (learned if _is_learned(name) else changed).append(item)
    added = sorted(set(pb) - set(pa))
    removed = sorted(set(pa) - set(pb))
    ma, mb = _metrics(a), _metrics(b)
    notes: list[str] = []
    if a.meta.vehicle_type != b.meta.vehicle_type:
        notes.append(f"different vehicle types ({a.meta.vehicle_type} vs {b.meta.vehicle_type})")
    if added or removed:
        notes.append(
            "parameters present in only one log usually mean a firmware change or a feature switched on/off "
            "(new parameter groups appear when e.g. a second GPS or a rangefinder is enabled)"
        )
    if len(pa) < 150 or len(pb) < 150:
        notes.append("one log has a short parameter dump (truncated log?); the diff may be incomplete")
    return LogComparison(
        path_a=a.path,
        path_b=b.path,
        firmware_a=a.meta.firmware_version,
        firmware_b=b.meta.firmware_version,
        firmware_changed=a.meta.firmware_version != b.meta.firmware_version,
        changed=changed,
        learned_changed=learned,
        added=added,
        removed=removed,
        metrics={k: MetricPair(a=ma[k], b=mb[k]) for k in ma},
        notes=notes,
    )
