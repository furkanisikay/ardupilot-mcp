"""Infer the aircraft's physical / architecture profile from a log.

Gives the LLM concrete configuration to reason about — and to combine with
real-world dimensions the user supplies (weight, prop diameter, motor KV). The
headline physical insight is the **power margin**: how much throttle headroom
there is above hover, which is what underpowered/overweight crashes come down to.
"""

from __future__ import annotations

import numpy as np

from . import ardupilot_docs as docs
from . import ardupilot_meta as meta
from .flight_log import FlightLog
from .model import VehicleProfile

PLAUSIBLE_MIN_V = 4.0
PLAUSIBLE_MAX_V = 70.0
# Multirotor hover-throttle interpretation (fraction 0..1).
HOVER_UNDERPOWERED = 0.62  # >= this: little headroom for climb/wind/maneuver
HOVER_OVERPOWERED = 0.30  # <= this: very overpowered


def _battery_source(log: FlightLog) -> str | None:
    for name in ("BAT", "CURR"):
        if log.has(name):
            return name
    return None


def _max_voltage(log: FlightLog) -> float | None:
    src = _battery_source(log)
    if not src:
        return None
    _, v = log.series(src, "Volt")
    if v.size == 0:
        return None
    mx = float(np.max(v))
    return mx if PLAUSIBLE_MIN_V <= mx <= PLAUSIBLE_MAX_V else None


def _hover_throttle(log: FlightLog) -> tuple[float | None, bool]:
    """(hover throttle 0..1, estimated?). Prefers the learned MOT_THST_HOVER."""
    h = log.params.get("MOT_THST_HOVER")
    if h is not None and 0.0 < float(h) <= 1.0:
        return float(h), False
    # Fallback: median throttle-out while clearly flying (rough).
    if log.has("CTUN"):
        _, tho = log.series("CTUN", "ThO")
        if tho.size:
            flying = tho[(tho > 0.1) & (tho < 0.95)]
            if flying.size > 20:
                return float(np.median(flying)), True
    return None, False


def build_profile(log: FlightLog) -> VehicleProfile:
    params = log.params
    kind = log.meta.vehicle_kind
    fc = params.get("FRAME_CLASS")
    ft = params.get("FRAME_TYPE")

    max_v = _max_voltage(log)
    cells = meta.estimate_cells(max_v)
    capacity = params.get("BATT_CAPACITY")
    hover, hover_estimated = _hover_throttle(log)
    power_margin = (1.0 - hover) * 100.0 if hover is not None else None
    t2w = (1.0 / hover) if (hover and hover > 0) else None
    motor_count = meta.motor_count_for_frame(fc)
    fcn = meta.frame_class_name(fc)
    ftn = meta.frame_type_name(ft)
    nominal_v = cells * 3.7 if cells else None

    assessment: list[str] = []
    notes: list[str] = []

    if fcn:
        desc = fcn + (f" ({ftn})" if ftn else "")
        if motor_count:
            desc += f", {motor_count} motors"
        assessment.append(f"Airframe: {desc}.")
    if cells:
        bat = f"{cells}S battery (~{nominal_v:.1f} V nominal, peak {max_v:.1f} V)"
        if capacity:
            bat += f", {float(capacity):.0f} mAh"
        assessment.append(bat + ".")

    if hover is not None and kind in ("copter", "heli"):
        pct = hover * 100.0
        src = " (estimated from throttle, no MOT_THST_HOVER)" if hover_estimated else ""
        if hover >= HOVER_UNDERPOWERED:
            assessment.append(
                f"Underpowered/heavy: hovers at {pct:.0f}% throttle{src} — only {power_margin:.0f}% "
                f"headroom for climb, wind and maneuvers (rough thrust-to-weight ~{t2w:.1f}). Low "
                "headroom means the craft can run out of authority and lose attitude in a gust or hard turn."
            )
        elif hover <= HOVER_OVERPOWERED:
            assessment.append(
                f"Very overpowered: hovers at only {pct:.0f}% throttle{src} (rough thrust-to-weight "
                f"~{t2w:.1f}); plenty of authority but watch for twitchy/over-tuned response."
            )
        else:
            assessment.append(
                f"Healthy power margin: hovers at {pct:.0f}% throttle{src} (~{power_margin:.0f}% "
                f"headroom, rough thrust-to-weight ~{t2w:.1f})."
            )
    elif hover is None and kind == "copter":
        notes.append("Hover throttle unknown (no MOT_THST_HOVER and no usable CTUN.ThO).")

    if max_v is None and _battery_source(log) is None:
        notes.append("No battery monitor in this log; voltage/cell/power figures unavailable.")

    return VehicleProfile(
        path=log.path,
        vehicle_type=log.meta.vehicle_type,
        vehicle_kind=kind,
        firmware_version=log.meta.firmware_version,
        frame_class=int(fc) if fc is not None else None,
        frame_class_name=fcn,
        frame_type=int(ft) if ft is not None else None,
        frame_type_name=ftn,
        motor_count=motor_count,
        battery_cells=cells,
        battery_capacity_mah=float(capacity) if capacity else None,
        nominal_voltage_v=nominal_v,
        max_voltage_v=max_v,
        hover_throttle=hover,
        power_margin_pct=power_margin,
        thrust_to_weight_estimate=t2w,
        assessment=assessment,
        notes=notes,
        references=docs.baseline_references(kind),
    )
