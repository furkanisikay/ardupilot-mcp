# SPDX-License-Identifier: AGPL-3.0-only
"""Plane: a flare that did not slow the descent before touchdown.

The flare is the last stage of an automatic landing: the autopilot cuts the throttle and
raises the pitch so the aircraft sinks onto the ground at TECS_LAND_SINK. It starts when the
aircraft is within LAND_FLARE_ALT of the ground, or within LAND_FLARE_SEC of it at the sink
rate it has, and the firmware announces it as "Flare %.1fm sink=%.2f speed=%.1f dist=%.1f"
(AP_Landing_Slope::verify_land, libraries/AP_Landing/AP_Landing_Slope.cpp, Plane-4.6.3). The
right time to flare depends on the aircraft and is set by those two parameters (wiki,
Automatic Landing, "Flare").

For each announced flare the check follows the climb rate TECS logs (TECS.dh) to the moment
the descent stops, which is the touchdown, and compares the sink rate just before it with the
one at the flare. A flare that left most of the sink in place put the aircraft on the ground
at nearly its approach sink rate: a hard landing.
"""

from __future__ import annotations

import re

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

FLARE_RE = re.compile(r"^Flare (-?[\d.]+)m sink=(-?[\d.]+)")
# Design choice: the touchdown is looked for this long after the flare starts.
TOUCHDOWN_WINDOW_S = 10.0
# Design choice: a flare is judged only when it began with a real descent (m/s).
MIN_FLARE_SINK = 0.5
# Design choice: the flare did not slow the descent when the aircraft still sank at this
# share of the flare's starting sink rate when it reached the ground. Validation: 26 flares
# with a found touchdown in 23 Plane logs (the benchmark's 142129 and 21 forum logs). The 12
# with a touchdown jolt of 11 m/s^2 or less kept at most 0.31 of the sink; the 8 that kept 0.5
# or more all met the ground at 26-81 m/s^2. Four hard touchdowns (32-78 m/s^2) kept less than
# 0.5 and are missed: the rule sees only a sink rate the flare left in place.
SINK_KEPT_RATIO = 0.5
TECS_LAND_SINK_DEFAULT = 0.25  # AP_TECS.cpp LAND_SINK default (m/s)
GRAVITY = 9.80665


@register_check
class FlareCheck(Check):
    id = "landing"
    title = "Landing flare"
    category = "landing"
    requires = {"MSG", "TECS"}
    vehicles = {"plane"}
    description = (
        "Automatic-landing flares (the firmware's Flare message) that left most of the sink rate in place, "
        "so the aircraft met the ground at nearly its approach descent rate."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        t, dh = log.series("TECS", "dh")
        if t.size < 2:
            return []
        findings = []
        flares = _flares(log)
        last = max((f[0] for f in flares), default=None)
        for t_flare, height, sink in flares:
            if sink < MIN_FLARE_SINK:
                continue
            after = np.flatnonzero((t > t_flare) & (t <= t_flare + TOUCHDOWN_WINDOW_S))
            stop = next((int(k) for k in after if dh[k] >= 0), None)
            if stop is None or stop == 0:
                continue
            touchdown = float(t[stop])
            sink_td = -float(dh[stop - 1])
            ratio = sink_td / sink
            if ratio < SINK_KEPT_RATIO:
                continue
            f = self._finding(log, t_flare, height, sink, touchdown, sink_td, ratio)
            if t_flare == last:
                # The last landing in the log: the timeline takes its touchdown as the impact.
                f.evidence.samples["final_touchdown_s"] = round(touchdown, 2)
            findings.append(f)
        return findings

    def _finding(
        self,
        log: FlightLog,
        t_flare: float,
        height: float,
        sink: float,
        touchdown: float,
        sink_td: float,
        ratio: float,
    ) -> Finding:
        p = log.params
        land_sink = p.get("TECS_LAND_SINK", TECS_LAND_SINK_DEFAULT)
        flare_alt, flare_sec = p.get("LAND_FLARE_ALT"), p.get("LAND_FLARE_SEC")
        jolt = _touchdown_jolt(log, touchdown)
        settings = ", ".join(
            f"{name} {value:g}"
            for name, value in (("LAND_FLARE_ALT", flare_alt), ("LAND_FLARE_SEC", flare_sec))
            if value is not None
        )
        msg = (
            f"The flare began {height:.1f} m above the ground sinking {sink:.2f} m/s, {height / sink:.1f} s from it at that "
            f"rate{f' ({settings})' if settings else ''}. The aircraft reached the ground {touchdown - t_flare:.1f} s later still "
            f"sinking {sink_td:.2f} m/s, {ratio * 100:.0f}% of that rate, where the flare aims for TECS_LAND_SINK "
            f"{land_sink:g} m/s (TECS.dh)."
        )
        samples: dict[str, float] = {
            "flare_height_m": round(height, 1),
            "flare_sink_ms": round(sink, 2),
            "touchdown_s": round(touchdown, 2),
            "touchdown_sink_ms": round(sink_td, 2),
            "sink_kept_ratio": round(ratio, 2),
        }
        if jolt is not None:
            msg += f" The touchdown jolt was {jolt:.0f} m/s^2 ({jolt / GRAVITY:.1f} g, IMU.AccZ)."
            samples["touchdown_jolt_ms2"] = round(jolt, 1)
        return make_finding(
            self.id,
            Severity.WARN,
            "Flare did not slow the descent",
            msg,
            recommendation=(
                "The flare starts at LAND_FLARE_ALT, or LAND_FLARE_SEC before the ground at the current sink rate, and the "
                "right time depends on the aircraft: start it earlier (raise LAND_FLARE_ALT and LAND_FLARE_SEC together, "
                "keeping LAND_FLARE_ALT close to the height LAND_FLARE_SEC gives), so the pitch has time to arrest the sink, "
                "and check the aircraft reaches LAND_PITCH_DEG in the flare."
            ),
            time_start_s=t_flare,
            time_end_s=touchdown,
            message_types=["MSG", "TECS"],
            samples=samples,
        )


def _flares(log: FlightLog) -> list[tuple[float, float, float]]:
    """(time, height, sink rate) of each "Flare ..." announcement."""
    out = []
    for rec in log.get("MSG"):
        text, ts = rec.get("Message"), rec.get("timestamp")
        if not isinstance(text, str) or ts is None:
            continue
        m = FLARE_RE.match(text.strip())
        if m:
            out.append((float(ts), float(m.group(1)), float(m.group(2))))
    return out


def _touchdown_jolt(log: FlightLog, touchdown: float) -> float | None:
    """Largest vertical acceleration away from 1 g around the touchdown (first IMU)."""
    if not log.has("IMU"):
        return None
    inst = log.instances("IMU")
    ti, az = log.series("IMU", "AccZ", inst[0] if inst else None)
    w = (ti >= touchdown - 0.3) & (ti <= touchdown + 0.5) & np.isfinite(az)
    if not w.any():
        return None
    return float(np.max(np.abs(az[w] + GRAVITY)))
