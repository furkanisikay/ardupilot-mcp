# SPDX-License-Identifier: AGPL-3.0-only
"""Circling while holding position ("toilet bowling").

In a position-hold mode the position controller logs its target and the estimated
position (``PSCN``/``PSCE``: TPN/PN and TPE/PE, Copter-4.6.3 AC_PosControl). With a heading
error the controller pushes in a direction rotated from the one it wants, so the vehicle
misses the target sideways, corrects, misses again, and orbits it: the position error
vector keeps turning the same way. The wiki: "The vehicle circles (aka 'toiletbowls'). This
is normally caused by a compass problem the most likely being magnetic interference from
the power cables under the autopilot" (Loiter Mode, Common Problems), and the compass is
the primary heading source whose error "can lead to circling (aka 'toiletbowling')"
(Advanced Compass Setup).

The check counts how far the error vector turns while it is larger than a position
noise floor, in each LOITER / POSHOLD (Copter) or QLOITER (QuadPlane) stretch.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# Hold modes by vehicle (ardupilot_meta.COPTER_MODES / PLANE_MODES).
HOLD_MODES = {"copter": {5, 16}, "plane": {19}}  # LOITER, POSHOLD; QLOITER
# Design choice: the error vector's turning is counted only while the error exceeds this,
# so position noise around the target (a few tens of cm) adds no rotation.
ERROR_FLOOR_M = 0.5
# Design choice: two full turns in one direction is an orbit, not wandering...
MIN_TURNS = 2.0
# ...when at least this share of all the turning went that way.
MIN_CONSISTENCY = 0.75
# A hold stretch shorter than this (after the mode change settles) is not judged.
MIN_HOLD_S = 10.0
SETTLE_S = 2.0
# Design choice: while the hold target itself moves faster than this (the pilot repositioning
# with the sticks, flying circles in LOITER), the vehicle trails a moving point and its error
# turns with the target's path; those samples are not counted. In the QLOITER toilet bowl
# below the target never moved faster than 2 m/s.
TARGET_SPEED_MAX = 2.0
# Validation: on the 48 benchmark logs (34 hold stretches) the largest net turning above
# 0.5 m was 0.8 turns; a QuadPlane that toilet-bowled in QLOITER after a fan was fitted
# under its compass turned 3.8 times, and 0.2-0.3 times without the fan (forum thread 140262).


@register_check
class CirclingCheck(Check):
    id = "circling"
    title = "Circling in position hold"
    category = "compass"
    requires = {"PSCN", "PSCE", "MODE"}
    vehicles = {"copter", "plane"}
    description = (
        "Toilet bowling: the position error turning the same way, orbit after orbit, in LOITER / POSHOLD / "
        "QLOITER, the circling a heading (compass) error causes."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        modes = HOLD_MODES.get(log.meta.vehicle_kind or "copter", HOLD_MODES["copter"])
        tn, (tpn, pn, tvn) = log.series_many("PSCN", ["TPN", "PN", "TVN"])
        te, (tpe, pe, tve) = log.series_many("PSCE", ["TPE", "PE", "TVE"])
        n = min(tn.size, te.size)
        if n < 50:
            return []
        t = tn[:n]
        # PSCN and PSCE are written together each cycle; pair them by index.
        err_n, err_e = (pn - tpn)[:n], (pe - tpe)[:n]
        target_speed = np.hypot(tvn[:n], tve[:n])
        # Without a logged target velocity (NaN), the target is taken as still.
        target_still = ~(target_speed > TARGET_SPEED_MAX)
        ok = np.isfinite(t) & np.isfinite(err_n) & np.isfinite(err_e)
        hold = log.mode_mask(t, modes) & ok
        flying = log.flying_mask(t)
        if flying is not None:
            hold &= flying
        best: tuple[float, float, float, float, float] | None = None  # turns, consistency, t0, t1, radius
        for i0, i1 in _runs(hold):
            seg = slice(i0, i1)
            ts = t[seg]
            if ts.size < 2 or ts[-1] - ts[0] < MIN_HOLD_S:
                continue
            keep = ts >= ts[0] + SETTLE_S
            ts, en, ee = ts[keep], err_n[seg][keep], err_e[seg][keep]
            still = target_still[seg][keep]
            r = np.hypot(en, ee)
            ang = np.unwrap(np.arctan2(ee, en))
            big = (r[1:] > ERROR_FLOOR_M) & (r[:-1] > ERROR_FLOOR_M) & still[1:] & still[:-1]
            d = np.diff(ang)[big]
            if d.size == 0:
                continue
            net = float(np.sum(d)) / (2 * np.pi)
            total = float(np.sum(np.abs(d))) / (2 * np.pi)
            consistency = abs(net) / total if total > 0 else 0.0
            if abs(net) >= MIN_TURNS and consistency >= MIN_CONSISTENCY:
                radius = float(np.median(r[1:][big]))
                if best is None or abs(net) > abs(best[0]):
                    best = (net, consistency, float(ts[0]), float(ts[-1]), radius)
        if best is None:
            return []
        net, consistency, t0, t1, radius = best
        direction = "clockwise" if net > 0 else "counter-clockwise"
        return [
            make_finding(
                self.id,
                Severity.WARN,
                "Circling while holding position (toilet bowl)",
                f"Between t={t0:.0f}s and t={t1:.0f}s in a position-hold mode, with the hold target still "
                f"(moving under {TARGET_SPEED_MAX:.0f} m/s), the position error (PSCN/PSCE, position minus target) turned {abs(net):.1f} times {direction} around the target, "
                f"{consistency * 100:.0f}% of its turning in that one direction, at a median {radius:.1f} m off. "
                "The vehicle was orbiting the point it tried to hold. The wiki ties this circling to a compass "
                "problem, most likely magnetic interference from power cables under the autopilot; a heading "
                "error makes every correction point the wrong way.",
                recommendation=(
                    "Check the compass: interference (COMPASS_MOT / compassmot, power wiring and ESCs near it), "
                    "calibration and COMPASS_ORIENT; an external GPS+compass on a mast usually resolves it."
                ),
                time_start_s=t0,
                time_end_s=t1,
                message_types=["PSCN", "PSCE"],
                samples={
                    "turns": round(abs(net), 1),
                    "consistency": round(consistency, 2),
                    "median_error_m": round(radius, 2),
                },
            )
        ]


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """[start, end) index ranges where ``mask`` is True."""
    if mask.size == 0:
        return []
    edges = np.diff(np.concatenate(([0], mask.astype(np.int8), [0])))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)
    return list(zip(starts.tolist(), ends.tolist(), strict=True))
