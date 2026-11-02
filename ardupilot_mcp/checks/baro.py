# SPDX-License-Identifier: AGPL-3.0-only
"""Barometer that follows the throttle (multirotor).

Prop wash or duct airflow over the barometer changes the pressure it reads with the thrust,
so its altitude rises and falls with the throttle while the vehicle's real height does not.
The EKF's height then follows the throttle, which the altitude controller answers with more
throttle: altitude-hold surges and bounces. Firmware 4.7 can subtract a thrust-proportional
pressure, ``BARO1_THST_SCALE`` ("This value scaled by the normalized thrust is subtracted from
the barometer pressure ... for local pressure difference induced by the props",
libraries/AP_Baro/AP_Baro.cpp, Copter-4.7.1, on boards built with AP_BARO_THST_COMP_ENABLED).

The check compares the barometer's altitude with the GPS altitude, an independent height,
removes their slow drift apart (a 30 s running median: weather, baro temperature, GPS bias),
and asks whether what is left follows the throttle (CTUN.ThO).
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

FIX_3D = 3
# Design choice: the slow drift between barometer and GPS altitude is the running median
# over this window; what is left is the fast, thrust-driven part.
DRIFT_WINDOW_S = 30.0
# Design choice: the residual follows the throttle when they correlate at least this well...
MIN_CORRELATION = 0.6
# ...and the barometer moves at least this much (m) between zero and full throttle...
MIN_SLOPE_M = 5.0
# ...over a flight whose throttle varied enough to tell (standard deviation of CTUN.ThO).
MIN_THROTTLE_STD = 0.03
MIN_SAMPLES = 100
# Validation: on the 48 benchmark logs the two flights of the cinewhoop whose thread found
# thrust-dependent barometer errors (145480) correlate 0.71 and 0.75 (14-16 m per full
# throttle); every other copter log is at most 0.47.


@register_check
class BaroThrustCheck(Check):
    id = "baro"
    title = "Barometer following the throttle"
    category = "sensors"
    requires = {"BARO", "GPS", "CTUN"}
    vehicles = {"copter"}
    description = (
        "Barometer altitude that rises and falls with the throttle against the GPS altitude: prop wash or "
        "duct airflow on the barometer, which makes altitude hold surge."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        tg, (galt, status) = log.series_many("GPS", ["Alt", "Status"], _first(log, "GPS"))
        tb, balt = log.series("BARO", "Alt", _first(log, "BARO"))
        tc, tho = log.series("CTUN", "ThO")
        if tg.size < MIN_SAMPLES or tb.size < MIN_SAMPLES or tc.size < MIN_SAMPLES:
            return []
        keep = (status >= FIX_3D) & np.isfinite(galt)
        flying = log.flying_mask(tg)
        if flying is not None:
            keep &= flying
        t, g = tg[keep], galt[keep]
        if t.size < MIN_SAMPLES:
            return []
        okb, okc = np.isfinite(balt), np.isfinite(tho)
        diff = np.interp(t, tb[okb], balt[okb]) - g
        throttle = np.interp(t, tc[okc], tho[okc])
        dt = float(np.median(np.diff(t)))
        win = max(3, int(round(DRIFT_WINDOW_S / dt))) if dt > 0 else 3
        if diff.size < win + 2:
            return []
        residual = diff - _running_median(diff, win)
        if float(np.std(throttle)) < MIN_THROTTLE_STD or float(np.std(residual)) <= 0:
            return []
        corr = float(np.corrcoef(residual, throttle)[0, 1])
        slope = float(np.polyfit(throttle, residual, 1)[0])
        if not np.isfinite(corr) or abs(corr) < MIN_CORRELATION or abs(slope) < MIN_SLOPE_M:
            return []
        way = "higher" if slope > 0 else "lower"
        return [
            make_finding(
                self.id,
                Severity.WARN,
                "Barometer altitude follows the throttle",
                f"Against the GPS altitude, the barometer read {abs(slope):.0f} m {way} at full throttle than at "
                f"zero: once their slow drift apart is removed, the difference correlates {corr:.2f} with the "
                "throttle (CTUN.ThO). Prop wash or duct airflow over the barometer changes the pressure it reads "
                "with thrust, so the height estimate follows the throttle and altitude hold surges or bounces.",
                recommendation=(
                    "Shield the barometer from the airflow (open-cell foam, a closed enclosure, away from props and "
                    "ducts). On firmware and boards that have BARO1_THST_SCALE, it subtracts a pressure "
                    "proportional to thrust."
                ),
                time_start_s=float(t[0]),
                time_end_s=float(t[-1]),
                message_types=["BARO", "GPS", "CTUN"],
                samples={
                    "correlation": round(corr, 2),
                    "metres_per_full_throttle": round(slope, 1),
                },
            )
        ]


def _first(log: FlightLog, mtype: str) -> int | None:
    insts = log.instances(mtype)
    return insts[0] if insts else None


def _running_median(x: np.ndarray, win: int) -> np.ndarray:
    from numpy.lib.stride_tricks import sliding_window_view

    padded = np.pad(x, (win // 2, win - win // 2 - 1), mode="edge")
    return np.median(sliding_window_view(padded, win), axis=1)
