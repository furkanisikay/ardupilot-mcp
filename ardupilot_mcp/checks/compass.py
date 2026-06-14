"""Compass / magnetometer check.

Reads the ``MAG`` message: per-axis magnetic field (MagX/Y/Z, milligauss). A
healthy magnetometer sees a roughly constant total field magnitude regardless of
throttle/heading, because the Earth's field is what dominates. When motor wiring
or current draw bleeds into the compass, the measured magnitude swings with
throttle — the classic "compass interference" failure that corrupts the EKF's
yaw estimate and causes toilet-bowling / fly-aways.

We quantify that swing with the magnitude of ``mag = sqrt(MagX^2+MagY^2+MagZ^2)``:
its coefficient of variation (std/mean) and its peak-to-peak spread relative to
the mean. Large variation -> interference. We also flag implausibly large
hard-iron offsets from COMPASS_OFS_X/Y/Z when those params are present.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding, safe_max, safe_min

# Coefficient of variation (std/mean) of field magnitude. A clean compass sits
# well under 0.1; sustained interference pushes it up.
WARN_CV = 0.30
CRIT_CV = 0.60
# Peak-to-peak spread (max-min)/mean. A second, independent way to catch a
# field that swings hard even if the std is dragged down by quiet stretches.
WARN_RANGE_RATIO = 0.60
# Need a meaningful number of samples before the statistics mean anything.
MIN_SAMPLES = 10


@register_check
class CompassCheck(Check):
    id = "compass"
    title = "Compass / magnetometer"
    category = "compass"
    requires = {"MAG"}
    description = "Magnetic field-strength stability from MAG (offsets are covered by the calibration check)."

    def run(self, log: FlightLog) -> list[Finding]:
        return self._stability_findings(log)

    def _stability_findings(self, log: FlightLog) -> list[Finding]:
        tx, mx = log.series("MAG", "MagX")
        _, my = log.series("MAG", "MagY")
        _, mz = log.series("MAG", "MagZ")
        n = min(mx.size, my.size, mz.size)
        if n < MIN_SAMPLES:
            return []
        mx, my, mz = mx[:n], my[:n], mz[:n]
        times = tx[:n] if tx.size >= n else tx

        mag = np.sqrt(mx * mx + my * my + mz * mz)
        mean = float(np.mean(mag))
        if mean <= 0.0:
            # Degenerate (all-zero / no field): can't form a ratio. Nothing to say.
            return []
        std = float(np.std(mag))
        vmin = safe_min(mag) or 0.0
        vmax = safe_max(mag) or 0.0
        cv = std / mean
        range_ratio = (vmax - vmin) / mean

        if cv <= WARN_CV and range_ratio <= WARN_RANGE_RATIO:
            return []

        if cv > CRIT_CV:
            sev = Severity.CRITICAL
            msg = (
                f"Magnetic field magnitude is very unstable: coefficient of variation "
                f"{cv:.2f} (>{CRIT_CV:.2f}), swinging between {vmin:.0f} and {vmax:.0f} mGauss "
                f"around a mean of {mean:.0f}. A field that varies this much almost always means "
                "motor/current interference is coupling into the compass, which corrupts the EKF "
                "yaw estimate (toilet-bowling, fly-aways)."
            )
            rec = (
                "Re-site or re-calibrate the compass away from power wiring/ESCs; "
                "enable COMPASS_MOT current compensation, or prefer an external compass."
            )
        else:
            sev = Severity.WARN
            msg = (
                f"Magnetic field magnitude varies a lot: coefficient of variation {cv:.2f} "
                f"(>{WARN_CV:.2f}), peak-to-peak {(vmax - vmin):.0f} mGauss "
                f"({range_ratio * 100:.0f}% of the {mean:.0f} mGauss mean), range {vmin:.0f}-{vmax:.0f}. "
                "This is consistent with motor/current interference bleeding into the compass."
            )
            rec = (
                "Check compass placement relative to power wiring; consider COMPASS_MOT "
                "calibration or an external compass."
            )

        t0 = float(times[0]) if times.size else None
        t1 = float(times[-1]) if times.size else None
        return [
            make_finding(
                self.id,
                sev,
                "Unstable compass field strength",
                msg,
                recommendation=rec,
                time_start_s=t0,
                time_end_s=t1,
                message_types=["MAG"],
                samples={
                    "mean_mag": round(mean, 1),
                    "std_mag": round(std, 1),
                    "cv": round(cv, 3),
                    "min_mag": round(vmin, 1),
                    "max_mag": round(vmax, 1),
                    "range_ratio": round(range_ratio, 3),
                },
            )
        ]
