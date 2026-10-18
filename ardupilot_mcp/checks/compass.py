"""Compass / magnetometer check.

Reads the ``MAG`` message: per-axis magnetic field (MagX/Y/Z, milligauss). A
healthy magnetometer sees a roughly constant total field magnitude regardless of
throttle/heading, because the Earth's field is what dominates. When motor wiring
or current draw bleeds into the compass, the measured magnitude swings with
throttle — the classic "compass interference" failure that corrupts the EKF's
yaw estimate and causes toilet-bowling / fly-aways.

We quantify that swing with the magnitude of ``mag = sqrt(MagX^2+MagY^2+MagZ^2)``:
its coefficient of variation (std/mean) grades the field; the peak-to-peak spread
relative to the mean is reported as evidence only (one glitched sample sets it). We also flag implausibly large
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
# Peak-to-peak spread (max-min)/mean is reported as evidence only. It is not a trigger:
# one glitched sample sets it, and on 154 compass instances from 100 forum logs it
# exceeded 0.60 on 55 (36 %) while CV > 0.30 did so on 10 (6 %); none of the labelled
# flights among the 55 was a compass fault (bench/real_crashes.json).
# Need a meaningful number of samples before the statistics mean anything.
MIN_SAMPLES = 10
# CompassMot interference bands (% field change at full throttle): <30 acceptable,
# 30-60 grey zone, >60 unacceptable.
# https://ardupilot.org/copter/docs/common-compass-setup-advanced.html
INTERFERENCE_GREY_PCT = 30.0
INTERFERENCE_BAD_PCT = 60.0
# Design choice: the armed throttle must span at least this much (0..1) for a slope.
MIN_THROTTLE_SPAN = 0.15
# Vehicles whose CTUN.ThO is a 0..1 throttle fraction (Copter/Heli: motors->get_throttle(),
# ArduCopter/Log.cpp). ArduPlane logs ThO in percent (-100..100, ArduPlane/Log.cpp), so
# the per-unit-throttle slope would be 100x too small there; None = unknown/synthetic log.
THROTTLE_FRACTION_KINDS = {"copter", "heli", None}


@register_check
class CompassCheck(Check):
    id = "compass"
    title = "Compass / magnetometer"
    category = "compass"
    requires = {"MAG"}
    description = (
        "Per-compass field-strength stability and throttle-correlated interference from MAG + CTUN "
        "(offsets are covered by the calibration check)."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        # Each compass has its own field magnitude (an external mag far from the
        # wiring reads differently to the internal one), so interleaving them would
        # turn two perfectly stable compasses into a fake "unstable field".
        insts = log.instances("MAG")
        findings: list[Finding] = []
        for inst in insts:
            for f in self._stability_findings(log, inst) + self._throttle_findings(log, inst):
                if len(insts) > 1:
                    f.title += f" (compass {inst + 1})"
                    f.explanation = f"Compass {inst + 1}: " + f.explanation
                    f.evidence.samples["compass_instance"] = inst
                findings.append(f)
        return findings

    def _throttle_findings(self, log: FlightLog, inst: int = 0) -> list[Finding]:
        """Estimate motor/current interference as the field change per unit throttle.

        Fits |B| = a + b * throttle over armed samples (CTUN.ThO interpolated onto
        MAG times) and reports b/a as the % change of the field at full throttle — the
        quantity Mission Planner's CompassMot shows. Only the magnitude is visible in a
        log, so this underestimates vector interference: a lower bound, never inflated.
        """
        if log.meta.vehicle_kind not in THROTTLE_FRACTION_KINDS:
            return []
        tm, mx = log.series("MAG", "MagX", inst)
        _, my = log.series("MAG", "MagY", inst)
        _, mz = log.series("MAG", "MagZ", inst)
        tt, thr = log.series("CTUN", "ThO")
        n = min(tm.size, mx.size, my.size, mz.size)
        if n < MIN_SAMPLES or thr.size < MIN_SAMPLES:
            return []
        tm, mag = tm[:n], np.sqrt(mx[:n] ** 2 + my[:n] ** 2 + mz[:n] ** 2)
        armed = log.armed_mask(tm)
        if armed is not None:
            tm, mag = tm[armed], mag[armed]
        inside = (tm >= tt[0]) & (tm <= tt[-1])
        tm, mag = tm[inside], mag[inside]
        if mag.size < MIN_SAMPLES:
            return []
        th = np.interp(tm, tt, thr)
        if float(np.ptp(th)) < MIN_THROTTLE_SPAN:
            return []  # throttle barely moved: the slope would be noise
        slope, intercept = np.polyfit(th, mag, 1)
        if intercept <= 0.0:
            return []
        pct = 100.0 * float(slope) / float(intercept)
        if abs(pct) <= INTERFERENCE_GREY_PCT:
            return []
        sev = Severity.WARN if abs(pct) > INTERFERENCE_BAD_PCT else Severity.INFO
        return [
            make_finding(
                self.id,
                sev,
                "Compass interference from motors/current",
                f"The magnetic field strength moves with throttle: about {pct:+.0f}% at full throttle relative to "
                f"the zero-throttle field ({intercept:.0f} mGauss), fitted over {mag.size} armed samples. "
                f"ArduPilot's CompassMot guidance: under {INTERFERENCE_GREY_PCT:.0f}% is acceptable, "
                f"{INTERFERENCE_GREY_PCT:.0f}-{INTERFERENCE_BAD_PCT:.0f}% is a grey zone, above "
                f"{INTERFERENCE_BAD_PCT:.0f}% is unacceptable. This estimate uses field magnitude only, so the real "
                "interference is at least this large.",
                recommendation=(
                    "Move the compass away from power wiring/ESCs (an external GPS/compass mast helps) and run "
                    "CompassMot calibration with a current sensor."
                ),
                time_start_s=float(tm[0]),
                time_end_s=float(tm[-1]),
                message_types=["MAG", "CTUN"],
                samples={
                    "interference_pct": round(pct, 1),
                    "field_at_zero_throttle": round(float(intercept), 1),
                    "throttle_span": round(float(np.ptp(th)), 3),
                },
            )
        ]

    def _stability_findings(self, log: FlightLog, inst: int = 0) -> list[Finding]:
        tx, mx = log.series("MAG", "MagX", inst)
        _, my = log.series("MAG", "MagY", inst)
        _, mz = log.series("MAG", "MagZ", inst)
        n = min(mx.size, my.size, mz.size)
        if n < MIN_SAMPLES:
            return []
        mx, my, mz = mx[:n], my[:n], mz[:n]
        times = tx[:n] if tx.size >= n else tx

        mag = np.sqrt(mx * mx + my * my + mz * mz)
        # In flight only: on the ground the craft is carried, set down next to cars and
        # metal, or powered up; none of that is in-flight compass behaviour.
        flying = log.flying_mask(times)
        if flying is not None:
            times, mag = times[flying], mag[flying]
            if mag.size < MIN_SAMPLES:
                return []
        mean = float(np.mean(mag))
        if mean <= 0.0:
            # Degenerate (all-zero / no field): can't form a ratio. Nothing to say.
            return []
        std = float(np.std(mag))
        vmin = safe_min(mag) or 0.0
        vmax = safe_max(mag) or 0.0
        cv = std / mean
        range_ratio = (vmax - vmin) / mean

        if cv <= WARN_CV:
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
