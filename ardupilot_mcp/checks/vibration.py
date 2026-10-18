"""Vibration check.

Reads the ``VIBE`` message: per-axis vibration (VibeX/Y/Z, m/s^2) and the
cumulative accelerometer-clip counter(s). Modern firmware (4.x) logs one VIBE
stream per IMU, distinguished by the ``IMU`` instance field, each with a single
``Clip`` counter ("TimeUS,IMU,VibeX,VibeY,VibeZ,Clip" in Copter-4.6.3
libraries/AP_InertialSensor/LogStructure.h). Older firmware logged one stream with
``Clip0/1/2`` (one counter per IMU). Both layouts are handled, per IMU. Per ArduPilot's vibration
guidance, sustained vibration below ~30 m/s^2 is generally acceptable, 30-60 may
cause problems, and >60 nearly always degrades position/altitude hold. Ideal
clipping is zero; a small count (<~100) is likely ok (especially around hard
landings), but a steadily increasing count means the IMU is railing — a serious
vibration problem. Accordingly the clip check warns on any clipping and escalates
to critical once the count is large (>=100).

Source: https://ardupilot.org/copter/docs/common-measuring-vibration.html
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import fraction_above, intervals_above, make_finding

WARN_MS2 = 30.0
CRIT_MS2 = 60.0
# Fraction of the flight above WARN that turns a marginal note into a warning.
SUSTAINED_FRACTION = 0.10
# Clip count at which clipping is critical: the wiki's "less than ~100 is likely ok".
CLIP_CRIT_COUNT = 100


@register_check
class VibrationCheck(Check):
    id = "vibration"
    title = "Vibration levels"
    category = "vibration"
    requires = {"VIBE"}
    description = "Per-axis vibration magnitude and accelerometer clipping from VIBE."

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        findings.extend(self._axis_findings(log))
        findings.extend(self._clip_findings(log))
        return findings

    def _axis_findings(self, log: FlightLog) -> list[Finding]:
        out: list[Finding] = []
        imus = log.instances("VIBE")
        primary = primary_imu(log, imus)
        for axis in ("VibeX", "VibeY", "VibeZ"):
            # Judge each IMU on its own samples: interleaving IMUs would mix a quiet,
            # isolated IMU with a noisy one.
            main = self._axis_finding(log, axis, primary, multi=len(imus) > 1)
            # Worst secondary IMU, one severity lower (see primary_imu): reported
            # only when it adds something beyond the primary's own finding.
            side: Finding | None = None
            for imu in imus:
                if imu == primary:
                    continue
                f = self._axis_finding(log, axis, imu, multi=True)
                if f is not None and (
                    side is None or f.evidence.samples[f"max_{axis}"] > side.evidence.samples[f"max_{axis}"]
                ):
                    side = f
            if main is not None:
                out.append(main)
            if (
                side is not None
                and demote(side)
                and (main is None or side.severity.rank > main.severity.rank)
            ):
                out.append(side)
        return out

    def _axis_finding(self, log: FlightLog, axis: str, imu: int, *, multi: bool) -> Finding | None:
        times, vals = log.series("VIBE", axis, imu)
        if vals.size == 0:
            return None
        vmax = float(np.max(vals))
        if vmax <= WARN_MS2:
            return None
        frac_warn = fraction_above(vals, WARN_MS2)
        frac_crit = fraction_above(vals, CRIT_MS2)
        where = f" on IMU{imu}" if multi else ""
        if frac_crit > 0.0 or vmax > CRIT_MS2:
            sev = Severity.CRITICAL
            msg = (
                f"{axis} vibration{where} peaks at {vmax:.0f} m/s^2 (>{CRIT_MS2:.0f}); "
                f"{frac_crit * 100:.0f}% of samples exceed {CRIT_MS2:.0f} m/s^2. "
                "Vibration this high corrupts the accelerometers and can trigger EKF failures."
            )
            rec: str | None = (
                "Improve flight-controller isolation/balance; check prop/motor condition before flying again."
            )
        elif frac_warn >= SUSTAINED_FRACTION:
            sev = Severity.WARN
            msg = (
                f"{axis} vibration{where} is elevated: peak {vmax:.0f} m/s^2, "
                f"{frac_warn * 100:.0f}% of the flight above {WARN_MS2:.0f} m/s^2."
            )
            rec = "Inspect mounting/balance; consider the harmonic notch filter (see recommend_tuning)."
        else:
            sev = Severity.INFO
            msg = (
                f"{axis} vibration{where} is mostly fine but peaks at {vmax:.0f} m/s^2 "
                f"(brief excursions above {WARN_MS2:.0f} m/s^2)."
            )
            rec = None
        ivs = intervals_above(times, vals, WARN_MS2)
        return make_finding(
            self.id,
            sev,
            f"Elevated {axis} vibration" + (f" (IMU{imu})" if multi else ""),
            msg,
            recommendation=rec,
            time_start_s=ivs[0].start_s if ivs else None,
            time_end_s=ivs[-1].end_s if ivs else None,
            message_types=["VIBE"],
            samples={
                f"max_{axis}": round(vmax, 1),
                f"frac_above_{int(WARN_MS2)}": round(frac_warn, 3),
                "imu": imu,
            },
        )

    @staticmethod
    def _clip_streams(log: FlightLog) -> list[tuple[int, np.ndarray, np.ndarray]]:
        """(imu, times, cumulative clip counter) for every IMU, in either log layout."""
        streams: list[tuple[int, np.ndarray, np.ndarray]] = []
        for imu in log.instances("VIBE"):
            times, vals = log.series("VIBE", "Clip", imu)
            if vals.size:
                streams.append((imu, times, vals))
        if streams:
            return streams
        for imu, clip in enumerate(("Clip0", "Clip1", "Clip2")):  # pre-4.0 layout
            times, vals = log.series("VIBE", clip)
            if vals.size:
                streams.append((imu, times, vals))
        return streams

    def _clip_findings(self, log: FlightLog) -> list[Finding]:
        out: list[Finding] = []
        streams = self._clip_streams(log)
        primary = primary_imu(log, [imu for imu, _, _ in streams])
        for imu, times, vals in streams:
            # Counters are cumulative. Sum only the increments so a counter reset
            # (reboot mid-log) is not read as a negative or a huge jump.
            steps = np.diff(vals)
            total = float(np.sum(steps[steps > 0])) if steps.size else 0.0
            if total <= 0:
                continue
            first = int(np.flatnonzero(steps > 0)[0]) + 1
            onset = float(times[first]) if first < times.size else None
            sev = Severity.CRITICAL if total >= CLIP_CRIT_COUNT else Severity.WARN
            out.append(
                make_finding(
                    self.id,
                    sev,
                    f"Accelerometer clipping (IMU{imu})",
                    f"IMU{imu} recorded {int(total)} accelerometer clipping event(s). Clipping means the "
                    "IMU saturated (railed) — its data is unusable while clipped, which is a serious "
                    "vibration problem.",
                    recommendation="Reduce vibration at the source (balance props, soft-mount the FC) before the next flight.",
                    time_start_s=onset,
                    time_end_s=float(times[-1]) if times.size else None,
                    message_types=["VIBE"],
                    samples={"total_clips": int(total), "imu": imu, "crit_threshold": CLIP_CRIT_COUNT},
                )
            )
            if imu != primary and len(streams) > 1:
                demote(out[-1])
        return out


def primary_imu(log: FlightLog, imus: list[int]) -> int:
    """The IMU whose readings decide severity.

    Design choice: ArduPilot does not log which IMU is primary, so we take the first
    IMU that is in use (INS_USE / INS_USE2 / INS_USE3 != 0, default in use), i.e.
    normally IMU0. Secondary IMUs are still reported, one severity lower: some boards
    deliberately leave one IMU unisolated (the Cube's third IMU), so its higher
    vibration is expected and must not drive a CRITICAL on its own.
    """
    for imu in sorted(imus):
        use = log.params.get("INS_USE" if imu == 0 else f"INS_USE{imu + 1}")
        if use is None or use != 0:
            return imu
    return min(imus) if imus else 0


def demote(f: Finding) -> bool:
    """Lower a secondary IMU's finding one severity level; False if it drops below INFO."""
    if f.severity == Severity.INFO:
        return False
    f.severity = Severity.WARN if f.severity == Severity.CRITICAL else Severity.INFO
    f.explanation += " (Secondary IMU: reported one level lower; the primary IMU decides severity.)"
    return True
