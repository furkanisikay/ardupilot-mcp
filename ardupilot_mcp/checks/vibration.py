"""Vibration check.

Reads the ``VIBE`` message: per-axis vibration (VibeX/Y/Z, m/s^2) and the
cumulative accelerometer-clip counters (Clip0/1/2). Per ArduPilot's vibration
guidance, sustained vibration below ~30 m/s^2 is generally acceptable, 30-60 may
cause problems, and >60 nearly always degrades position/altitude hold. Ideal
clipping is zero; a small count (<~100) is likely ok (especially around hard
landings), but a steadily increasing count means the IMU is railing — a serious
vibration problem. Accordingly the clip check warns on any clipping and escalates
to critical once the count is large (>=100).

Source: https://ardupilot.org/copter/docs/common-measuring-vibration.html
"""

from __future__ import annotations

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import fraction_above, intervals_above, make_finding, safe_max

WARN_MS2 = 30.0
CRIT_MS2 = 60.0
# Fraction of the flight above WARN that turns a marginal note into a warning.
SUSTAINED_FRACTION = 0.10


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
        for axis in ("VibeX", "VibeY", "VibeZ"):
            times, vals = log.series("VIBE", axis)
            if vals.size == 0:
                continue
            vmax = safe_max(vals) or 0.0
            frac_warn = fraction_above(vals, WARN_MS2)
            frac_crit = fraction_above(vals, CRIT_MS2)
            if vmax <= WARN_MS2:
                continue
            if frac_crit > 0.0 or vmax > CRIT_MS2:
                sev = Severity.CRITICAL
                msg = (
                    f"{axis} vibration peaks at {vmax:.0f} m/s^2 (>{CRIT_MS2:.0f}); "
                    f"{frac_crit * 100:.0f}% of samples exceed {CRIT_MS2:.0f} m/s^2. "
                    "Vibration this high corrupts the accelerometers and can trigger EKF failures."
                )
                rec = "Improve flight-controller isolation/balance; check prop/motor condition before flying again."
            elif frac_warn >= SUSTAINED_FRACTION:
                sev = Severity.WARN
                msg = (
                    f"{axis} vibration is elevated: peak {vmax:.0f} m/s^2, "
                    f"{frac_warn * 100:.0f}% of the flight above {WARN_MS2:.0f} m/s^2."
                )
                rec = "Inspect mounting/balance; consider the harmonic notch filter (see recommend_tuning)."
            else:
                sev = Severity.INFO
                msg = (
                    f"{axis} vibration is mostly fine but peaks at {vmax:.0f} m/s^2 "
                    f"(brief excursions above {WARN_MS2:.0f} m/s^2)."
                )
                rec = None

            ivs = intervals_above(times, vals, WARN_MS2)
            t0 = ivs[0].start_s if ivs else None
            t1 = ivs[-1].end_s if ivs else None
            out.append(
                make_finding(
                    self.id,
                    sev,
                    f"Elevated {axis} vibration",
                    msg,
                    recommendation=rec,
                    time_start_s=t0,
                    time_end_s=t1,
                    message_types=["VIBE"],
                    samples={
                        f"max_{axis}": round(vmax, 1),
                        f"frac_above_{int(WARN_MS2)}": round(frac_warn, 3),
                    },
                )
            )
        return out

    def _clip_findings(self, log: FlightLog) -> list[Finding]:
        out: list[Finding] = []
        for clip in ("Clip0", "Clip1", "Clip2"):
            times, vals = log.series("VIBE", clip)
            if vals.size == 0:
                continue
            # Counters are cumulative; total clips = last - first.
            total = float(vals[-1] - vals[0]) if vals.size else 0.0
            if total <= 0:
                continue
            # First time the counter increments.
            onset = None
            for i in range(1, vals.size):
                if vals[i] > vals[i - 1]:
                    onset = float(times[i]) if i < times.size else None
                    break
            sev = Severity.CRITICAL if total >= 100 else Severity.WARN
            out.append(
                make_finding(
                    self.id,
                    sev,
                    f"Accelerometer clipping ({clip})",
                    f"{clip} recorded {int(total)} accelerometer clipping event(s). Clipping means an "
                    "IMU saturated (railed) — its data is unusable while clipped, which is a serious "
                    "vibration problem.",
                    recommendation="Reduce vibration at the source (balance props, soft-mount the FC) before the next flight.",
                    time_start_s=onset,
                    message_types=["VIBE"],
                    samples={f"total_{clip}": int(total)},
                )
            )
        return out
