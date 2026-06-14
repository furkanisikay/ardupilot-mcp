"""Motor outputs & balance check.

Reads the ``RCOU`` message: the PWM (microseconds) commanded to each ESC/motor
output channel (C1..C8), nominally ~1000 (idle/min) to ~2000 (full). Two failure
modes are diagnosed deterministically:

- **Imbalance** — if one motor consistently runs much hotter than the others,
  the autopilot is fighting an asymmetry (CG offset, twisted frame, weak/strong
  motor, prop pitch error). We compare the per-motor mean outputs of the active
  motors and flag a large spread.
- **Saturation** — if a motor spends a large fraction of the flight pinned near
  maximum PWM there is little headroom left for control; the craft is likely
  underpowered or overweight and can lose attitude authority.

Only channels that look like real, spinning motors (mean output above
``ACTIVE_MIN_PWM``) are considered; unused/disarmed-at-min channels are ignored.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import fraction_above, make_finding

# Channels RCOU may carry; we only inspect the ones present and active.
MOTOR_CHANNELS = ("C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8")

# A channel is treated as an active motor only when its mean PWM exceeds this.
# Idle/min for a spinning motor sits ~1000; a disarmed or unused output stays at
# ~1000 or below, so 1100 cleanly separates "really driving a motor" from noise.
ACTIVE_MIN_PWM = 1100.0

# Imbalance: spread between the hottest and coolest active motor mean (PWM us).
IMBALANCE_WARN_PWM = 150.0
IMBALANCE_CRIT_PWM = 300.0

# Saturation: a sample at/above this PWM is "near maximum" (typical max ~2000).
SATURATION_PWM = 1950.0
# Fraction of the flight a motor may spend saturated before it is a concern.
SATURATION_WARN_FRACTION = 0.20
SATURATION_CRIT_FRACTION = 0.50


@register_check
class MotorsCheck(Check):
    id = "motors"
    title = "Motor outputs & balance"
    category = "motors"
    requires = {"RCOU"}
    # Multirotor only: on a heli RCOU C1-C4 are swashplate servos, on a plane
    # they are control surfaces, on a rover throttle/steering — "motor balance"
    # is meaningless there and would be a false positive.
    vehicles = {"copter"}
    description = "Per-motor PWM output balance and saturation from RCOU (multirotor only)."

    def run(self, log: FlightLog) -> list[Finding]:
        # Collect aligned (times, values) for every channel that is logged and active.
        active: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for ch in MOTOR_CHANNELS:
            times, vals = log.series("RCOU", ch)
            if vals.size == 0:
                continue
            if float(np.mean(vals)) > ACTIVE_MIN_PWM:
                active[ch] = (times, vals)
        if not active:
            return []

        findings: list[Finding] = []
        means = {ch: float(np.mean(vals)) for ch, (_, vals) in active.items()}
        findings.extend(self._imbalance_findings(means))
        findings.extend(self._saturation_findings(active))
        return findings

    def _imbalance_findings(self, means: dict[str, float]) -> list[Finding]:
        # Need at least two active motors to talk about balance.
        if len(means) < 2:
            return []
        hot_ch = max(means, key=means.__getitem__)
        cold_ch = min(means, key=means.__getitem__)
        spread = means[hot_ch] - means[cold_ch]
        if spread <= IMBALANCE_WARN_PWM:
            return []

        others = [m for ch, m in means.items() if ch != hot_ch]
        others_mean = float(np.mean(others)) if others else means[cold_ch]
        above_others = means[hot_ch] - others_mean

        if spread > IMBALANCE_CRIT_PWM:
            sev = Severity.CRITICAL
        else:
            sev = Severity.WARN
        msg = (
            f"motor output imbalance: motor {hot_ch} averages {above_others:.0f} PWM higher "
            f"than the others (spread {spread:.0f} PWM between {hot_ch} and {cold_ch}, "
            f"threshold {IMBALANCE_WARN_PWM:.0f}) — possible CG/frame/mechanical issue."
        )
        samples: dict[str, float] = {f"mean_{ch}": round(m, 1) for ch, m in sorted(means.items())}
        samples["imbalance_pwm"] = round(spread, 1)
        return [
            make_finding(
                self.id,
                sev,
                "Motor output imbalance",
                msg,
                recommendation=(
                    "Check CG/level, motor mounts, prop pitch and ESC calibration; a single hot "
                    f"motor ({hot_ch}) means the autopilot is continuously compensating for an asymmetry."
                ),
                message_types=["RCOU"],
                samples=samples,
            )
        ]

    def _saturation_findings(self, active: dict[str, tuple[np.ndarray, np.ndarray]]) -> list[Finding]:
        out: list[Finding] = []
        for ch in sorted(active):
            times, vals = active[ch]
            frac = fraction_above(vals, SATURATION_PWM)
            if frac <= SATURATION_WARN_FRACTION:
                continue
            sev = Severity.CRITICAL if frac > SATURATION_CRIT_FRACTION else Severity.WARN
            msg = (
                f"motor {ch} spent {frac * 100:.0f}% of the flight near maximum "
                f"(>{SATURATION_PWM:.0f} PWM, threshold {SATURATION_WARN_FRACTION * 100:.0f}%) "
                "— the craft may be underpowered/overweight."
            )
            out.append(
                make_finding(
                    self.id,
                    sev,
                    f"Motor saturation ({ch})",
                    msg,
                    recommendation=(
                        "Reduce takeoff weight or fit higher-thrust motors/props; a motor pinned near "
                        "maximum leaves no headroom for attitude control and risks loss of control."
                    ),
                    time_start_s=float(times[0]) if times.size else None,
                    time_end_s=float(times[-1]) if times.size else None,
                    message_types=["RCOU"],
                    samples={
                        f"mean_{ch}": round(float(np.mean(vals)), 1),
                        f"frac_above_{int(SATURATION_PWM)}_{ch}": round(frac, 3),
                    },
                )
            )
        return out
