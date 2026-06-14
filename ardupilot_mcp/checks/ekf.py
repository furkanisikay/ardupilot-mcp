"""EKF / estimator health check.

ArduPilot's EKF logs normalised innovation/variance "test ratios" — one per
sensor — in the ``XKF4`` message (EKF3) or ``NKF4`` (the older EKF2). A ratio
near 0 is healthy; the filter consults a sensor more cautiously as the ratio
climbs and, above 1.0, actively *rejects* that sensor's data (the innovation is
too large relative to the modelled uncertainty). Sustained high ratios mean the
estimator distrusted a sensor — a precursor to position/altitude drift, toppling
or an EKF failsafe.

Fields (the dimensionless squared-innovation test ratios from XKF4/NKF4):
  SV  velocity     SP  position     SH  height     SM  magnetometer
(SVT is intentionally not used — it is not a normalised test ratio; see SENSOR_FIELDS.)

For each present field we take the peak ratio over the log:
  > 1.0 -> CRITICAL (sensor was being rejected)
  > 0.8 -> WARN     (sensor variance high, approaching rejection)
We prefer XKF4 and fall back to NKF4; if neither is present the check is a no-op.
"""

from __future__ import annotations

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import intervals_above, make_finding, safe_max

# Variance test ratios are normalised: 1.0 is the rejection boundary, so a peak
# above it means the EKF actually distrusted/rejected that sensor's data.
CRIT_RATIO = 1.0
# Below the rejection line but high enough to warn — the filter is straining.
WARN_RATIO = 0.8

# EKF3 message preferred; EKF2 (NKF4) is the legacy fallback. Same field layout.
PREFERRED_MSG = "XKF4"
FALLBACK_MSG = "NKF4"

# Variance field -> readable sensor name, in a stable order.
# Only SV/SP/SH/SM are the normalised "squared innovation test ratios" (1.0 =
# rejection boundary) per the XKF4 log definition. SVT is deliberately EXCLUDED:
# in XKF4 (EKF3) it is the airspeed *variance* (not a normalised test ratio), and
# in NKF4 (EKF2) the same column is a tilt-error convergence metric entirely — so
# applying the >1.0 rejection threshold to it would be wrong.
# https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_NavEKF3/LogStructure.h
SENSOR_FIELDS: tuple[tuple[str, str], ...] = (
    ("SV", "velocity"),
    ("SP", "position"),
    ("SH", "height"),
    ("SM", "magnetometer"),
)


@register_check
class EKFCheck(Check):
    id = "ekf"
    title = "EKF / estimator health"
    category = "estimator"
    # Multi-source: runs always and guards each message type inside run().
    requires: set[str] = set()
    description = "EKF innovation/variance test ratios (XKF4/NKF4) per sensor."

    def run(self, log: FlightLog) -> list[Finding]:
        # Prefer EKF3 (XKF4); fall back to EKF2 (NKF4); otherwise nothing to do.
        if log.has(PREFERRED_MSG):
            mtype = PREFERRED_MSG
        elif log.has(FALLBACK_MSG):
            mtype = FALLBACK_MSG
        else:
            return []

        findings: list[Finding] = []
        for field, sensor in SENSOR_FIELDS:
            times, vals = log.series(mtype, field)
            if vals.size == 0:
                continue
            vmax = safe_max(vals)
            if vmax is None or vmax <= WARN_RATIO:
                continue

            if vmax > CRIT_RATIO:
                sev = Severity.CRITICAL
                threshold = CRIT_RATIO
                msg = (
                    f"EKF {sensor} variance peaked at {vmax:.2f} (>{CRIT_RATIO:.1f}) in {mtype}.{field} "
                    f"— the estimator was rejecting {sensor} data. A test ratio above {CRIT_RATIO:.1f} "
                    f"means the {sensor} innovation exceeded the filter's modelled uncertainty, so that "
                    "sensor was distrusted; sustained rejection leads to position/altitude drift or an EKF failsafe."
                )
                rec = (
                    f"Investigate the {sensor} sensor and its noise/health for this flight; "
                    "review the rest of the diagnosis (vibration, GPS, compass) for the root cause."
                )
            else:
                sev = Severity.WARN
                threshold = WARN_RATIO
                msg = (
                    f"EKF {sensor} variance high (>{WARN_RATIO:.1f}): peaked at {vmax:.2f} in {mtype}.{field}. "
                    f"The test ratio approached the {CRIT_RATIO:.1f} rejection boundary, so the estimator was "
                    f"straining to trust {sensor} data even though it did not fully reject it."
                )
                rec = f"Keep an eye on {sensor} estimator health; check the relevant sensor if this recurs."

            # Window when the ratio was above the (warn) line for timing evidence.
            ivs = intervals_above(times, vals, WARN_RATIO)
            t0 = ivs[0].start_s if ivs else None
            t1 = ivs[-1].end_s if ivs else None

            findings.append(
                make_finding(
                    self.id,
                    sev,
                    f"EKF {sensor} variance ({field})",
                    msg,
                    recommendation=rec,
                    time_start_s=t0,
                    time_end_s=t1,
                    message_types=[mtype],
                    samples={
                        f"max_{field}": round(vmax, 3),
                        "threshold": threshold,
                    },
                )
            )
        return findings
