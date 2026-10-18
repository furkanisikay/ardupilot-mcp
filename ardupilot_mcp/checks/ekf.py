# SPDX-License-Identifier: AGPL-3.0-only
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

Cores: EKF3 runs one core ("lane") per IMU and logs them all under XKF4, told
apart by the ``C`` instance field; ``XKF4.PI`` is the primary core index — the one
actually flying the vehicle (Copter-4.6.3 libraries/AP_NavEKF3/LogStructure.h).
Ratios are judged per core, at full severity only while that core was primary. A
standby core straining is reported as INFO, and the primary core changing
mid-log (a lane switch) is reported as INFO with its time.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import intervals_above, make_finding

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

        cores = log.instances(mtype)
        findings: list[Finding] = []
        for core in cores:
            # High-rate table: stay in numpy (record-aligned, NaN where a field is missing).
            times, cols = log.aligned(mtype, ["PI", *(f for f, _ in SENSOR_FIELDS)], core)
            pi = cols["PI"]
            # Samples where this core was flying the vehicle. Without PI (EKF2, old
            # logs, single core) every sample counts, as before.
            primary = (
                (pi == core) if (len(cores) > 1 and np.isfinite(pi).any()) else np.ones(times.size, bool)
            )
            for field, sensor in SENSOR_FIELDS:
                vals = cols[field]
                f = self._field_finding(mtype, field, sensor, times, vals, primary)
                if f is None:
                    continue
                if len(cores) > 1:
                    f.title += f" (core {core})"
                    f.evidence.samples["ekf_core"] = core
                findings.append(f)
        findings.extend(self._lane_switch(log, mtype))
        return findings

    def _field_finding(
        self,
        mtype: str,
        field: str,
        sensor: str,
        times: np.ndarray,
        vals: np.ndarray,
        primary: np.ndarray,
    ) -> Finding | None:
        ok = np.isfinite(vals) & np.isfinite(times)
        p_ok = ok & primary
        p_max = float(np.max(vals[p_ok])) if np.any(p_ok) else None
        all_max = float(np.max(vals[ok])) if np.any(ok) else None
        if all_max is None or all_max <= WARN_RATIO:
            return None
        standby_only = p_max is None or p_max <= WARN_RATIO
        sel = ok if standby_only else p_ok
        vmax: float = all_max if (standby_only or p_max is None) else p_max
        t_sel, v_sel = times[sel], vals[sel]

        if standby_only:
            sev = Severity.INFO
            threshold = WARN_RATIO
            msg = (
                f"A standby EKF core's {sensor} test ratio peaked at {vmax:.2f} in {mtype}.{field}, but only "
                "while it was NOT the primary core flying the vehicle, so it did not affect navigation."
            )
            rec = None
        elif vmax > CRIT_RATIO:
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
        ivs = intervals_above(t_sel, v_sel, WARN_RATIO)
        return make_finding(
            self.id,
            sev,
            f"EKF {sensor} variance ({field})",
            msg,
            recommendation=rec,
            time_start_s=ivs[0].start_s if ivs else None,
            time_end_s=ivs[-1].end_s if ivs else None,
            message_types=[mtype],
            samples={f"max_{field}": round(vmax, 3), "threshold": threshold},
        )

    def _lane_switch(self, log: FlightLog, mtype: str) -> list[Finding]:
        times, pi = log.series(mtype, "PI")
        if pi.size < 2:
            return []
        changes = np.flatnonzero(np.diff(pi) != 0) + 1
        if changes.size == 0:
            return []
        t = float(times[changes[0]])
        return [
            make_finding(
                self.id,
                Severity.INFO,
                "EKF primary core changed (lane switch)",
                f"The primary EKF core changed {changes.size} time(s); first at t={t:.1f}s "
                f"(core {int(pi[changes[0] - 1])} -> {int(pi[changes[0]])}). EKF3 switches lanes when the "
                "active core's sensors look worse than another's — look at what that IMU/sensor was doing then.",
                time_start_s=t,
                time_end_s=float(times[changes[-1]]),
                message_types=[mtype],
                samples={"lane_switches": int(changes.size)},
            )
        ]
