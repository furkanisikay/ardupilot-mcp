"""GPS quality check.

Reads the ``GPS`` message and grades three independent concerns that each
degrade position-hold / RTL reliability:

* **Fix quality** — ``Status`` is the fix type (0/1 = no fix, 2 = 2D, 3 = 3D,
  4 = DGPS, 5 = RTK float, 6 = RTK fixed). A 3D fix (Status >= 3) is the
  minimum for GPS-aided flight. Losing it *after* acquiring it mid-flight is a
  hard failure; never acquiring one means the log flew (or sat) without usable
  GPS.
* **Satellite count** — ``NSats``. ArduPilot wants roughly 6+ sats for a stable
  solution; dipping below ~4 means the receiver cannot even maintain a 3D fix.
* **HDOP** — ``HDop`` (horizontal dilution of precision). Under ~1.5 is good,
  and ArduPilot's default ``GPS_HDOP_GOOD`` arming gate is 1.4; rising HDOP
  means the satellite geometry is poor and the horizontal position is loose.

Emits at most one finding per concern (fix, sats, hdop), each citing the
concrete min/max value, threshold and time window.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import intervals_above, make_finding, safe_max, safe_min

# Fix-type thresholds (ArduPilot GPS Status enum).
FIX_3D = 3  # Status >= 3 is a usable 3D fix.

# Satellite-count thresholds.
SATS_WARN = 6  # below this the solution is marginal.
SATS_CRIT = 4  # below this a 3D fix cannot be held.

# HDOP thresholds (horizontal dilution of precision).
HDOP_WARN = 2.0  # above ~2 the geometry is getting poor (good is < ~1.5).
HDOP_CRIT = 5.0  # above 5 the horizontal position is effectively unusable.
# Without a fix the receiver reports a sentinel HDOP (~99.99); that is "no signal",
# not real dilution, and is already covered by the fix/sats findings.
HDOP_SENTINEL = 50.0


@register_check
class GpsCheck(Check):
    id = "gps"
    title = "GPS quality"
    category = "gps"
    requires = {"GPS"}
    description = "GPS fix quality, satellite count and HDOP from the GPS message."

    def run(self, log: FlightLog) -> list[Finding]:
        # Evaluate GPS over the armed-flight window so the pre-arm acquisition
        # phase (sats climbing from 0, no fix yet) and post-land idling don't
        # look like in-flight GPS failures.
        window = log.armed_window()
        findings: list[Finding] = []
        for sub in (self._fix_finding, self._sats_finding, self._hdop_finding):
            f = sub(log, window)
            if f is not None:
                findings.append(f)
        return findings

    def _fix_finding(self, log: FlightLog, window) -> Finding | None:
        times, status = log.clip_to_window(*log.series("GPS", "Status"), window)
        if status.size == 0:
            return None

        # Index of the first 3D fix, if any.
        achieved = np.flatnonzero(status >= FIX_3D)
        if achieved.size == 0:
            # Never reached a 3D fix at all.
            worst = safe_max(status) or 0.0
            return make_finding(
                self.id,
                Severity.WARN,
                "GPS never achieved a 3D fix",
                f"GPS Status never reached {FIX_3D} (3D fix) across {status.size} samples; "
                f"the best fix type seen was {int(worst)}. Without a 3D fix the GPS cannot "
                "supply usable position to the EKF.",
                recommendation="Check antenna placement and interference; allow more time/clear sky to acquire a fix before flying.",
                time_start_s=float(times[0]) if times.size else None,
                time_end_s=float(times[-1]) if times.size else None,
                message_types=["GPS"],
                samples={
                    "max_Status": int(worst),
                    "min_Status": int(safe_min(status) or 0.0),
                    "fix_3d_threshold": FIX_3D,
                },
            )

        # A 3D fix was achieved; did Status ever drop below 3 afterwards?
        first_fix_i = int(achieved[0])
        first_fix_t = float(times[first_fix_i]) if first_fix_i < times.size else None
        post_times = times[first_fix_i:]
        post_status = status[first_fix_i:]
        # intervals_above on the *negated* status finds windows where Status < FIX_3D.
        loss_intervals = intervals_above(post_times, -post_status, float(-FIX_3D))
        if not loss_intervals:
            return None

        min_after = safe_min(post_status) or 0.0
        t0 = loss_intervals[0].start_s
        t1 = loss_intervals[-1].end_s
        return make_finding(
            self.id,
            Severity.CRITICAL,
            "GPS lost 3D fix during the flight",
            f"GPS first reached a 3D fix at t={first_fix_t:.1f}s but then dropped below 3D "
            f"(Status fell to {int(min_after)}) in {len(loss_intervals)} window(s) between "
            f"t={t0:.1f}s and t={t1:.1f}s. A mid-flight fix loss removes GPS position aiding "
            "and can trigger EKF failsafes or position drift.",
            recommendation="Investigate GPS dropout cause (interference, antenna, power) before relying on GPS modes again.",
            time_start_s=t0,
            time_end_s=t1,
            message_types=["GPS"],
            samples={
                "first_fix_time_s": round(first_fix_t, 1) if first_fix_t is not None else 0.0,
                "min_Status_after_fix": int(min_after),
                "loss_windows": len(loss_intervals),
                "fix_3d_threshold": FIX_3D,
            },
        )

    def _sats_finding(self, log: FlightLog, window) -> Finding | None:
        times, nsats = log.clip_to_window(*log.series("GPS", "NSats"), window)
        if nsats.size == 0:
            return None
        nmin = safe_min(nsats)
        if nmin is None or nmin >= SATS_WARN:
            return None

        # Time window where NSats is at its (low) extreme — windows below SATS_WARN.
        low_intervals = intervals_above(times, -nsats, float(-SATS_WARN))
        t0 = low_intervals[0].start_s if low_intervals else None
        t1 = low_intervals[-1].end_s if low_intervals else None

        if nmin < SATS_CRIT:
            sev = Severity.CRITICAL
            msg = (
                f"GPS satellite count dropped to {int(nmin)} (below {SATS_CRIT}); with this few "
                f"satellites the receiver cannot hold a 3D fix. Min across {nsats.size} samples "
                f"was {int(nmin)}."
            )
            rec = "Treat GPS position as unreliable; check antenna/sky view and interference before GPS-dependent flight."
        else:
            sev = Severity.WARN
            msg = (
                f"GPS satellite count fell to a low of {int(nmin)} (below {SATS_WARN}); the "
                f"position solution is marginal. Min across {nsats.size} samples was {int(nmin)}."
            )
            rec = "Prefer 6+ satellites before arming in GPS modes; improve antenna placement/sky view."

        return make_finding(
            self.id,
            sev,
            f"Low GPS satellite count, min {int(nmin)}",
            msg,
            recommendation=rec,
            time_start_s=t0,
            time_end_s=t1,
            message_types=["GPS"],
            samples={
                "min_NSats": int(nmin),
                "warn_threshold": SATS_WARN,
                "crit_threshold": SATS_CRIT,
            },
        )

    def _hdop_finding(self, log: FlightLog, window) -> Finding | None:
        times, hdop = log.clip_to_window(*log.series("GPS", "HDop"), window)
        if hdop.size == 0:
            return None
        # HDOP only means something when there's a fix. Drop no-fix sentinel
        # readings (~99.99) so we don't mislabel "no signal" as "poor geometry"
        # (the fix/sats findings already cover the no-fix case).
        _, status = log.clip_to_window(*log.series("GPS", "Status"), window)
        if status.size == hdop.size:
            valid = (status >= FIX_3D) & (hdop < HDOP_SENTINEL)
        else:
            valid = hdop < HDOP_SENTINEL
        times, hdop = times[valid], hdop[valid]
        if hdop.size == 0:
            return None
        hmax = safe_max(hdop)
        if hmax is None or hmax <= HDOP_WARN:
            return None

        if hmax > HDOP_CRIT:
            sev = Severity.CRITICAL
            threshold = HDOP_CRIT
            msg = (
                f"GPS HDOP peaked at {hmax:.1f} (above {HDOP_CRIT:.1f}); horizontal position is "
                "effectively unusable at this dilution. Good HDOP is under ~1.5."
            )
            rec = "Do not trust horizontal position; resolve the cause (sky view, interference, multipath) before GPS flight."
        else:
            sev = Severity.WARN
            threshold = HDOP_WARN
            msg = (
                f"GPS HDOP peaked at {hmax:.1f} (above {HDOP_WARN:.1f}); satellite geometry is "
                "poor and the horizontal position is loose. Good HDOP is under ~1.5."
            )
            rec = "Aim for HDOP under ~1.5 before arming in GPS modes; improve antenna placement/sky view."

        ivs = intervals_above(times, hdop, threshold)
        t0 = ivs[0].start_s if ivs else None
        t1 = ivs[-1].end_s if ivs else None
        return make_finding(
            self.id,
            sev,
            f"High GPS HDOP, peak {hmax:.1f}",
            msg,
            recommendation=rec,
            time_start_s=t0,
            time_end_s=t1,
            message_types=["GPS"],
            samples={
                "max_HDop": round(hmax, 2),
                "warn_threshold": HDOP_WARN,
                "crit_threshold": HDOP_CRIT,
            },
        )
