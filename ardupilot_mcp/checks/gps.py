# SPDX-License-Identifier: AGPL-3.0-only
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

Judged in flight (``FlightLog.flying_mask``): armed, and airborne where the log says so, so a
plane with ARMING_REQUIRE=0, armed from boot, is not judged while its GPS still acquires on
the ground. A fix loss, satellite drop or HDOP excursion
is critical when it lasts 1 s; a shorter one is a dropout (warning). Emits at most
one finding per concern (fix, sats, hdop) per receiver, each
citing the concrete min/max value, threshold and time window.

Multiple receivers: modern firmware logs every GPS under the one ``GPS`` message,
told apart by the ``I`` instance field (0 = GPS1, 1 = GPS2, 2 = the blended
solution when GPS_AUTO_SWITCH=2; Copter-4.6.3 libraries/AP_GPS). Each receiver is
judged on its own samples. A problem on one receiver while another physical
receiver held a 3D fix for the same window is downgraded from CRITICAL to WARN:
the autopilot can keep navigating on the healthy unit, so it is a hardware fault
to fix, not a loss of navigation. This only applies when GPS_AUTO_SWITCH != 0
(0 = always use the primary receiver).
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import intervals_above, longest_run_above, make_finding, safe_max, safe_min

# Fix-type thresholds (ArduPilot GPS Status enum).
FIX_3D = 3  # Status >= 3 is a usable 3D fix.
# GPS instance id used for the blended solution (GPS_AUTO_SWITCH=2): it is derived
# from the physical receivers, so only receivers 0/1 are judged when present.
# Source: Copter-4.6.3 libraries/AP_GPS/AP_GPS_config.h (GPS_BLENDED_INSTANCE = GPS_MAX_RECEIVERS = 2).
BLENDED_INSTANCE = 2

# Satellite-count thresholds.
SATS_WARN = 6  # below this the solution is marginal.
SATS_CRIT = 4  # below this a 3D fix cannot be held.

# HDOP thresholds (horizontal dilution of precision).
HDOP_WARN = 2.0  # above ~2 the geometry is getting poor (good is < ~1.5).
HDOP_CRIT = 5.0  # above 5 the horizontal position is effectively unusable.
# Without a fix the receiver reports a sentinel HDOP (~99.99); that is "no signal",
# not real dilution, and is already covered by the fix/sats findings.
HDOP_SENTINEL = 50.0
# Design choice: a fix loss, satellite drop or HDOP excursion is critical when it lasts this
# long. A shorter one is a dropout of a sample or two (GPS logs at 5-10 Hz), which the EKF
# rides through on its inertial prediction. On the 48 benchmark crash logs every such run
# in flight lasted either 0.2 s (one sample) or 4.4 s and more.
LOSS_MIN_DURATION_S = 1.0


# AP_GPS update-rate health (libraries/AP_GPS/AP_GPS.cpp, Copter-4.6.3): a frame more than
# 245 ms after the last ("200 ms (5Hz) + 45 ms buffer") is delayed; the receiver is unhealthy
# with two delayed frames in a row, or when the running average delta (0.98 old + 0.02 new,
# frames under 2 s) reaches 215 ms (333 ms for an RTK moving-baseline rover, not judged).
GPS_MAX_DELTA_MS = 245.0
GPS_DELAY_THRESHOLD = 2
GPS_AVG_DELTA_MAX_MS = 215.0
# Design choice: report the rate when the firmware's rule holds the receiver unhealthy for at
# least this share of the armed time (benchmark: one log at 100%, a 1 Hz NMEA pair; the next
# highest any log reaches is 7%).
GPS_SLOW_SHARE = 0.10


# EKF3 starts using a GPS only once its reported horizontal accuracy is within 5 m, scaled by
# EK3_CHECK_SCALE (percent, default 100) ("GPS horiz error", AP_NavEKF3_VehicleStatus.cpp
# calcGpsGoodToAlign, Copter-4.6.3). GPA.HAcc 655.35 m is the "not reported" value.
EKF_HACC_LIMIT_M = 5.0
# Once a GPS has passed, the thresholds are raised by this factor for hysteresis (same
# function), so in flight the limit is 6.5 m at the default scale.
EKF_ALIGNED_HYSTERESIS = 1.3
HACC_NOT_REPORTED_M = 655.0
# Design choice: in flight, a receiver whose reported accuracy stays outside that limit this
# long is degraded.
HACC_MIN_DURATION_S = 10.0


def _timing_unhealthy(delta_ms: np.ndarray) -> np.ndarray:
    """Per GPA sample, whether AP_GPS::is_healthy's timing test fails after that frame."""
    bad = np.zeros(delta_ms.shape, dtype=bool)
    avg: float | None = None
    delayed = 0
    for i, d in enumerate(delta_ms):
        if not np.isfinite(d):
            continue
        delayed = delayed + 1 if d > GPS_MAX_DELTA_MS else 0
        if d < 2000:
            avg = float(d) if avg is None or avg <= 0 else 0.98 * avg + 0.02 * float(d)
        bad[i] = delayed >= GPS_DELAY_THRESHOLD or (avg is not None and avg >= GPS_AVG_DELTA_MAX_MS)
    return bad


def _in_flight(log: FlightLog, times: np.ndarray, *arrays: np.ndarray) -> tuple[np.ndarray, ...]:
    """``times`` and ``arrays`` restricted to flight (``FlightLog.flying_mask``).

    Each flight on its own: a log with several flights would otherwise count the time on
    the ground between them. Unknown arming keeps every sample.
    """
    mask = log.flying_mask(times)
    if mask is None:
        return (times, *arrays)
    return (times[mask], *(a[mask] for a in arrays))


@register_check
class GpsCheck(Check):
    id = "gps"
    title = "GPS quality"
    category = "gps"
    requires = {"GPS"}
    description = "GPS fix quality, satellite count and HDOP from the GPS message."

    def run(self, log: FlightLog) -> list[Finding]:
        # Evaluate GPS over the flight window so the pre-arm acquisition
        # phase (sats climbing from 0, no fix yet) and post-land idling don't
        # look like in-flight GPS failures.
        if log.never_armed:
            return []  # fix and satellite quality are judged in flight; this log never armed
        insts = log.instances("GPS")
        receivers = [i for i in insts if i < BLENDED_INSTANCE] or insts
        multi = len(receivers) > 1
        findings: list[Finding] = []
        for inst in receivers:
            label = f"GPS{inst + 1}" if multi else "GPS"
            for sub in (self._fix_finding, self._sats_finding, self._hdop_finding):
                f = sub(log, inst, label)
                if f is None:
                    continue
                if multi:
                    f.evidence.samples["gps_instance"] = inst
                    self._downgrade_if_covered(log, f, inst, receivers)
                findings.append(f)
            for extra in (self._rate_finding(log, inst, label), self._accuracy_finding(log, inst, label)):
                if extra is not None:
                    findings.append(extra)
        return findings

    def _rate_finding(self, log: FlightLog, inst: int, label: str) -> Finding | None:
        """The receiver updating too slowly for AP_GPS's health test (GPA.Delta, 4.x)."""
        if not log.has("GPA") or inst not in (log.instances("GPA") or [0]):
            return None
        t, delta = log.series("GPA", "Delta", inst if log.instances("GPA") else None)
        if t.size < 20:
            return None
        bad = _timing_unhealthy(delta)
        armed = log.armed_mask(t)
        if armed is not None:
            t, delta, bad = t[armed], delta[armed], bad[armed]
        if t.size < 20:
            return None
        share = float(np.mean(bad))
        if share < GPS_SLOW_SHARE:
            return None
        med = float(np.nanmedian(delta))
        return make_finding(
            self.id,
            Severity.WARN,
            f"{label} updates too slowly for the autopilot",
            f"{label} delivered a position every {med:.0f} ms ({1000.0 / med:.1f} Hz, median GPA.Delta) while armed. "
            f"ArduPilot treats a receiver as unhealthy when frames arrive more than {GPS_MAX_DELTA_MS:.0f} ms apart "
            f"twice in a row or average {GPS_AVG_DELTA_MAX_MS:.0f} ms or more (it expects 5 Hz); by that rule it was "
            f"unhealthy for {share * 100:.0f}% of the armed time.",
            recommendation=(
                "Set the receiver to 5 Hz or faster. ArduPilot configures u-blox receivers itself (GPS_RATE_MS, "
                "GPS_AUTO_CONFIG) but not NMEA ones, which must be set up with the maker's tool; enabling fewer "
                "constellations (GPS_GNSS_MODE) can also restore the rate."
            ),
            time_start_s=float(t[0]),
            time_end_s=float(t[-1]),
            message_types=["GPA"],
            samples={"median_delta_ms": round(med), "unhealthy_share": round(share, 2)},
        )

    def _accuracy_finding(self, log: FlightLog, inst: int, label: str) -> Finding | None:
        """The receiver's own horizontal accuracy (GPA.HAcc) outside EKF3's start limit in flight."""
        if not log.has("GPA") or inst not in (log.instances("GPA") or [0]):
            return None
        t, hacc = log.series("GPA", "HAcc", inst if log.instances("GPA") else None)
        if t.size < 20:
            return None
        t, hacc = _in_flight(log, t, hacc)
        reported = hacc < HACC_NOT_REPORTED_M
        if not reported.any():
            return None
        scale = float(log.params.get("EK3_CHECK_SCALE", 100.0) or 100.0) / 100.0
        limit = EKF_HACC_LIMIT_M * EKF_ALIGNED_HYSTERESIS * scale
        run = longest_run_above(t, np.where(reported, hacc, 0.0), limit)
        if run < HACC_MIN_DURATION_S:
            return None
        bad = reported & (hacc > limit)
        worst = float(np.nanmax(hacc[reported]))
        return make_finding(
            self.id,
            Severity.WARN,
            f"{label} reported poor accuracy in flight",
            f"In flight {label} rated its own horizontal accuracy (GPA.HAcc) worse than {limit:.1f} m for up to "
            f"{run:.0f} s at a time, peaking at {worst:.0f} m. EKF3 starts using a GPS only within 5 m and holds "
            "it to 1.3 times that once in use (EK3_CHECK_SCALE scales both); a position this loose pulls the "
            "EKF's position and, when GPS is the height source, its height.",
            recommendation=(
                "Look for interference near the GPS (power wiring, ESCs, video transmitters, a USB3 or SD-card "
                "source), the antenna's view of the sky and its ground plane; compare GPA.HAcc with NSats."
            ),
            time_start_s=float(t[bad][0]) if bad.any() else None,
            time_end_s=float(t[bad][-1]) if bad.any() else None,
            message_types=["GPA"],
            samples={"longest_s": round(run, 1), "peak_hacc_m": round(worst, 1), "limit_m": round(limit, 2)},
        )

    @staticmethod
    def _downgrade_if_covered(log: FlightLog, f: Finding, inst: int, receivers: list[int]) -> None:
        """CRITICAL -> WARN when another receiver held a 3D fix across the whole window."""
        if f.severity != Severity.CRITICAL:
            return
        # Only true when the autopilot is allowed to switch receivers
        # (GPS_AUTO_SWITCH: 0 = use primary only; default 1 = use best; 2 = blend;
        # 4 = primary while it has 3D fix, else best. Copter-4.6.3 libraries/AP_GPS/AP_GPS.cpp).
        if log.params.get("GPS_AUTO_SWITCH", 1.0) == 0:
            return
        t0, t1 = f.evidence.time_start_s, f.evidence.time_end_s
        for other in receivers:
            if other == inst:
                continue
            times, status = log.series("GPS", "Status", other)
            if t0 is not None and t1 is not None:
                mask = (times >= t0) & (times <= t1)
                times, status = times[mask], status[mask]
            if status.size and bool(np.all(status >= FIX_3D)):
                f.severity = Severity.WARN
                f.explanation += (
                    f" GPS{other + 1} kept a 3D fix throughout, so the autopilot could keep navigating "
                    "on it; this is a fault of one receiver (wiring, antenna, interference), not a loss "
                    "of GPS navigation."
                )
                return

    def _fix_finding(self, log: FlightLog, inst: int = 0, label: str = "GPS") -> Finding | None:
        times, status = _in_flight(log, *log.series("GPS", "Status", inst))
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
                f"{label} never achieved a 3D fix",
                f"{label} Status never reached {FIX_3D} (3D fix) across {status.size} samples; "
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
        longest = longest_run_above(post_times, -post_status, float(-FIX_3D))
        lasting = longest >= LOSS_MIN_DURATION_S
        return make_finding(
            self.id,
            Severity.CRITICAL if lasting else Severity.WARN,
            f"{label} lost 3D fix during the flight" if lasting else f"{label} 3D fix dropped out briefly",
            f"{label} first reached a 3D fix at t={first_fix_t:.1f}s but then dropped below 3D "
            f"(Status fell to {int(min_after)}) in {len(loss_intervals)} window(s) between "
            f"t={t0:.1f}s and t={t1:.1f}s, the longest {longest:.1f} s. "
            + (
                "A mid-flight fix loss removes GPS position aiding and can trigger EKF failsafes or position drift."
                if lasting
                else f"No dropout lasted {LOSS_MIN_DURATION_S:.0f} s; the EKF rides through a missing sample or "
                "two on its inertial prediction."
            ),
            recommendation="Investigate GPS dropout cause (interference, antenna, power) before relying on GPS modes again.",
            time_start_s=t0,
            time_end_s=t1,
            message_types=["GPS"],
            samples={
                "first_fix_time_s": round(first_fix_t, 1) if first_fix_t is not None else 0.0,
                "min_Status_after_fix": int(min_after),
                "loss_windows": len(loss_intervals),
                "longest_loss_s": round(longest, 1),
                "fix_3d_threshold": FIX_3D,
            },
        )

    def _sats_finding(self, log: FlightLog, inst: int = 0, label: str = "GPS") -> Finding | None:
        times, (nsats, status) = log.series_many("GPS", ["NSats", "Status"], inst)
        times, nsats, status = _in_flight(log, times, nsats, status)
        if nsats.size == 0:
            return None
        if np.isfinite(status).any() and not np.any(status >= FIX_3D):
            # Never a 3D fix in flight (indoors, no sky view, no GPS in use): the fix finding
            # already says so, and a satellite count of 0 adds nothing to it.
            return None
        nmin = safe_min(nsats)
        if nmin is None or nmin >= SATS_WARN:
            return None

        # Time window where NSats is at its (low) extreme — windows below SATS_WARN.
        low_intervals = intervals_above(times, -nsats, float(-SATS_WARN))
        t0 = low_intervals[0].start_s if low_intervals else None
        t1 = low_intervals[-1].end_s if low_intervals else None

        longest_crit = longest_run_above(times, -nsats, float(-SATS_CRIT))
        if nmin < SATS_CRIT and longest_crit >= LOSS_MIN_DURATION_S:
            sev = Severity.CRITICAL
            msg = (
                f"{label} satellite count dropped to {int(nmin)} (below {SATS_CRIT}); with this few "
                f"satellites the receiver cannot hold a 3D fix. Min across {nsats.size} samples "
                f"was {int(nmin)}."
            )
            rec = "Treat GPS position as unreliable; check antenna/sky view and interference before GPS-dependent flight."
        elif nmin < SATS_CRIT:
            sev = Severity.WARN
            msg = (
                f"{label} satellite count dipped to {int(nmin)} (below {SATS_CRIT}) for at most "
                f"{longest_crit:.1f} s, a dropout rather than a loss (none lasted {LOSS_MIN_DURATION_S:.0f} s)."
            )
            rec = "Check the GPS antenna, cable and interference sources if dropouts recur."
        else:
            sev = Severity.WARN
            msg = (
                f"{label} satellite count fell to a low of {int(nmin)} (below {SATS_WARN}); the "
                f"position solution is marginal. Min across {nsats.size} samples was {int(nmin)}."
            )
            rec = "Prefer 6+ satellites before arming in GPS modes; improve antenna placement/sky view."

        return make_finding(
            self.id,
            sev,
            f"Low {label} satellite count, min {int(nmin)}",
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

    def _hdop_finding(self, log: FlightLog, inst: int = 0, label: str = "GPS") -> Finding | None:
        # HDOP only means something when there's a fix. Drop no-fix sentinel
        # readings (~99.99) so we don't mislabel "no signal" as "poor geometry"
        # (the fix/sats findings already cover the no-fix case). Status comes from the
        # same records; without it, only the sentinel is dropped.
        times, (hdop, status) = log.series_many("GPS", ["HDop", "Status"], inst)
        if hdop.size:
            times, hdop, status = _in_flight(log, times, hdop, status)
            valid = (status >= FIX_3D) & (hdop < HDOP_SENTINEL)
        else:
            times, hdop = _in_flight(log, *log.series("GPS", "HDop", inst))
            valid = hdop < HDOP_SENTINEL
        if hdop.size == 0:
            return None
        times, hdop = times[valid], hdop[valid]
        if hdop.size == 0:
            return None
        hmax = safe_max(hdop)
        if hmax is None or hmax <= HDOP_WARN:
            return None

        longest_crit = longest_run_above(times, hdop, HDOP_CRIT)
        if hmax > HDOP_CRIT and longest_crit >= LOSS_MIN_DURATION_S:
            sev = Severity.CRITICAL
            threshold = HDOP_CRIT
            msg = (
                f"{label} HDOP peaked at {hmax:.1f} (above {HDOP_CRIT:.1f}); horizontal position is "
                "effectively unusable at this dilution. Good HDOP is under ~1.5."
            )
            rec = "Do not trust horizontal position; resolve the cause (sky view, interference, multipath) before GPS flight."
        else:
            sev = Severity.WARN
            threshold = HDOP_WARN
            brief = (
                f" It was above {HDOP_CRIT:.0f} for at most {longest_crit:.1f} s, a dropout rather than lasting "
                "poor geometry."
                if hmax > HDOP_CRIT
                else ""
            )
            msg = (
                f"{label} HDOP peaked at {hmax:.1f} (above {HDOP_WARN:.1f}); satellite geometry is "
                f"poor and the horizontal position is loose. Good HDOP is under ~1.5.{brief}"
            )
            rec = "Aim for HDOP under ~1.5 before arming in GPS modes; improve antenna placement/sky view."

        ivs = intervals_above(times, hdop, threshold)
        t0 = ivs[0].start_s if ivs else None
        t1 = ivs[-1].end_s if ivs else None
        return make_finding(
            self.id,
            sev,
            f"High {label} HDOP, peak {hmax:.1f}",
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
