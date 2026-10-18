"""Scheduler timing & log gaps check.

Two independent signals about whether the autopilot's main loop kept up and
whether the log itself is continuous:

* **Logging gaps** — pick the highest-rate message the log actually has among
  ("IMU", "ATT", "VIBE") and look at the spacing between consecutive
  timestamps. The expected spacing is the *median* dt (robust to outliers); any
  jump far larger than that is either a dropped logging window or a stalled
  main loop. Either way the log is not telling the whole story for that window,
  which matters for every other check.

* **Scheduler overruns** — ArduPilot's ``PM`` (performance monitoring) message
  records, per reporting window, how many main loops ran (``NL``/``NLoop``) and
  how many overran their budget (``NLon``). A high long-loop *percentage* means
  the flight controller was CPU-starved (a handful per window is normal).

This check never *requires* a specific message (``requires=set()``); it guards
each signal internally and returns ``[]`` when none of the relevant messages
are present.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# Message types that carry a regular high-rate timestamp, in descending order of
# typical logging rate. We use the first one the log actually has for gap analysis.
GAP_SOURCES = ("IMU", "ATT", "VIBE")

# A gap counts as anomalous only if it exceeds BOTH an absolute floor and a
# relative multiple of the nominal (median) sample spacing. The absolute floor
# stops us flagging slow-logged messages; the relative factor catches stalls in
# fast streams.
GAP_ABS_FLOOR_S = 0.5  # never flag a gap smaller than half a second
GAP_REL_FACTOR = 10.0  # ...or smaller than 10x the nominal sample spacing

# Cap how many gap findings we emit so a badly fragmented log does not flood the
# report; we report the largest gaps first.
MAX_GAP_FINDINGS = 3

# PM scheduler counters: "NLon" = long loops and "NL" (4.x; "NLoop" in older logs)
# = loops measured in the reporting window (Copter-4.6.3 AP_Logger/LogStructure.h).
# A few long loops per window are normal, so judge the percentage, as ArduPilot's
# LogAnalyzer TestPerformance did: a window is "slow" above 6 %; WARN (LogAnalyzer
# FAIL) when any window exceeds 10 % or more than 6 windows are slow; INFO (its WARN)
# when only the 6 % line is crossed.
# https://github.com/ArduPilot/ardupilot/blob/Copter-4.0.7/Tools/LogAnalyzer/tests/TestPerformance.py
PM_LONG_LOOP_FIELD = "NLon"
PM_LOOP_FIELDS = ("NL", "NLoop")
PM_SLOW_PCT = 6.0
PM_FAIL_PCT = 10.0
PM_FAIL_SLOW_RECORDS = 6


@register_check
class TimingCheck(Check):
    id = "timing"
    title = "Scheduler timing & log gaps"
    category = "performance"
    requires: set[str] = set()  # multi-source; guarded inside run()
    description = "Logging gaps / main-loop stalls (IMU/ATT/VIBE timing) and PM scheduler long-loop overruns."

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        findings.extend(self._gap_findings(log))
        findings.extend(self._pm_findings(log))
        return findings

    # -- logging gaps --------------------------------------------------------

    def _gap_findings(self, log: FlightLog) -> list[Finding]:
        mtype = next((m for m in GAP_SOURCES if log.has(m)), None)
        if mtype is None:
            return []
        # One instance's stream: interleaved IMUs would hide one sensor's logging stopping.
        times = log.times(mtype, log.instances(mtype)[0])
        if times.size < 3:
            # Need a few samples to establish a nominal spacing.
            return []

        dt = np.diff(times)
        # Only positive spacings define the nominal cadence (defensive against
        # any non-monotonic timestamps).
        positive = dt[dt > 0.0]
        if positive.size == 0:
            return []
        median_dt = float(np.median(positive))
        if median_dt <= 0.0:
            return []

        threshold = max(GAP_ABS_FLOOR_S, GAP_REL_FACTOR * median_dt)

        # Collect every anomalous gap with the timestamp it starts at. With
        # LOG_DISARMED=0 logging pauses at disarm and resumes at the next arm in the
        # same file, so a gap that starts while disarmed is expected, not a stall.
        armed = log.armed_intervals()
        gaps: list[tuple[float, float, float]] = []  # (gap_s, start_s, end_s)
        for i in range(dt.size):
            if dt[i] > threshold:
                start = float(times[i])
                if armed and not any(a <= start < b for a, b in armed):
                    continue
                gaps.append((float(dt[i]), start, float(times[i + 1])))
        if not gaps:
            return []

        # Largest gaps first, then cap.
        gaps.sort(key=lambda g: g[0], reverse=True)

        out: list[Finding] = []
        for gap_s, start_s, end_s in gaps[:MAX_GAP_FINDINGS]:
            out.append(
                make_finding(
                    self.id,
                    Severity.WARN,
                    "Logging gap / possible main-loop stall",
                    (
                        f"A {gap_s:.2f} s gap in {mtype} logging starts at t={start_s:.2f} s "
                        f"(resumes at t={end_s:.2f} s). The nominal {mtype} spacing is "
                        f"{median_dt * 1000:.1f} ms, so this gap is {gap_s / median_dt:.0f}x the "
                        f"expected interval (threshold {threshold:.2f} s). This means logging was "
                        "interrupted or the main loop stalled; data in that window is missing."
                    ),
                    recommendation=(
                        "Check SD-card write performance and CPU load (PM message); a stalled main "
                        "loop can also indicate an overloaded scheduler or a failing log device."
                    ),
                    time_start_s=start_s,
                    time_end_s=end_s,
                    message_types=[mtype],
                    samples={
                        "gap_s": round(gap_s, 3),
                        "gap_start_s": round(start_s, 3),
                        "nominal_dt_s": round(median_dt, 4),
                        "threshold_s": round(threshold, 3),
                    },
                )
            )
        return out

    # -- PM scheduler overruns ----------------------------------------------

    def _pm_findings(self, log: FlightLog) -> list[Finding]:
        if not log.has("PM"):
            return []
        loops_field = next((f for f in PM_LOOP_FIELDS if self._pm_has_field(log, f)), None)
        if loops_field is None or not self._pm_has_field(log, PM_LONG_LOOP_FIELD):
            return []
        times, nlon = log.series("PM", PM_LONG_LOOP_FIELD)
        _, nloop = log.series("PM", loops_field)
        n = min(nlon.size, nloop.size)
        if n == 0:
            return []
        times, nlon, nloop = times[:n], nlon[:n], nloop[:n]
        valid = nloop > 0
        if not np.any(valid):
            return []
        pct = np.zeros(n)
        pct[valid] = 100.0 * nlon[valid] / nloop[valid]
        max_pct = float(np.max(pct))
        slow_records = int(np.count_nonzero(pct > PM_SLOW_PCT))
        if max_pct <= PM_SLOW_PCT:
            return []  # a few long loops per reporting window are normal
        sev = (
            Severity.WARN if (max_pct > PM_FAIL_PCT or slow_records > PM_FAIL_SLOW_RECORDS) else Severity.INFO
        )
        worst = int(np.argmax(pct))
        _, extra = log.series("PM", "Ex")
        max_ex = float(np.max(extra)) if extra.size else 0.0
        ex_txt = (
            f" The scheduler was also adding up to {max_ex:.0f} us to each loop to absorb overruns (PM.Ex)."
            if max_ex > 0
            else ""
        )
        samples: dict[str, float] = {
            "max_long_loop_pct": round(max_pct, 1),
            "slow_records": slow_records,
            "pm_records": n,
            "long_loops_total": int(np.sum(nlon)),
        }
        if extra.size:
            samples["max_extra_loop_us"] = max_ex
        return [
            make_finding(
                self.id,
                sev,
                "Scheduler recorded long loops",
                (
                    f"Up to {max_pct:.1f}% of main loops ran over their time budget in one PM window "
                    f"(PM.NLon/PM.{loops_field}); {slow_records} of {n} windows exceeded {PM_SLOW_PCT:.0f}%. "
                    "Persistent long loops mean the flight controller was CPU-starved, which delays control "
                    "updates and degrades estimator/control performance." + ex_txt
                ),
                recommendation=(
                    "Reduce CPU load: lower logging/notch/EKF rates or disable unused features; "
                    "check for a fast loop rate set too high for the board."
                ),
                time_start_s=float(times[worst]),
                time_end_s=float(times[-1]),
                message_types=["PM"],
                samples=samples,
            )
        ]

    @staticmethod
    def _pm_has_field(log: FlightLog, name: str) -> bool:
        return any(name in r for r in log.get("PM"))
