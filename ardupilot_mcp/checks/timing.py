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
  records scheduler statistics, including a counter of "long loops" (the
  ``NLon`` field) where the main loop ran over its time budget. A nonzero total
  means the flight controller was, at times, CPU-starved.

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

# PM scheduler "long loop" counter field name(s). ArduPilot's PM message uses
# "NLon" for the number of long loops in the reporting window.
PM_LONG_LOOP_FIELDS = ("NLon",)


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
        times = log.times(mtype)
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

        # Collect every anomalous gap with the timestamp it starts at.
        gaps: list[tuple[float, float, float]] = []  # (gap_s, start_s, end_s)
        for i in range(dt.size):
            if dt[i] > threshold:
                gaps.append((float(dt[i]), float(times[i]), float(times[i + 1])))
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
        field = next((f for f in PM_LONG_LOOP_FIELDS if self._pm_has_field(log, f)), None)
        if field is None:
            return []

        recs = log.get("PM")
        total = 0
        for r in recs:
            v = r.get(field)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                total += int(v)
        if total <= 0:
            return []

        times = log.times("PM")
        t0 = float(times[0]) if times.size else None
        t1 = float(times[-1]) if times.size else None
        return [
            make_finding(
                self.id,
                Severity.WARN,
                "Scheduler recorded long loops",
                (
                    f"The PM scheduler log recorded {total} long loop(s) (field '{field}') — the "
                    "main loop ran over its time budget. Persistent long loops indicate the flight "
                    "controller was CPU-starved, which can delay control updates and degrade "
                    "estimator/control performance."
                ),
                recommendation=(
                    "Reduce CPU load: lower logging/notch/EKF rates or disable unused features; "
                    "check for a fast loop rate set too high for the board."
                ),
                time_start_s=t0,
                time_end_s=t1,
                message_types=["PM"],
                samples={"long_loops": total},
            )
        ]

    @staticmethod
    def _pm_has_field(log: FlightLog, name: str) -> bool:
        return any(name in r for r in log.get("PM"))
