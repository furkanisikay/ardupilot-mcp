"""In-memory flight-log domain model.

Deliberately free of any ``pymavlink`` dependency: checks, the orchestrator and
their tests all operate on this pure structure, so the entire logic layer is
testable with synthetic logs and never needs a real ``.bin`` file. Binary
parsing lives in :mod:`ardupilot_mcp.parser` and produces a ``FlightLog``.

A "record" is a plain ``dict`` of a DataFlash message's fields. Every record is
normalised to carry a ``timestamp`` key in seconds (derived from ``TimeUS`` when
present). Times are seconds-since-boot and monotonic within a log.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .model import LogIntegrity, LogSummary

Record = dict[str, float | int | str]


@dataclass
class LogMeta:
    vehicle_type: str | None = None
    vehicle_kind: str | None = None  # copter/heli/plane/rover/sub/tracker/unknown
    firmware_version: str | None = None
    board: str | None = None
    start_time_s: float | None = None
    duration_s: float | None = None
    integrity: LogIntegrity = LogIntegrity.OK
    integrity_detail: str | None = None
    flight_modes: list[str] = field(default_factory=list)
    max_altitude_m: float | None = None


def _timestamp_of(rec: Record) -> float | None:
    """Best-effort seconds timestamp for a record."""
    if "timestamp" in rec:
        return float(rec["timestamp"])
    if "TimeUS" in rec:
        return float(rec["TimeUS"]) / 1_000_000.0
    if "TimeMS" in rec:
        return float(rec["TimeMS"]) / 1000.0
    return None


class FlightLog:
    """A parsed flight log: messages grouped by type, plus params and metadata."""

    def __init__(
        self,
        messages: dict[str, list[Record]],
        params: dict[str, float] | None = None,
        meta: LogMeta | None = None,
        path: str = "",
        _copy: bool = True,
    ) -> None:
        self.path = path
        self.params: dict[str, float] = dict(params or {})
        self.meta = meta or LogMeta()
        # Normalise: stamp a numeric 'timestamp' on each record. ``_copy=False``
        # (used by the parser, which hands over fresh dicts) skips a per-record
        # copy — a meaningful saving on logs with ~100k records.
        self._messages: dict[str, list[Record]] = {}
        for mtype, records in messages.items():
            norm: list[Record] = []
            for rec in records:
                r = dict(rec) if _copy else rec
                ts = _timestamp_of(r)
                if ts is not None:
                    r["timestamp"] = ts
                norm.append(r)
            self._messages[mtype] = norm
        self._rebase_times()

    # -- construction helpers ------------------------------------------------

    #: High-rate flight-controller messages that reliably carry the boot clock.
    #: Used to anchor the flight duration so a message family on a *different*
    #: clock (e.g. old GPS-week time in pre-2015 logs) can't distort it.
    _CLOCK_REFERENCE_MSGS = ("IMU", "IMU2", "ATT", "CTUN", "RATE", "NKF1", "XKF1", "BARO", "RCOU")

    def _rebase_times(self) -> None:
        """Shift record timestamps to be flight-relative (start at 0).

        Raw DataFlash ``TimeUS`` is microseconds since boot, so a log that began
        logging 200 s after power-up has timestamps starting at ~200 s. That is
        confusing in a diagnosis ("clipping at 905 s" on a "781 s flight"), so we
        rebase every timestamp to seconds-from-log-start. ``meta.start_time_s``
        keeps the original boot-relative origin for reference, and all findings,
        ``query_timeseries`` and ``list_events`` then share one intuitive clock.

        The flight *duration* is taken from a high-rate flight-controller stream
        when available, not the global min/max, so an oddly-clocked message family
        (some pre-2015 logs put GPS time-of-week in GPS records) cannot inflate it.
        """
        all_ts: list[float] = []
        for records in self._messages.values():
            for r in records:
                if "timestamp" in r:
                    all_ts.append(float(r["timestamp"]))
        if not all_ts:
            return
        ref = self._reference_span()
        if ref is not None:
            lo, hi = ref  # robust FC-stream span (origin + duration)
        else:
            lo, hi = min(all_ts), max(all_ts)
        if lo != 0.0:
            for records in self._messages.values():
                for r in records:
                    if "timestamp" in r:
                        r["timestamp"] = float(r["timestamp"]) - lo
            hi -= lo
        if self.meta.start_time_s is None:
            self.meta.start_time_s = lo
        if self.meta.duration_s is None:
            self.meta.duration_s = max(0.0, hi)

    def _reference_span(self) -> tuple[float, float] | None:
        """Robust (start, end) of the reliable FC stream that spans the most time.

        Rejects gross outlier timestamps via an IQR fence (some old logs carry a
        single garbage TimeUS that would otherwise imply a multi-day flight) but
        keeps the exact min/max of the inliers, so a clean log's duration is
        unchanged. Picks the widest-spanning candidate so a stream that only
        covers part of the flight isn't chosen.
        """
        best: tuple[float, float] | None = None
        best_span = -1.0
        for name in self._CLOCK_REFERENCE_MSGS:
            records = self._messages.get(name)
            if not records:
                continue
            ts = np.array([float(r["timestamp"]) for r in records if "timestamp" in r])
            if ts.size < 10:
                continue
            q1, q3 = np.percentile(ts, [25, 75])
            iqr = q3 - q1
            inliers = ts
            if iqr > 0:
                fence = 5.0 * iqr  # far enough that clean data is never trimmed
                kept = ts[(ts >= q1 - fence) & (ts <= q3 + fence)]
                if kept.size:
                    inliers = kept
            lo, hi = float(inliers.min()), float(inliers.max())
            if hi - lo > best_span:
                best_span = hi - lo
                best = (lo, hi)
        return best

    # -- access --------------------------------------------------------------

    @property
    def available_messages(self) -> set[str]:
        return set(self._messages.keys())

    def has(self, mtype: str) -> bool:
        return bool(self._messages.get(mtype))

    def has_all(self, *mtypes: str) -> bool:
        return all(self.has(m) for m in mtypes)

    def get(self, mtype: str) -> list[Record]:
        return self._messages.get(mtype, [])

    def count(self, mtype: str) -> int:
        return len(self._messages.get(mtype, []))

    def message_counts(self) -> dict[str, int]:
        return {m: len(r) for m, r in sorted(self._messages.items())}

    def times(self, mtype: str) -> np.ndarray:
        """Timestamps (s) for a message type, in record order."""
        recs = self._messages.get(mtype, [])
        return np.array([float(r["timestamp"]) for r in recs if "timestamp" in r], dtype=float)

    def field(self, mtype: str, name: str) -> np.ndarray:
        """Numeric values of one field across all records of a type (missing skipped)."""
        recs = self._messages.get(mtype, [])
        vals = [r[name] for r in recs if name in r and _is_number(r[name])]
        return np.array(vals, dtype=float)

    def armed_window(self) -> tuple[float | None, float | None]:
        """(first-armed, last-disarmed) times from EV events, or (None, None).

        Lets checks ignore the pre-arm (e.g. GPS still acquiring) and post-land
        phases, which otherwise look like in-flight faults.
        """
        ev = self._messages.get("EV", [])
        arms = [float(r["timestamp"]) for r in ev if r.get("Id") == 10 and "timestamp" in r]
        disarms = [float(r["timestamp"]) for r in ev if r.get("Id") == 11 and "timestamp" in r]
        return (min(arms) if arms else None, max(disarms) if disarms else None)

    def clip_to_window(
        self, times: np.ndarray, values: np.ndarray, window: tuple[float | None, float | None]
    ) -> tuple[np.ndarray, np.ndarray]:
        """Restrict an aligned (times, values) pair to a [t0, t1] window (open-ended ok)."""
        t0, t1 = window
        if t0 is None and t1 is None:
            return times, values
        mask = np.ones(times.shape, dtype=bool)
        if t0 is not None:
            mask &= times >= t0
        if t1 is not None:
            mask &= times <= t1
        return times[mask], values[mask]

    def series(self, mtype: str, name: str) -> tuple[np.ndarray, np.ndarray]:
        """Aligned (timestamps, values) for one field, restricted to records that have both."""
        recs = self._messages.get(mtype, [])
        ts: list[float] = []
        vs: list[float] = []
        for r in recs:
            if name in r and _is_number(r[name]) and "timestamp" in r:
                ts.append(float(r["timestamp"]))
                vs.append(float(r[name]))
        return np.array(ts, dtype=float), np.array(vs, dtype=float)

    # -- summary -------------------------------------------------------------

    def summary(self) -> LogSummary:
        return LogSummary(
            path=self.path,
            vehicle_type=self.meta.vehicle_type,
            firmware_version=self.meta.firmware_version,
            board=self.meta.board,
            duration_s=self.meta.duration_s,
            start_time_s=self.meta.start_time_s,
            available_messages=sorted(self.available_messages),
            message_counts=self.message_counts(),
            flight_modes=list(self.meta.flight_modes),
            max_altitude_m=self.meta.max_altitude_m,
            integrity=self.meta.integrity,
            integrity_detail=self.meta.integrity_detail,
        )


def _is_number(v: object) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)
