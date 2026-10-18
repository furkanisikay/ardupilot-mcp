# SPDX-License-Identifier: AGPL-3.0-only
"""In-memory flight-log domain model.

Deliberately free of any ``pymavlink`` dependency: checks, the orchestrator and
their tests all operate on this pure structure, so the entire logic layer is
testable with synthetic logs and never needs a real ``.bin`` file. Binary
parsing lives in :mod:`ardupilot_mcp.parser` and produces a ``FlightLog``.

Storage is columnar: each message type is a table of numpy arrays (one per
field) plus a flight-relative ``timestamp`` array in seconds (derived from
``TimeUS``/``TimeMS``). A 20-minute log with 400 Hz IMU data is ~1.6 M records;
as one Python dict per record it took ~1.5 GB, as columns it takes ~100 MB.
``get()`` still hands out plain ``dict`` records (with a ``timestamp`` key) for
the small, event-like message types checks iterate over (ERR/EV/MODE/MSG/...).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from .model import LogIntegrity, LogSummary

Record = dict[str, float | int | str]

#: Field that carries the sensor/core instance for multi-instance messages. Modern
#: firmware logs every instance under ONE message name (e.g. all IMUs as ``VIBE``),
#: distinguished only by this field (the ``#`` unit in the format's FMTU). Used for
#: synthetic logs; the parser overrides it with the log's own FMTU declarations.
#: Source: Copter-4.6.3 LogStructure.h (AP_InertialSensor, AP_GPS, AP_Logger,
#: AP_BattMonitor, AP_NavEKF3, AP_Baro).
DEFAULT_INSTANCE_FIELDS: dict[str, str] = {
    "VIBE": "IMU",
    "IMU": "I",
    "GPS": "I",
    "GPA": "I",
    "MAG": "I",
    "BAT": "Inst",
    "BARO": "I",
    "XKF1": "C",
    "XKF2": "C",
    "XKF3": "C",
    "XKF4": "C",
    "XKF5": "C",
    "RFND": "Instance",
}

#: Before 4.1 a second (third, ...) sensor or EKF core was logged under its own message
#: name with the same fields: base name -> (instance field, member names, member i is
#: instance i). Folded into the base name with the modern instance field, so every check
#: sees them as it does on 4.1+ logs. Source: Copter-3.5.7 / 3.6.12 / 4.0.7
#: LogStructure.h (IMU2/3, GPS2, GPA2, MAG2/3, BAR2/3, BAT2-9, CUR2). The EKF loggers write
#: core 0 as NKF1-4 / XKF1-4, core 1 as NKF6-9 / XKF6-9 and, from 4.0, core 2 as NK11-14 /
#: XK11-14 (NKF5 / XKF5 is written for one core only).
LEGACY_INSTANCES: dict[str, tuple[str, tuple[str, ...]]] = {
    "IMU": ("I", ("IMU", "IMU2", "IMU3")),
    "GPS": ("I", ("GPS", "GPS2")),
    "GPA": ("I", ("GPA", "GPA2")),
    "MAG": ("I", ("MAG", "MAG2", "MAG3")),
    "BARO": ("I", ("BARO", "BAR2", "BAR3")),
    "BAT": ("Inst", ("BAT",) + tuple(f"BAT{k}" for k in range(2, 10))),
    "CURR": ("Inst", ("CURR", "CUR2")),
    **{
        f"{ekf}F{k}": ("C", (f"{ekf}F{k}", f"{ekf}F{k + 5}", f"{ekf}1{k}"))
        for ekf in ("NK", "XK")
        for k in range(1, 5)
    },
}


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
    #: "dataflash" (the onboard .bin log) or "tlog" (a ground station's MAVLink telemetry log).
    source: Literal["dataflash", "tlog"] = "dataflash"
    #: True when the log records the arm state throughout (a telemetry log's heartbeats), so
    #: its arm/disarm records are complete and no arming is inferred from LOG_DISARMED.
    arming_recorded: bool = False
    flight_modes: list[str] = field(default_factory=list)
    max_altitude_m: float | None = None


@dataclass
class ImuBatch:
    """One batch-sampler capture (ISBH header + its ISBD blocks): raw IMU at full rate.

    AP_InertialSensor's BatchSampler logs ``smp_cnt`` consecutive samples of one sensor
    as int16 scaled by ``mul`` (BatchSampler.cpp). ``post_filter`` is True for samples taken
    after the filters, False for raw ones, None when the log cannot tell (INS_LOG_BAT_OPT).
    """

    time_s: float  # when the header was written (ISBH.TimeUS), on the log's rebased clock
    sensor: str  # "gyro" or "accel"
    instance: int
    post_filter: bool | None
    rate_hz: float
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray


class _Table:
    """One message type: equal-length field columns plus a seconds time base.

    Numeric columns hold NaN where a record lacks the field (dict-built tables);
    string columns are object arrays with ``None`` for missing values.
    """

    __slots__ = ("cols", "n", "t", "int_cols", "_records")

    def __init__(self, cols: dict[str, np.ndarray], n: int, int_cols: set[str] | None = None) -> None:
        self.cols = cols
        self.n = n
        self.int_cols = (
            int_cols if int_cols is not None else {k for k, v in cols.items() if v.dtype.kind in "iu"}
        )
        self.t = _time_base(cols, n)
        self._records: list[Record] | None = None

    @classmethod
    def from_records(cls, records: list[Record]) -> _Table:
        n = len(records)
        keys: dict[str, None] = {}
        for r in records:
            for k in r:
                keys.setdefault(k, None)
        cols: dict[str, np.ndarray] = {}
        ints: set[str] = set()
        for k in keys:
            vals = [r.get(k) for r in records]
            present = [v for v in vals if v is not None]
            if present and all(_is_number(v) for v in present):
                cols[k] = np.array([np.nan if v is None else float(v) for v in vals], dtype=float)
                if all(isinstance(v, int) for v in present):
                    ints.add(k)
            else:
                arr = np.empty(n, dtype=object)
                arr[:] = vals
                cols[k] = arr
        return cls(cols, n, ints)

    def records(self) -> list[Record]:
        """The table as plain dicts (built once, then cached)."""
        if self._records is not None:
            return self._records
        names = [k for k in self.cols if k != "timestamp"]
        out: list[Record] = []
        for i in range(self.n):
            rec: Record = {}
            for k in names:
                v = self.cols[k][i]
                if v is None:
                    continue
                if isinstance(v, (str, bytes)):
                    rec[k] = v.decode(errors="backslashreplace") if isinstance(v, bytes) else v
                    continue
                fv = float(v)
                if math.isnan(fv):
                    # Missing in a dict-built table (an int column with gaps is stored as
                    # float NaN); a real NaN sample is indistinguishable and equally
                    # unusable, so it is left out too.
                    continue
                rec[k] = int(v) if k in self.int_cols else fv
            ts = self.t[i]
            if not math.isnan(ts):
                rec["timestamp"] = float(ts)
            out.append(rec)
        self._records = out
        return out


def _time_base(cols: dict[str, np.ndarray], n: int) -> np.ndarray:
    """Seconds timestamps per record from timestamp/TimeUS/TimeMS (NaN when absent)."""
    for name, per_s in (("timestamp", 1.0), ("TimeUS", 1e6), ("TimeMS", 1e3)):
        col = cols.get(name)
        if col is not None and col.dtype != object:
            # Divide (not multiply by 1e-6): exact for whole-ms stamps, so window
            # bounds like start_s=0.2 include the sample logged at 200000 us.
            return col.astype(float) / per_s
    return np.full(n, np.nan)


class FlightLog:
    """A parsed flight log: messages grouped by type, plus params and metadata."""

    def __init__(
        self,
        messages: dict[str, list[Record]] | None = None,
        params: dict[str, float] | None = None,
        meta: LogMeta | None = None,
        path: str = "",
        _copy: bool = True,
        instance_fields: dict[str, str] | None = None,
        columns: dict[str, dict[str, np.ndarray]] | None = None,
    ) -> None:
        self.path = path
        self.params: dict[str, float] = dict(params or {})
        self.meta = meta or LogMeta()
        self._instance_fields = dict(DEFAULT_INSTANCE_FIELDS)
        if instance_fields:
            self._instance_fields.update(instance_fields)
        # mtype -> {instance: record indices}; built lazily per type.
        self._instance_index: dict[str, dict[int, np.ndarray]] = {}
        self._tables: dict[str, _Table] = {}
        # Batch-sampler captures (ISBH/ISBD), kept apart from the per-record tables and
        # decoded on first use (only the notch advice reads them; see ``batches``).
        self._batches: list[ImuBatch] = []
        self._batch_loader: Callable[[FlightLog], list[ImuBatch]] | None = None
        for mtype, records in (messages or {}).items():
            self._tables[mtype] = _Table.from_records(records)
        for mtype, cols in (columns or {}).items():
            n = len(next(iter(cols.values()))) if cols else 0
            self._tables[mtype] = _Table(cols, n)
        #: Base names whose instances were folded in from pre-4.1 per-instance messages.
        self.merged_legacy: dict[str, list[str]] = self._merge_legacy_instances()
        self._rebase_times()

    # -- construction helpers ------------------------------------------------

    def _merge_legacy_instances(self) -> dict[str, list[str]]:
        """Fold pre-4.1 per-instance messages (IMU2, NKF6, ...) into their base name.

        Only when the base has no instance field of its own and every member has exactly
        the base's fields, so a modern log, or an unrelated message that happens to share a
        name, is never touched. The member tables stay available under their own names.
        """
        merged: dict[str, list[str]] = {}
        for base, (field_name, members) in LEGACY_INSTANCES.items():
            present = [(i, m) for i, m in enumerate(members) if m in self._tables and self._tables[m].n > 0]
            if len(present) < 2 or present[0][1] != base:
                continue
            names = set(self._tables[base].cols)
            declared = self._instance_fields.get(base)
            if field_name in names or (declared is not None and declared in names):
                continue
            if any(set(self._tables[m].cols) != names for _, m in present):
                continue
            tables = [self._tables[m] for _, m in present]
            inst = np.concatenate(
                [np.full(t.n, i, dtype=np.int64) for (i, _), t in zip(present, tables, strict=True)]
            )
            # Interleave by time, as 4.1+ logs them (stable: equal stamps keep member order).
            # Reordered column by column so only one concatenated temporary is alive at a time.
            order = np.argsort(np.concatenate([t.t for t in tables]), kind="stable")
            cols = {k: np.concatenate([t.cols[k] for t in tables])[order] for k in self._tables[base].cols}
            cols[field_name] = inst[order]
            # A field is integer only if it is integer in every member (else int() truncates).
            int_cols = set.intersection(*(t.int_cols for t in tables)) | {field_name}
            self._tables[base] = _Table(cols, int(inst.size), int_cols)
            self._instance_fields[base] = field_name
            merged[base] = [m for _, m in present]
        return merged

    #: High-rate flight-controller messages that reliably carry the boot clock.
    #: Used to anchor the flight duration so a message family on a *different*
    #: clock (e.g. old GPS-week time in pre-2015 logs) can't distort it.
    _CLOCK_REFERENCE_MSGS = ("IMU", "IMU2", "ATT", "CTUN", "RATE", "NKF1", "XKF1", "BARO", "RCOU")

    def _rebase_times(self) -> None:
        """Shift timestamps to be flight-relative (start at 0).

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
        finite = [tb.t[np.isfinite(tb.t)] for tb in self._tables.values()]
        finite = [f for f in finite if f.size]
        if not finite:
            return
        ref = self._reference_span()
        if ref is not None:
            lo, hi = ref  # robust FC-stream span (origin + duration)
        else:
            lo = min(float(f.min()) for f in finite)
            hi = max(float(f.max()) for f in finite)
        if lo != 0.0:
            for tb in self._tables.values():
                tb.t = tb.t - lo
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
            tb = self._tables.get(name)
            if tb is None or not tb.n:
                continue
            ts = tb.t[np.isfinite(tb.t)]
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
    def batches(self) -> list[ImuBatch]:
        """Batch-sampler captures, decoded by the parser's loader the first time they are read.

        Raw sensor-rate data can be most of a large log, so it is not held for tools that
        never look at it. A loader that fails leaves no batches (they are optional data).
        """
        loader, self._batch_loader = self._batch_loader, None
        if loader is not None:
            try:
                self._batches = loader(self)
            except Exception:  # noqa: BLE001 - batch data is optional; never fail a tool over it
                self._batches = []
        return self._batches

    @batches.setter
    def batches(self, value: list[ImuBatch]) -> None:
        self._batches = list(value)
        self._batch_loader = None

    @property
    def available_messages(self) -> set[str]:
        return set(self._tables.keys())

    def has(self, mtype: str) -> bool:
        tb = self._tables.get(mtype)
        return tb is not None and tb.n > 0

    def has_all(self, *mtypes: str) -> bool:
        return all(self.has(m) for m in mtypes)

    def fields(self, mtype: str) -> list[str]:
        """Field names logged for ``mtype`` (excluding the derived ``timestamp``)."""
        tb = self._tables.get(mtype)
        return [k for k in tb.cols if k != "timestamp"] if tb is not None else []

    def get(self, mtype: str, instance: int | None = None) -> list[Record]:
        """Records of a type as dicts; with ``instance``, only that sensor/core's records.

        Meant for small, event-like types. For high-rate numeric data use
        :meth:`series` / :meth:`times`, which never build per-record dicts.
        """
        tb = self._tables.get(mtype)
        if tb is None:
            return []
        recs = tb.records()
        if instance is None:
            return recs
        idx = self._index(mtype, instance)
        if idx is None:
            return []
        return recs[idx] if isinstance(idx, slice) else [recs[i] for i in idx]

    def count(self, mtype: str) -> int:
        tb = self._tables.get(mtype)
        return tb.n if tb is not None else 0

    def message_counts(self) -> dict[str, int]:
        """Records per message type; a folded pre-4.1 member (IMU2) counts only under its base."""
        folded = {m for base, members in self.merged_legacy.items() for m in members if m != base}
        return {m: tb.n for m, tb in sorted(self._tables.items()) if m not in folded}

    def instance_field(self, mtype: str) -> str | None:
        """Name of the field that carries the instance for ``mtype``, if it has one in this log."""
        name = self._instance_fields.get(mtype)
        tb = self._tables.get(mtype)
        if name is None or tb is None or name not in tb.cols:
            return None
        return name

    def instances(self, mtype: str) -> list[int]:
        """Sorted instance ids present for ``mtype`` (``[0]`` for a single-instance stream)."""
        if not self.has(mtype):
            return []
        if self.instance_field(mtype) is None:
            return [0]
        return sorted(self._instance_map(mtype))

    def _instance_map(self, mtype: str) -> dict[int, np.ndarray]:
        cached = self._instance_index.get(mtype)
        if cached is not None:
            return cached
        out: dict[int, np.ndarray] = {}
        name = self.instance_field(mtype)
        if name is not None:
            col = self._numeric(self._tables[mtype].cols[name])
            for v in np.unique(col[np.isfinite(col)]):
                out[int(v)] = np.flatnonzero(col == v)
        self._instance_index[mtype] = out
        return out

    def _index(self, mtype: str, instance: int | None) -> np.ndarray | slice | None:
        """Record selector for ``instance`` (None = all records; None result = no such instance).

        All records are selected with a slice, so indexing is a view, not a copy of the column.
        """
        tb = self._tables.get(mtype)
        if tb is None:
            return None
        if instance is None:
            return slice(None)
        if self.instance_field(mtype) is None:
            # Not a multi-instance type: the whole stream is instance 0.
            return slice(None) if int(instance) == 0 else None
        return self._instance_map(mtype).get(int(instance))

    @staticmethod
    def _numeric(col: np.ndarray) -> np.ndarray:
        if col.dtype != object:
            return col.astype(float, copy=False)
        return np.array([float(v) if _is_number(v) else np.nan for v in col], dtype=float)

    def times(self, mtype: str, instance: int | None = None) -> np.ndarray:
        """Timestamps (s) for a message type, in record order."""
        idx = self._index(mtype, instance)
        if idx is None:
            return np.array([], dtype=float)
        t = self._tables[mtype].t[idx]
        return t[np.isfinite(t)]

    def field(self, mtype: str, name: str, instance: int | None = None) -> np.ndarray:
        """Numeric values of one field across all records of a type (missing/NaN skipped)."""
        idx = self._index(mtype, instance)
        tb = self._tables.get(mtype)
        if idx is None or tb is None or name not in tb.cols:
            return np.array([], dtype=float)
        v = self._numeric(tb.cols[name][idx])
        return v[np.isfinite(v)]

    def series(self, mtype: str, name: str, instance: int | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Aligned (timestamps, values) for one field, restricted to records that have both.

        Pass ``instance`` for multi-instance messages (VIBE/GPS/MAG/BAT/IMU/XKF*):
        without it, every sensor's samples are interleaved in one series. NaN/inf
        samples (corrupt sensor reads) are dropped: they would poison max/mean
        comparisons and silently hide findings.
        """
        idx = self._index(mtype, instance)
        tb = self._tables.get(mtype)
        if idx is None or tb is None or name not in tb.cols:
            return np.array([], dtype=float), np.array([], dtype=float)
        t = tb.t[idx]
        v = self._numeric(tb.cols[name][idx])
        ok = np.isfinite(t) & np.isfinite(v)
        return t[ok], v[ok]

    def aligned(
        self, mtype: str, names: list[str], instance: int | None = None
    ) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        """Record-aligned (times, {field: values}) — NaN where a record lacks a numeric value.

        Unlike :meth:`series`, records are never dropped, so every field lines up with
        the same time axis (used by ``query_timeseries``).
        """
        idx = self._index(mtype, instance)
        tb = self._tables.get(mtype)
        if idx is None or tb is None:
            return np.array([], dtype=float), {n: np.array([], dtype=float) for n in names}
        t = tb.t[idx]
        keep = np.isfinite(t)
        out: dict[str, np.ndarray] = {}
        for n in names:
            col = tb.cols.get(n)
            out[n] = self._numeric(col[idx])[keep] if col is not None else np.full(int(keep.sum()), np.nan)
        return t[keep], out

    @property
    def is_telemetry(self) -> bool:
        """A ground station's telemetry log: a missing message or a gap is the link, not the vehicle."""
        return self.meta.source == "tlog"

    @property
    def never_armed(self) -> bool:
        """True when the log records the arm state throughout and the vehicle never armed."""
        return self.meta.arming_recorded and not self.armed_intervals()

    def was_armed_at(self, t: float) -> bool:
        """Whether the vehicle was armed at ``t`` (True when arming is unknown)."""
        ivs = self.armed_intervals()
        if not ivs:
            return not self.meta.arming_recorded
        return any(a <= t <= b for a, b in ivs)

    def armed_window(self) -> tuple[float | None, float | None]:
        """(first-armed, last-disarmed) times, or (None, None) when arming is unknown.

        Lets checks ignore the pre-arm (e.g. GPS still acquiring) and post-land
        phases, which otherwise look like in-flight faults. From ``armed_intervals``,
        so EV and ARM records (and a telemetry log's heartbeats) all count.
        """
        ivs = self.armed_intervals()
        if not ivs:
            return None, None
        return ivs[0][0], ivs[-1][1]

    def armed_intervals(self) -> list[tuple[float, float]]:
        """Each (arm, disarm) interval, in order. A log can hold several flights.

        From EV ARMED(10)/DISARMED(11), which AP_Arming writes for every vehicle in
        4.x, together with ``ARM.ArmState`` (1 = armed). Two real-log cases matter:

        * With ``LOG_DISARMED=0`` logging starts *at* arming, so the arm record is
          often written before the file opens. A log whose first edge is a disarm was
          therefore armed from its start.
        * Such a log may carry no edge at all (the disarm was not reached either). When
          ``LOG_DISARMED`` is 0 the whole log was recorded while armed.

        An arm with no matching disarm (crash, truncated log) runs to the end of the log.
        Returns ``[]`` when arming cannot be determined.
        """
        edges: list[tuple[float, bool]] = []
        for r in self.get("EV"):
            if r.get("Id") in (10, 11) and "timestamp" in r:
                edges.append((float(r["timestamp"]), r.get("Id") == 10))
        for r in self.get("ARM"):
            if _is_number(r.get("ArmState")) and "timestamp" in r:
                edges.append((float(r["timestamp"]), bool(r["ArmState"])))
        edges.sort()
        end = float(self.meta.duration_s) if self.meta.duration_s is not None else None
        complete = self.meta.arming_recorded  # every arm and disarm is recorded
        if not edges:
            if not complete and self.params.get("LOG_DISARMED") == 0 and end is not None:
                return [(0.0, end)]
            return []  # with complete records: never armed
        out: list[tuple[float, float]] = []
        # An onboard log whose first record is a disarm was armed from its start; complete
        # records have no such gap.
        start: float | None = 0.0 if not edges[0][1] and not complete else None
        for t, armed in edges:
            if armed and start is None:
                start = t
            elif not armed and start is not None:
                out.append((start, t))
                start = None
        if start is not None:
            last = end if end is not None else start
            out.append((start, max(start, last)))
        return out

    def armed_mask(self, times: np.ndarray) -> np.ndarray | None:
        """Boolean mask of ``times`` that fall inside any armed interval (None if never armed)."""
        ivs = self.armed_intervals()
        if not ivs:
            # Known never armed: no sample is armed. Otherwise arming is unknown.
            return np.zeros(times.shape, dtype=bool) if self.meta.arming_recorded else None
        mask = np.zeros(times.shape, dtype=bool)
        for t0, t1 in ivs:
            mask |= (times >= t0) & (times <= t1)
        return mask

    def flying_mask(self, times: np.ndarray) -> np.ndarray | None:
        """``times`` that are armed AND not on the ground (None if arming is unknown).

        Plane logs ``STAT.isFlying`` (ArduPlane/Log.cpp), which is used when present.
        Copter's land detector logs EV LAND_COMPLETE (18) and NOT_LANDED (28)
        (ArduCopter/land_detector.cpp). When the log has them, armed time before a
        NOT_LANDED and after a LAND_COMPLETE is ground time: spool-up, touch-down and
        a crashed airframe lying on its side are not flight. Without them this is the
        armed mask.
        """
        armed = self.armed_mask(times)
        if armed is None:
            return None
        # ArduPlane: STAT.isFlying ("True if aircraft is probably flying", ArduPlane/Log.cpp).
        st, is_flying = self.series("STAT", "isFlying")
        if is_flying.size:
            idx = np.searchsorted(st, times, side="right") - 1
            known = idx >= 0
            flying_now = np.zeros(times.shape, dtype=bool)
            flying_now[known] = is_flying[idx[known]] > 0
            return armed & flying_now
        edges = sorted(
            (float(r["timestamp"]), r.get("Id") == 28)
            for r in self.get("EV")
            if r.get("Id") in (18, 28) and "timestamp" in r
        )
        if not any(airborne for _, airborne in edges):
            return armed
        flying = np.zeros(times.shape, dtype=bool)
        state = False
        last = -np.inf
        for t, airborne in edges + [(np.inf, False)]:
            if state:
                flying |= (times >= last) & (times < t)
            state, last = airborne, t
        return armed & flying

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
            source=self.meta.source,
        )


def _is_number(v: object) -> bool:
    return isinstance(v, (int, float, np.integer, np.floating)) and not isinstance(v, bool)
