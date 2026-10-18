"""Parse an ArduPilot DataFlash ``.bin`` into a :class:`FlightLog`.

The only module that depends on ``pymavlink``. It is defensive by design:
crashed-vehicle logs are exactly the ones we must analyse, so a truncated or
corrupt tail yields a *partial* FlightLog (with an integrity flag) rather than an
exception.
"""

from __future__ import annotations

import contextlib
import functools
import io
import os

import numpy as np

from . import ardupilot_meta as meta
from .flight_log import FlightLog, ImuBatch, LogMeta
from .model import LogIntegrity

# Pure structural/metadata messages we don't keep as data rows, plus the raw
# high-rate sensor dumps (IMU batch sampler ISBH/ISBD, raw ACC/GYR) that no check
# reads: in large logs they are most of the records and would dominate memory.
_SKIP_TYPES = {"FMT", "FMTU", "UNIT", "MULT", "ISBH", "ISBD", "ACC", "GYR"}


def parse_bin(path: str) -> FlightLog:
    if not os.path.exists(path):
        raise FileNotFoundError(f"log file not found: {path}")

    from pymavlink import DFReader  # local import keeps the dependency at the edge

    integrity = LogIntegrity.OK
    integrity_detail: str | None = None
    columns: dict[str, dict[str, np.ndarray]] | None = None
    messages: dict[str, list[dict]] | None = None

    # DFReader writes corruption warnings to stderr (it does not raise) during both
    # the initial index pass and message reads. Capture them so we can flag a
    # truncated/corrupt log instead of leaking noise to the console.
    capture = io.StringIO()
    with contextlib.redirect_stderr(capture):
        try:
            reader = _open_reader(DFReader, path)
        except Exception as exc:  # noqa: BLE001
            raise ValueError(f"could not open DataFlash log {path!r}: {exc}") from exc
        try:
            # A partially written final record is dropped silently: it is what any
            # log looks like when the battery is unplugged after landing, so on its
            # own it says nothing (the integrity check judges "ended while armed").
            columns, _cut = _decode_columns(reader)
        except Exception:  # noqa: BLE001 - unusual layout: fall back to pymavlink's own decoding
            columns = None
            reader.rewind()
            messages, integrity, integrity_detail = _read_records(reader)

    integrity, integrity_detail = _assess_integrity(capture.getvalue(), integrity, integrity_detail)
    integrity, integrity_detail = _assess_tail(path, integrity, integrity_detail)

    log = FlightLog(
        messages=messages,
        columns=columns,
        meta=LogMeta(integrity=integrity, integrity_detail=integrity_detail),
        path=path,
        instance_fields=_extract_instance_fields(reader),
    )
    log.params = _extract_params(log, reader)
    fw, vehicle = _extract_firmware(log)
    kind = meta.vehicle_kind(vehicle, log.params.get("FRAME_CLASS"))
    log.meta.vehicle_type = vehicle
    log.meta.vehicle_kind = kind
    log.meta.firmware_version = fw
    log.meta.flight_modes = _extract_modes(log, kind)
    log.meta.max_altitude_m = _extract_max_alt(log)
    _close(reader)
    if columns is not None:
        # Decoded on first use: re-reading the file then costs one index pass, which only
        # the notch advice pays, instead of every tool holding the raw batches in memory.
        log._batch_loader = functools.partial(_load_batches, path)
    return log


def _load_batches(path: str, log: FlightLog) -> list[ImuBatch]:
    """Re-open ``path`` and decode its batch-sampler captures for ``log``."""
    from pymavlink import DFReader

    with contextlib.redirect_stderr(io.StringIO()):
        reader = _open_reader(DFReader, path)
        try:
            return _decode_batches(reader, log)
        finally:
            _close(reader)


def _close(reader) -> None:
    """Release the log's file handle and memory map now.

    pymavlink's reader sits in reference cycles, so without this the map stays open until
    a garbage collection, and on Windows the user cannot replace or delete the log while
    the server holds it. Every decoded column is a copy, so nothing still views the map;
    if something did, mmap refuses to close (BufferError) and the collector frees it later.
    """
    try:
        reader.close()
    except (BufferError, OSError, AttributeError):
        pass


def _open_reader(DFReader, path: str):
    """``DFReader_binary`` without its log-clock discovery.

    pymavlink looks for a GPS message to build a wall-clock time base; in a log with
    no GPS (indoor or optical-flow craft) that search decodes every message one by
    one, which took ~7 s on a 15 MB forum log. Our time base is TimeUS/TimeMS
    (see FlightLog), so a plain microsecond clock is all the reader needs.
    """

    class _Reader(DFReader.DFReader_binary):
        def init_clock(self) -> None:
            self.clock = DFReader.DFReaderClock_usec()

    return _Reader(path)


# DataFlash format char -> numpy dtype, mirroring pymavlink's FORMAT_TO_STRUCT
# (DFReader.py). c/C/e/E are scaled by 0.01 and L by 1e-7 on read, as pymavlink does.
_NP_DTYPES = {
    "b": "i1", "B": "u1", "M": "i1", "h": "<i2", "H": "<u2", "i": "<i4", "I": "<u4",
    "f": "<f4", "d": "<f8", "q": "<i8", "Q": "<u8", "c": "<i2", "C": "<u2", "e": "<i4",
    "E": "<u4", "L": "<i4", "n": "S4", "N": "S16", "Z": "S64",
}  # fmt: skip
_NP_SCALE = {"c": 0.01, "C": 0.01, "e": 0.01, "E": 0.01, "L": 1e-7}
# Bound the temporary (rows x bytes) gather index to ~32 MB per chunk.
_GATHER_BYTES = 4_000_000


def _gather(buf: np.ndarray, offs: np.ndarray, length: int, dtype: np.dtype) -> np.ndarray:
    """The records of ``length`` bytes at ``offs`` (header skipped), viewed as ``dtype``."""
    body = np.arange(3, length, dtype=np.int64)
    step = max(1, _GATHER_BYTES // max(1, body.size))
    parts = [buf[offs[i : i + step, None] + body] for i in range(0, offs.size, step)]
    return np.ascontiguousarray(np.concatenate(parts)).view(dtype).reshape(-1)


def _decode_columns(reader) -> tuple[dict[str, dict[str, np.ndarray]], int]:
    """Decode every message type into numpy columns using pymavlink's own index.

    pymavlink's indexer (``reader.offsets``, built by its C ``dfindexer`` when
    available) already knows where every record starts; this gathers those bytes
    and views them through a structured dtype built from the type's FMT, instead of
    creating one Python object per record. ~10x faster and ~15x less memory on
    large logs, and field-for-field identical to ``recv_match().to_dict()`` (checked
    by tests/test_parser.py). Returns (columns, records cut off at end-of-file).
    """
    buf = np.frombuffer(reader.data_map, dtype=np.uint8)
    out: dict[str, dict[str, np.ndarray]] = {}
    cut = 0
    for tid, fmt in reader.formats.items():
        if fmt.name in _SKIP_TYPES or tid >= len(reader.offsets):
            continue
        offs = np.asarray(reader.offsets[tid], dtype=np.int64)
        if offs.size == 0:
            continue
        whole = offs + fmt.len <= buf.size
        cut += int(offs.size - np.count_nonzero(whole))
        offs = offs[whole]
        # A type the structured view can't express (``a`` arrays, a format char we have
        # no dtype for such as ``g`` half-floats, or a length mismatch) is decoded by
        # pymavlink on its own, so one odd type never sends the whole log down the slow path.
        dtype = None
        if len(fmt.columns) == len(fmt.format) and all(ch in _NP_DTYPES for ch in fmt.format):
            dtype = np.dtype([(c, _NP_DTYPES[ch]) for c, ch in zip(fmt.columns, fmt.format, strict=True)])
        if dtype is None or dtype.itemsize != fmt.len - 3:
            out[fmt.name] = _columns_via_pymavlink(reader, fmt, offs)
            continue
        arr = _gather(buf, offs, fmt.len, dtype)
        cols: dict[str, np.ndarray] = {}
        for col, ch in zip(fmt.columns, fmt.format, strict=True):
            v = arr[col]
            if ch in _NP_SCALE:
                v = v.astype(np.float64) * _NP_SCALE[ch]
            elif ch in "nNZ":
                v = np.array([_cstr(x) for x in v], dtype=object)
            else:
                v = v.copy()  # detach from the gather buffer
            cols[col] = v
        out[fmt.name] = cols
    return out, cut


# INS_LOG_BAT_OPT bits (AP_InertialSensor.h BatchSampler): 0 sensor-rate, 1 post-filter,
# 2 pre- and post-filter. The sampler alternates pre- and post-filter passes; with pre+post,
# or post-filter at sensor rate, a post-filter batch is logged with instance + gyro count
# (Write_ISBH, AP_InertialSensor_Logging.cpp; doing_post_filter_logging, AP_InertialSensor.h).
_BAT_OPT_SENSOR_RATE, _BAT_OPT_POST, _BAT_OPT_PRE_POST = 1, 2, 4
# ISBH.type: IMU_SENSOR_TYPE_ACCEL = 0, IMU_SENSOR_TYPE_GYRO = 1 (AP_InertialSensor.h).
_BATCH_SENSOR = {0: "accel", 1: "gyro"}
_ISBD_ARRAY = ("<i2", (32,))  # ISBD x/y/z: int16[32] (LogStructure.h log_ISBD)


def _view_records(reader, buf: np.ndarray, name: str) -> tuple[np.ndarray, np.ndarray] | None:
    """(file offsets, structured records) of one type, ``a`` arrays as int16[32] sub-arrays."""
    found = [(tid, f) for tid, f in reader.formats.items() if f.name == name]
    if not found:
        return None
    tid, fmt = found[0]
    if tid >= len(reader.offsets):
        return None
    offs = np.asarray(reader.offsets[tid], dtype=np.int64)
    offs = offs[offs + fmt.len <= buf.size]
    if offs.size == 0 or len(fmt.columns) != len(fmt.format):
        return None
    fields: list[tuple] = []
    for col, ch in zip(fmt.columns, fmt.format, strict=True):
        if ch == "a":
            fields.append((col, *_ISBD_ARRAY))
        elif ch in _NP_DTYPES:
            fields.append((col, _NP_DTYPES[ch]))
        else:
            return None
    dtype = np.dtype(fields)
    if dtype.itemsize != fmt.len - 3:
        return None
    return offs, _gather(buf, offs, fmt.len, dtype)


def _decode_batches(reader, log: FlightLog) -> list[ImuBatch]:
    """Batch-sampler captures: each ISBH header joined to the ISBD blocks that follow it."""
    buf = np.frombuffer(reader.data_map, dtype=np.uint8)
    head, data = _view_records(reader, buf, "ISBH"), _view_records(reader, buf, "ISBD")
    if head is None or data is None:
        return []
    h_off, h = head
    d_off, d = data
    order = np.argsort(h_off)
    h_off, h = h_off[order], h[order]
    # Each ISBD belongs to the latest ISBH before it in the file with the same sequence
    # number (N is a uint16 and wraps on long logs).
    owner = np.searchsorted(h_off, d_off, side="right") - 1
    ok = owner >= 0
    ok[ok] = d["N"][ok] == h["N"][owner[ok]]
    if not np.any(ok):
        return []
    origin = log.meta.start_time_s or 0.0
    opt = int(log.params.get("INS_LOG_BAT_OPT", 0))
    n_imu = _imu_count(log)
    # Whether a post-filter batch carries the instance offset (see Write_ISBH).
    offset = bool(opt & _BAT_OPT_PRE_POST) or (bool(opt & _BAT_OPT_POST) and bool(opt & _BAT_OPT_SENSOR_RATE))
    seen = {(int(t), int(i)) for t, i in zip(h["type"], h["instance"], strict=True)}
    batches: list[ImuBatch] = []
    # Group the blocks by batch once (sorted by owner, then seqno) instead of masking per batch.
    d, owner = d[ok], owner[ok]
    by_batch = np.lexsort((d["seqno"], owner))
    d, owner = d[by_batch], owner[by_batch]
    keys, starts = np.unique(owner, return_index=True)
    for k, rows in zip(keys, np.split(d, starts[1:]), strict=True):
        # Keep the contiguous run of blocks from seqno 0 (a dropped block ends the batch).
        expect = np.arange(rows.size)
        run = int(np.argmax(rows["seqno"] != expect)) if np.any(rows["seqno"] != expect) else rows.size
        rows = rows[:run]
        hdr = h[k]
        sensor = _BATCH_SENSOR.get(int(hdr["type"]))
        mul = float(hdr["mul"]) or 1.0
        rate = float(hdr["smp_rate"])
        if rows.size == 0 or sensor is None or not rate > 0:
            continue
        inst = int(hdr["instance"])
        post: bool | None
        if offset and n_imu is None:
            # Without the gyro count the offset post-filter instances cannot be told apart.
            post = None
        else:
            n = n_imu or 1
            shifted = offset and inst >= n
            if shifted:
                inst -= n
            post = _batch_is_post(opt, shifted, (int(hdr["type"]), inst + n) in seen)
        batches.append(
            ImuBatch(
                time_s=float(hdr["TimeUS"]) / 1e6 - origin,
                sensor=sensor,
                instance=inst,
                post_filter=post,
                rate_hz=rate,
                x=rows["x"].reshape(-1).astype(np.float32) / mul,
                y=rows["y"].reshape(-1).astype(np.float32) / mul,
                z=rows["z"].reshape(-1).astype(np.float32) / mul,
            )
        )
    return batches


def _batch_is_post(opt: int, shifted: bool, sibling_shifted: bool) -> bool | None:
    """Whether a batch holds filtered samples (None when the log cannot tell).

    A pass flagged post-filter takes filtered samples only when sensor-rate sampling is
    off for that IMU: at sensor rate the backend always feeds the sampler raw samples
    (_notify_new_gyro/accel_sensor_rate_sample, AP_InertialSensor_Backend.cpp), though
    Write_ISBH still offsets the instance. ``shifted``: this batch carries the offset;
    ``sibling_shifted``: the same sensor's offset batches exist in the log.
    """
    sensor_rate = bool(opt & _BAT_OPT_SENSOR_RATE)
    if opt & _BAT_OPT_PRE_POST:
        if not shifted:
            return False
        # Offset either way; filtered only if this IMU was not sampled at sensor rate.
        return None if sensor_rate else True
    if opt & _BAT_OPT_POST:
        # With sensor rate, the offset appears only when sensor-rate sampling is on, and
        # then every pass is raw; without it every pass is filtered.
        return not (sensor_rate and (shifted or sibling_shifted))
    return False


def _imu_count(log: FlightLog) -> int | None:
    """Gyros on the board: the IMU instances logged, else the detected gyro IDs (None if unknown)."""
    n = len(log.instances("IMU"))
    if n == 0:
        n = sum(1 for k in ("INS_GYR_ID", "INS_GYR2_ID", "INS_GYR3_ID") if log.params.get(k))
    return n or None


def _cstr(raw: bytes) -> str:
    """pymavlink's null_term(): decode, then cut at the first NUL."""
    s = raw.decode(errors="backslashreplace")
    i = s.find("\0")
    return s[:i] if i != -1 else s


def _columns_via_pymavlink(reader, fmt, offs: np.ndarray) -> dict[str, np.ndarray]:
    """Columns for a type the vectorised path can't view (e.g. ``a`` int16[32] arrays)."""
    rows: list[dict] = []
    for ofs in offs:
        reader.offset = int(ofs)
        m = reader._parse_next()
        if m is None:
            continue
        d = m.to_dict()
        d.pop("mavpackettype", None)
        rows.append(d)
    cols: dict[str, np.ndarray] = {}
    for col in fmt.columns:
        vals = [r.get(col) for r in rows]
        if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in vals):
            cols[col] = np.array(vals, dtype=float)
        else:
            arr = np.empty(len(vals), dtype=object)
            arr[:] = vals
            cols[col] = arr
    return cols


def _read_records(reader) -> tuple[dict[str, list[dict]], LogIntegrity, str | None]:
    """Fallback: pymavlink's own per-message decoding (slow, but handles anything)."""
    messages: dict[str, list[dict]] = {}
    integrity = LogIntegrity.OK
    detail: str | None = None
    parsed = 0
    while True:
        try:
            m = reader.recv_match()
        except Exception as exc:  # noqa: BLE001 - truncated/corrupt tail
            integrity = LogIntegrity.TRUNCATED
            detail = f"parse stopped after {parsed} messages: {type(exc).__name__}: {exc}"
            break
        if m is None:
            break
        mtype = m.get_type()
        if mtype in _SKIP_TYPES:
            continue
        try:
            d = m.to_dict()
        except Exception:  # noqa: BLE001
            continue
        d.pop("mavpackettype", None)
        # Only TimeUS/TimeMS drive timestamps (handled in FlightLog). We do NOT
        # fall back to DFReader's _timestamp: for a GPS-clocked log that is a Unix
        # epoch (~1.7e9 s), and records without TimeUS (e.g. FILE) would otherwise
        # poison the time base and produce absurd durations.
        messages.setdefault(mtype, []).append(d)
        parsed += 1
    return messages, integrity, detail


def _extract_instance_fields(reader) -> dict[str, str]:
    """Message name -> instance field, from the log's own FMTU ``#`` unit declarations."""
    out: dict[str, str] = {}
    for fmt in (getattr(reader, "formats", None) or {}).values():
        name = getattr(fmt, "name", None)
        field = getattr(fmt, "instance_field", None)
        if isinstance(name, str) and isinstance(field, str):
            out[name] = field
    return out


# DataFlash tolerates up to ~528 trailing bytes of block padding; a longer run of a
# single fill byte (0x00 / 0xFF) at EOF is the signature of a log cut short by a
# power loss or crash. Detected from the raw file so it is robust regardless of
# whether pymavlink uses its fast (C) or slow (Python) indexer.
_PAD_TOLERANCE = 528


def _assess_tail(path: str, integrity: LogIntegrity, detail: str | None) -> tuple[LogIntegrity, str | None]:
    """Flag truncation from a long run of fill bytes (0x00/0xFF) at end-of-file."""
    if integrity != LogIntegrity.OK:
        return integrity, detail
    try:
        size = os.path.getsize(path)
        window = min(8192, size)
        with open(path, "rb") as fh:
            fh.seek(size - window)
            data = fh.read(window)
    except OSError:
        return integrity, detail
    if not data:
        return integrity, detail
    last = data[-1]
    if last not in (0x00, 0xFF):
        return integrity, detail
    run = 0
    for b in reversed(data):
        if b == last:
            run += 1
        else:
            break
    if run > _PAD_TOLERANCE:
        return LogIntegrity.TRUNCATED, (
            f"log ends with {run}{'+' if run == window else ''} bytes of 0x{last:02X} fill "
            f"(> {_PAD_TOLERANCE}); the log was likely truncated by a power loss or crash mid-write."
        )
    return integrity, detail


def _assess_integrity(
    stderr_text: str, integrity: LogIntegrity, detail: str | None
) -> tuple[LogIntegrity, str | None]:
    """Flag truncation from DFReader's stderr warnings ('bad header'/'bad msg').

    A run of these almost always means the log ends in garbage — a power loss or
    crash that cut the write short. That truncation is itself diagnostic.
    """
    if integrity != LogIntegrity.OK:
        return integrity, detail
    low = stderr_text.lower()
    if "bad header" in low or "bad msg" in low:
        lines = [ln for ln in stderr_text.strip().splitlines() if ln.strip()]
        first = lines[0][:140] if lines else ""
        return LogIntegrity.TRUNCATED, (
            f"DataFlash parser reported {len(lines)} corrupt/garbage record(s); the log is "
            f"likely truncated (power loss or crash mid-write). First: {first!r}"
        )
    return integrity, detail


def _extract_params(log: FlightLog, reader) -> dict[str, float]:
    """Final value of every parameter (last PARM record wins, as in pymavlink)."""
    out: dict[str, float] = {}
    for rec in log.get("PARM"):
        name, value = rec.get("Name"), rec.get("Value")
        if isinstance(name, str) and isinstance(value, (int, float)) and not isinstance(value, bool):
            out[name] = float(value)
    if out:
        return out
    raw = getattr(reader, "params", None) or {}
    for k, v in raw.items():
        try:
            out[str(k)] = float(v)
        except (TypeError, ValueError):
            continue
    return out


def _extract_firmware(log: FlightLog) -> tuple[str | None, str | None]:
    fw: str | None = None
    vehicle: str | None = None
    # Newer logs: VER message with a firmware string field.
    for rec in log.get("VER"):
        for key in ("FwString", "FWString", "fw_string"):
            if key in rec and isinstance(rec[key], str):
                fw = rec[key]  # type: ignore[assignment]
                vehicle = meta.vehicle_from_message(rec[key]) or vehicle  # type: ignore[arg-type]
                break
    # Classic: MSG text lines, the first matching a known vehicle prefix.
    for rec in log.get("MSG"):
        text = rec.get("Message") or rec.get("Msg")
        if isinstance(text, str):
            v = meta.vehicle_from_message(text)
            if v:
                vehicle = vehicle or v
                fw = fw or text.strip()
                break
    return fw, vehicle


def _extract_modes(log: FlightLog, kind: str | None = None) -> list[str]:
    seen: list[str] = []
    for rec in log.get("MODE"):
        num = rec.get("ModeNum", rec.get("Mode"))
        if not isinstance(num, (int, float, str)) or isinstance(num, bool):
            continue
        name = meta.mode_name(num if isinstance(num, str) else int(num), kind)
        if name not in seen:
            seen.append(name)
    return seen


# Altitude ABOVE HOME/take-off, in preference order. POS.Alt and GPS.Alt are
# AMSL (metres above sea level), so mixing them in would report e.g. ~585 m for a
# vehicle sitting on the ground at Canberra; only relative sources are used.
# POS.RelHomeAlt: AP_AHRS LogStructure.h; CTUN.Alt (Copter, relative to EKF origin)
# and BARO.Alt (relative to the ground-level calibration): Copter-4.6.3 sources.
_REL_ALT_SOURCES = (("POS", "RelHomeAlt"), ("CTUN", "Alt"), ("BARO", "Alt"))


def _extract_max_alt(log: FlightLog) -> float | None:
    """Maximum altitude above home (m), from the first relative source the log has."""
    for mtype, field in _REL_ALT_SOURCES:
        # First instance only: a second barometer is never mixed into the primary's altitude.
        insts = log.instances(mtype)
        _, vals = log.series(mtype, field, insts[0] if insts else None)
        if vals.size:
            return float(np.max(vals))
    return None
