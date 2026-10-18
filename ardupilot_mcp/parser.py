"""Parse an ArduPilot DataFlash ``.bin`` into a :class:`FlightLog`.

The only module that depends on ``pymavlink``. It is defensive by design:
crashed-vehicle logs are exactly the ones we must analyse, so a truncated or
corrupt tail yields a *partial* FlightLog (with an integrity flag) rather than an
exception.
"""

from __future__ import annotations

import contextlib
import io
import os

import numpy as np

from . import ardupilot_meta as meta
from .flight_log import FlightLog, LogMeta
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
    return log


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
        if "a" in fmt.format or len(fmt.columns) != len(fmt.format):
            out[fmt.name] = _columns_via_pymavlink(reader, fmt, offs)
            continue
        dtype = np.dtype([(c, _NP_DTYPES[ch]) for c, ch in zip(fmt.columns, fmt.format, strict=True)])
        if dtype.itemsize != fmt.len - 3:
            raise ValueError(f"{fmt.name}: FMT length {fmt.len} != fields {dtype.itemsize + 3}")
        body = np.arange(3, fmt.len, dtype=np.int64)
        step = max(1, _GATHER_BYTES // max(1, body.size))
        parts = [buf[offs[i : i + step, None] + body] for i in range(0, offs.size, step)]
        arr = np.ascontiguousarray(np.concatenate(parts)).view(dtype).reshape(-1)
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
        _, vals = log.series(mtype, field)
        if vals.size:
            return float(np.max(vals))
    return None
