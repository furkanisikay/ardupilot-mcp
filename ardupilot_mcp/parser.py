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

from . import ardupilot_meta as meta
from .flight_log import FlightLog, LogMeta
from .model import LogIntegrity

# Pure structural/metadata messages we don't keep as data rows.
_SKIP_TYPES = {"FMT", "FMTU", "UNIT", "MULT"}


def parse_bin(path: str) -> FlightLog:
    if not os.path.exists(path):
        raise FileNotFoundError(f"log file not found: {path}")

    from pymavlink import DFReader  # local import keeps the dependency at the edge

    messages: dict[str, list[dict]] = {}
    integrity = LogIntegrity.OK
    integrity_detail: str | None = None
    parsed = 0

    # DFReader writes corruption warnings to stderr (it does not raise) during both
    # the initial index pass and message reads. Capture them so we can flag a
    # truncated/corrupt log instead of leaking noise to the console.
    capture = io.StringIO()
    with contextlib.redirect_stderr(capture):
        try:
            reader = DFReader.DFReader_binary(path)
        except Exception as exc:  # noqa: BLE001
            raise ValueError(f"could not open DataFlash log {path!r}: {exc}") from exc

        while True:
            try:
                m = reader.recv_match()
            except Exception as exc:  # noqa: BLE001 - truncated/corrupt tail
                integrity = LogIntegrity.TRUNCATED
                integrity_detail = f"parse stopped after {parsed} messages: {type(exc).__name__}: {exc}"
                break
            if m is None:
                break
            mtype = m.get_type()
            if mtype in _SKIP_TYPES:
                continue
            try:
                d = m.to_dict()  # already a fresh dict per call; no need to copy
            except Exception:  # noqa: BLE001
                continue
            d.pop("mavpackettype", None)
            # Only TimeUS/TimeMS drive timestamps (handled in FlightLog). We do NOT
            # fall back to DFReader's _timestamp: for a GPS-clocked log that is a Unix
            # epoch (~1.7e9 s), and records without TimeUS (e.g. FILE) would otherwise
            # poison the time base and produce absurd durations.
            messages.setdefault(mtype, []).append(d)
            parsed += 1

    integrity, integrity_detail = _assess_integrity(capture.getvalue(), integrity, integrity_detail)
    integrity, integrity_detail = _assess_tail(path, integrity, integrity_detail)

    params = _extract_params(reader)
    fw, vehicle = _extract_firmware(messages)
    modes = _extract_modes(messages)
    max_alt = _extract_max_alt(messages)

    kind = meta.vehicle_kind(vehicle, params.get("FRAME_CLASS"))
    log_meta = LogMeta(
        vehicle_type=vehicle,
        vehicle_kind=kind,
        firmware_version=fw,
        board=None,
        integrity=integrity,
        integrity_detail=integrity_detail,
        flight_modes=modes,
        max_altitude_m=max_alt,
    )
    return FlightLog(messages=messages, params=params, meta=log_meta, path=path, _copy=False)


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


def _extract_params(reader) -> dict[str, float]:
    raw = getattr(reader, "params", None) or {}
    out: dict[str, float] = {}
    for k, v in raw.items():
        try:
            out[str(k)] = float(v)
        except (TypeError, ValueError):
            continue
    return out


def _extract_firmware(messages: dict[str, list[dict]]) -> tuple[str | None, str | None]:
    fw: str | None = None
    vehicle: str | None = None
    # Newer logs: VER message with a firmware string field.
    for rec in messages.get("VER", []):
        for key in ("FwString", "FWString", "fw_string"):
            if key in rec and isinstance(rec[key], str):
                fw = rec[key]
                vehicle = meta.vehicle_from_message(fw) or vehicle
                break
    # Classic: MSG text lines, the first matching a known vehicle prefix.
    for rec in messages.get("MSG", []):
        text = rec.get("Message") or rec.get("Msg")
        if isinstance(text, str):
            v = meta.vehicle_from_message(text)
            if v:
                vehicle = vehicle or v
                fw = fw or text.strip()
                break
    return fw, vehicle


def _extract_modes(messages: dict[str, list[dict]]) -> list[str]:
    seen: list[str] = []
    for rec in messages.get("MODE", []):
        num = rec.get("Mode", rec.get("ModeNum"))
        if num is None:
            continue
        name = meta.mode_name(int(num))
        if name not in seen:
            seen.append(name)
    return seen


def _extract_max_alt(messages: dict[str, list[dict]]) -> float | None:
    best: float | None = None
    for mtype, field in (("POS", "Alt"), ("GPS", "Alt"), ("CTUN", "Alt"), ("BARO", "Alt")):
        for rec in messages.get(mtype, []):
            v = rec.get(field)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                best = float(v) if best is None else max(best, float(v))
    return best
