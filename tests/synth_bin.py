"""Synthetic ArduPilot DataFlash (.bin) writer for hermetic tests.

Produces a real, DFReader-parseable ``.bin`` so the parser and the end-to-end
``analyze_log(path)`` path can be tested without shipping large binary fixtures.

On-disk format (see pymavlink ``DFReader``): every message is
``[0xA3][0x95][type_id][body]`` where ``body`` is the little-endian struct of the
format's fields. A ``FMT`` message (type 0x80) declares each other format. We use
float fields (``f``) for physical quantities so stored values equal the values
DFReader returns (no multiplier bookkeeping) — checks see the same physical units
they would from a real log's multiplied fields (degrees, m/s^2, volts, ...).
"""

from __future__ import annotations

import struct

HEAD1 = 0xA3
HEAD2 = 0x95
FMT_TYPE = 0x80

# Subset of DataFlash format chars -> python struct chars we emit.
_FMT_CHAR_TO_STRUCT: dict[str, str] = {
    "b": "b",
    "B": "B",
    "h": "h",
    "H": "H",
    "i": "i",
    "I": "I",
    "f": "f",
    "d": "d",
    "n": "4s",
    "N": "16s",
    "Z": "64s",
    "Q": "Q",
    "q": "q",
}

# Synthetic format table: name -> (type_id, format_chars, columns).
# Column 0 is always TimeUS so DFReader establishes a microsecond clock.
FORMATS: dict[str, tuple[int, str, list[str]]] = {
    "PARM": (64, "QNf", ["TimeUS", "Name", "Value"]),
    "MSG": (65, "QZ", ["TimeUS", "Message"]),
    "MODE": (66, "QBBB", ["TimeUS", "Mode", "ModeNum", "Rsn"]),
    "ERR": (67, "QBB", ["TimeUS", "Subsys", "ECode"]),
    "EV": (68, "QB", ["TimeUS", "Id"]),
    "ATT": (69, "Qffffff", ["TimeUS", "DesRoll", "Roll", "DesPitch", "Pitch", "DesYaw", "Yaw"]),
    "VIBE": (70, "QfffIII", ["TimeUS", "VibeX", "VibeY", "VibeZ", "Clip0", "Clip1", "Clip2"]),
    "BAT": (71, "Qfff", ["TimeUS", "Volt", "Curr", "CurrTot"]),
    # GWk/GMS let pymavlink's DFReader establish a clean GPS time base (avoids
    # noisy "bad msg" clock warnings). build_bin auto-fills them; checks ignore them.
    "GPS": (72, "QBBffffHI", ["TimeUS", "Status", "NSats", "HDop", "Alt", "Spd", "Yaw", "GWk", "GMS"]),
    "MAG": (73, "Qfff", ["TimeUS", "MagX", "MagY", "MagZ"]),
    "RCOU": (74, "QHHHH", ["TimeUS", "C1", "C2", "C3", "C4"]),
    "IMU": (75, "Qffffff", ["TimeUS", "GyrX", "GyrY", "GyrZ", "AccX", "AccY", "AccZ"]),
    "POS": (76, "Qfff", ["TimeUS", "Lat", "Lng", "Alt"]),
    "XKF4": (77, "Qfffff", ["TimeUS", "SV", "SP", "SH", "SM", "SVT"]),
}


def _body_struct(fmt_chars: str) -> str:
    s = "<"
    for c in fmt_chars:
        if c not in _FMT_CHAR_TO_STRUCT:
            raise ValueError(f"unsupported synthetic format char {c!r}")
        s += _FMT_CHAR_TO_STRUCT[c]
    return s


def _body_len(fmt_chars: str) -> int:
    return struct.calcsize(_body_struct(fmt_chars))


def _pack_str(value: str, width: int) -> bytes:
    return value.encode("ascii", "replace")[:width].ljust(width, b"\x00")


def _pack_value(char: str, value: object) -> object:
    if char in ("n", "N", "Z"):
        width = {"n": 4, "N": 16, "Z": 64}[char]
        return _pack_str(str(value), width)
    if char in ("f", "d"):
        return float(value)  # type: ignore[arg-type]
    return int(value)  # type: ignore[arg-type]


def _pack_message(name: str, fields: dict[str, object]) -> bytes:
    tid, fmt_chars, cols = FORMATS[name]
    values = [_pack_value(fmt_chars[i], fields.get(cols[i], 0)) for i in range(len(cols))]
    body = struct.pack(_body_struct(fmt_chars), *values)
    return struct.pack("<BBB", HEAD1, HEAD2, tid) + body


def _fmt_message(name: str) -> bytes:
    """A FMT message (type 0x80) declaring ``name``'s format."""
    tid, fmt_chars, cols = FORMATS[name]
    length = 3 + _body_len(fmt_chars)
    fmt_body_struct = "<BB4s16s64s"
    body = struct.pack(
        fmt_body_struct,
        tid,
        length,
        _pack_str(name, 4),
        _pack_str(fmt_chars, 16),
        _pack_str(",".join(cols), 64),
    )
    return struct.pack("<BBB", HEAD1, HEAD2, FMT_TYPE) + body


def _fmt_defines_fmt() -> bytes:
    """The self-describing FMT-defines-FMT message that opens a real log."""
    body = struct.pack(
        "<BB4s16s64s",
        FMT_TYPE,
        89,
        _pack_str("FMT", 4),
        _pack_str("BBnNZ", 16),
        _pack_str("Type,Length,Name,Format,Columns", 64),
    )
    return struct.pack("<BBB", HEAD1, HEAD2, FMT_TYPE) + body


def build_bin(
    path: str,
    messages: list[tuple[str, dict[str, object]]],
    *,
    params: dict[str, float] | None = None,
    firmware: str | None = "ArduCopter V4.5.7 (synthetic)",
    truncate_bytes: int = 0,
) -> str:
    """Write a synthetic ``.bin`` log.

    ``messages`` is an ordered list of ``(format_name, field_values)``. ``params``
    become PARM messages emitted up front. ``firmware`` becomes an opening MSG.
    ``truncate_bytes`` > 0 chops that many bytes off the end to simulate a log
    that ends abruptly (as a crashed vehicle's log often does).
    """
    used = {name for name, _ in messages}
    used.update({"MSG", "PARM"})

    out = bytearray()
    out += _fmt_defines_fmt()
    for name in sorted(used):
        out += _fmt_message(name)

    t = 0
    if firmware:
        out += _pack_message("MSG", {"TimeUS": t, "Message": firmware})
        t += 1000
    for pname, pval in (params or {}).items():
        out += _pack_message("PARM", {"TimeUS": t, "Name": pname, "Value": float(pval)})
        t += 100

    for name, fields in messages:
        if name in ("GPS", "GPS2") and ("GWk" not in fields or "GMS" not in fields):
            fields = dict(fields)
            fields.setdefault("GWk", 2300)  # any plausible modern GPS week
            fields.setdefault("GMS", int(fields.get("TimeUS", 0)) // 1000)  # ms past week start
        out += _pack_message(name, fields)

    if truncate_bytes > 0:
        out = out[: max(0, len(out) - truncate_bytes)]

    with open(path, "wb") as fh:
        fh.write(bytes(out))
    return path
