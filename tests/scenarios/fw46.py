"""DataFlash writer that uses ArduPilot Copter-4.6.3's *exact* log formats.

Formats (name, format chars, columns, unit chars) are copied verbatim from the
firmware's LogStructure.h files, including the ``#`` instance unit in FMTU, so
pymavlink parses these files exactly as it parses a real 4.6 log: multi-instance
VIBE/GPS/MAG/BAT/IMU/XKF4 interleaved under one message name, a single VIBE.Clip
counter per IMU, RCI2.Flags, PM.NL, scaled ``c``/``e``/``L`` fields, and so on.

Sources (Copter-4.6.3): libraries/AP_Logger/LogStructure.h, AP_InertialSensor/,
AP_GPS/, AP_BattMonitor/, AP_NavEKF3/, AP_AHRS/ LogStructure.h, ArduCopter/Log.cpp.
"""

from __future__ import annotations

import struct

HEAD = b"\xa3\x95"

# name: (type_id, format, columns, units)
FORMATS: dict[str, tuple[int, str, str, str]] = {
    "FMTU": (177, "QBNN", "TimeUS,FmtType,UnitIds,MultIds", "s---"),
    "PARM": (64, "QNff", "TimeUS,Name,Value,Default", "s---"),
    "MSG": (65, "QZ", "TimeUS,Message", "s-"),
    "MODE": (66, "QMBB", "TimeUS,Mode,ModeNum,Rsn", "s---"),
    "EV": (67, "QB", "TimeUS,Id", "s-"),
    "ERR": (68, "QBB", "TimeUS,Subsys,ECode", "s--"),
    "ATT": (69, "QffffffB", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,AEKF", "sddddhh-"),
    "VIBE": (70, "QBfffI", "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip", "s#ooo-"),
    "IMU": (
        71,
        "QBffffffIIfBBHH",
        "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz",
        "s#EEEooo--O--zz",
    ),
    "GPS": (
        72,
        "QBBIHBcLLeffffB",
        "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U",
        "s#-s-S-DUmnhnh-",
    ),
    "MAG": (
        73,
        "QBhhhhhhhhhBI",
        "TimeUS,I,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOX,MOY,MOZ,Health,S",
        "s#GGGGGGGGG-s",
    ),
    "BAT": (
        74,
        "QBfffffcfBBB",
        "TimeUS,Inst,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,RemPct,H,SH",
        "s#vvAaXOw%-%",
    ),
    "XKF4": (
        75,
        "QBcccccfffHBIHb",
        "TimeUS,C,SV,SP,SH,SM,SVT,errRP,OFN,OFE,FS,TS,SS,GPS,PI",
        "s#------mm-----",
    ),
    "RCIN": (76, "QHHHHHHHHHHHHHH", "TimeUS," + ",".join(f"C{i}" for i in range(1, 15)), "s" + "Y" * 14),
    "RCOU": (77, "QHHHHHHHHHHHHHH", "TimeUS," + ",".join(f"C{i}" for i in range(1, 15)), "s" + "Y" * 14),
    "POWR": (78, "QffHHB", "TimeUS,Vcc,VServo,Flags,AccFlags,Safety", "svv---"),
    "PM": (
        79,
        "QHHHIIHHIIIIIIQ",
        "TimeUS,LR,NLon,NL,MaxT,Mem,Load,ErrL,InE,ErC,SPIC,I2CC,I2CI,Ex,R",
        "sz---b%------ss",
    ),
    "CTUN": (
        80,
        "Qffffffefffhh",
        "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt",
        "s----mmmmmmnn",
    ),
    "POS": (81, "QLLfff", "TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt", "sDUmmm"),
    "RCI2": (82, "QHHHB", "TimeUS,C15,C16,OMask,Flags", "sYY--"),
}

_STRUCT = {
    "b": "b", "B": "B", "M": "b", "h": "h", "H": "H", "i": "i", "I": "I", "f": "f", "d": "d",
    "n": "4s", "N": "16s", "Z": "64s", "Q": "Q", "q": "q", "c": "h", "C": "H", "e": "i", "E": "I", "L": "i",
}  # fmt: skip
# pymavlink divides these by 100 (c, C, e, E) or 1e7 (L) on read.
_SCALE = {"c": 100.0, "C": 100.0, "e": 100.0, "E": 100.0, "L": 1e7}
_STR_WIDTH = {"n": 4, "N": 16, "Z": 64}


def _pstr(v: str, width: int) -> bytes:
    return v.encode("ascii", "replace")[:width].ljust(width, b"\x00")


def _fmt_msg(type_id: int, length: int, name: str, fmt: str, cols: str) -> bytes:
    body = struct.pack("<BB4s16s64s", type_id, length, _pstr(name, 4), _pstr(fmt, 16), _pstr(cols, 64))
    return HEAD + bytes([0x80]) + body


class Writer:
    """Append-only DataFlash writer: FMT + FMTU for every format, then records."""

    def __init__(self, path: str) -> None:
        self.fh = open(path, "wb")  # noqa: SIM115 - closed explicitly by close()
        self.fh.write(_fmt_msg(0x80, 89, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns"))
        self._structs: dict[str, tuple[struct.Struct, list[str], str]] = {}
        for name, (tid, fmt, cols, _units) in FORMATS.items():
            st = struct.Struct("<" + "".join(_STRUCT[c] for c in fmt))
            self._structs[name] = (st, cols.split(","), fmt)
            self.fh.write(_fmt_msg(tid, 3 + st.size, name, fmt, cols))
        for _name, (tid, _fmt, _cols, units) in FORMATS.items():
            self.write("FMTU", TimeUS=0, FmtType=tid, UnitIds=units, MultIds="")

    def write(self, name: str, **fields: object) -> None:
        st, cols, fmt = self._structs[name]
        vals: list[object] = []
        for c, col in zip(fmt, cols, strict=True):
            v = fields.get(col, 0)
            if c in _STR_WIDTH:
                vals.append(_pstr(str(v), _STR_WIDTH[c]))
            elif c in ("f", "d"):
                vals.append(float(v))  # type: ignore[arg-type]
            elif c in _SCALE:
                vals.append(int(round(float(v) * _SCALE[c])))  # type: ignore[arg-type]
            else:
                vals.append(int(v))  # type: ignore[call-overload]
        self.fh.write(HEAD + bytes([FORMATS[name][0]]) + st.pack(*vals))

    def close(self) -> None:
        self.fh.close()
