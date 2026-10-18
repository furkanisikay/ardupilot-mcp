"""Generate ``ardupilot_mcp/motor_geometry.py`` from ArduPilot's motor matrix source.

Usage (offline, against a local ArduPilot checkout at the tag you want to mirror)::

    python scripts/gen_motor_geometry.py /path/to/ardupilot Copter-4.6.3 > ardupilot_mcp/motor_geometry.py

It reads ``libraries/AP_Motors/AP_MotorsMatrix.cpp`` and records, for every frame
class/type pair, each motor's roll and pitch factor exactly as the firmware computes
them (``add_motor`` uses cos(roll_deg + 90) and cos(pitch_deg)). Motor *n* in the
table is index n-1, the same index "Potential Thrust Loss (n)" reports as n.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

CLASSES = {
    "quad": 1,
    "hexa": 2,
    "octa": 3,
    "octaquad": 4,
    "y6": 5,
    "dodecahexa": 12,
    "deca": 14,
}
NUM = r"-?\d+(?:\.\d+)?f?"
YAW = r"(?:AP_MOTORS_MATRIX_YAW_FACTOR_\w+|-?\d+(?:\.\d+)?f?)"
DEF = re.compile(rf"\{{\s*({NUM})\s*,\s*{YAW}\s*,\s*\d+\s*\}}")
RAW = re.compile(rf"\{{\s*({NUM})\s*,\s*({NUM})\s*,\s*{YAW}\s*,\s*\d+\s*\}}")
ADD4 = re.compile(rf"add_motor\(\s*AP_MOTORS_MOT_(\d+)\s*,\s*({NUM})\s*,\s*{YAW}\s*,\s*\d+\s*\)")
ADD5 = re.compile(
    rf"add_motor\(\s*AP_MOTORS_MOT_(\d+)\s*,\s*({NUM})\s*,\s*({NUM})\s*,\s*{YAW}\s*,\s*\d+\s*\)"
)
CASE = re.compile(r"case\s+MOTOR_FRAME_TYPE_(\w+)\s*:|default\s*:")
DEFAULT = "default"  # the frame class's own fallback layout, used for any unlisted FRAME_TYPE


def _f(s: str) -> float:
    return float(s.rstrip("f"))


def _from_angles(roll_deg: float, pitch_deg: float) -> tuple[float, float]:
    return (
        round(math.cos(math.radians(roll_deg + 90)), 4) + 0.0,
        round(math.cos(math.radians(pitch_deg)), 4) + 0.0,
    )


def _motors(block: str) -> list[tuple[float, float]]:
    raw = RAW.findall(block)
    if raw:
        return [(_f(r), _f(p)) for r, p in raw]
    defs = DEF.findall(block)
    if defs:
        return [_from_angles(_f(a), _f(a)) for a in defs]
    adds: dict[int, tuple[float, float]] = {}
    for n, r, p in ADD5.findall(block):
        adds[int(n)] = _from_angles(_f(r), _f(p))
    for n, a in ADD4.findall(block):
        adds.setdefault(int(n), _from_angles(_f(a), _f(a)))
    return [adds[k] for k in sorted(adds)] if adds else []


def parse(src: str) -> dict[tuple[int, str], list[tuple[float, float]]]:
    out: dict[tuple[int, str], list[tuple[float, float]]] = {}
    for name, fclass in CLASSES.items():
        m = re.search(rf"bool AP_MotorsMatrix::setup_{name}_matrix\(.*?\n\}}\n", src, re.S)
        if not m:
            continue
        body = m.group(0)
        cases = list(CASE.finditer(body))
        pending: list[str] = []
        for i, c in enumerate(cases):
            end = cases[i + 1].start() if i + 1 < len(cases) else len(body)
            motors = _motors(body[c.end() : end])
            pending.append(c.group(1) or DEFAULT)
            if not motors:
                continue  # falls through to the next case label
            for t in pending:
                out[(fclass, t)] = motors
            pending = []
    return out


def main() -> None:
    root, tag = Path(sys.argv[1]), sys.argv[2]
    src = (root / "libraries/AP_Motors/AP_MotorsMatrix.cpp").read_text()
    hdr = (root / "libraries/AP_Motors/AP_Motors_Class.h").read_text()
    types = {n: int(v) for n, v in re.findall(r"MOTOR_FRAME_TYPE_(\w+)\s*=\s*(\d+)", hdr)}
    table = parse(src)
    print(f'"""Motor roll/pitch factors per frame, generated from ArduPilot {tag}.')
    print()
    print("Source: libraries/AP_Motors/AP_MotorsMatrix.cpp (setup_*_matrix). Do not edit by hand:")
    print("regenerate with scripts/gen_motor_geometry.py. Key: (FRAME_CLASS, FRAME_TYPE); value:")
    print("one (roll_factor, pitch_factor) per motor, motor 1 first.")
    print('"""')
    print()
    print("from __future__ import annotations")
    print()
    print(f'SOURCE_TAG = "{tag}"')
    print()

    def row(motors: list[tuple[float, float]]) -> str:
        return "(" + ", ".join(f"({r}, {p})" for r, p in motors) + ",)"

    ordered = sorted(table.items(), key=lambda kv: (kv[0][0], types.get(kv[0][1], 99)))
    print("FRAME_GEOMETRY: dict[tuple[int, int], tuple[tuple[float, float], ...]] = {")
    for (fclass, tname), motors in ordered:
        if tname in types:
            print(f"    ({fclass}, {types[tname]}): {row(motors)},  # {tname}")
    print("}")
    print()
    print("# Layout a frame class uses for any FRAME_TYPE its switch does not list (`default:`).")
    print("FRAME_DEFAULT: dict[int, tuple[tuple[float, float], ...]] = {")
    for (fclass, tname), motors in ordered:
        if tname == DEFAULT:
            print(f"    {fclass}: {row(motors)},")
    print("}")


if __name__ == "__main__":
    main()
