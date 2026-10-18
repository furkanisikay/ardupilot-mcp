# SPDX-License-Identifier: AGPL-3.0-only
"""Generate the bundled parameter metadata in ``ardupilot_mcp/data/params/``.

Usage (offline, against apm.pdef.xml files you downloaded from ArduPilot's versioned
parameter archive, https://autotest.ardupilot.org/Parameters/versioned/)::

    python scripts/gen_param_meta.py Copter 4.6.3 /path/to/apm.pdef.xml

It writes ``ardupilot_mcp/data/params/Copter-4.6.3.json.gz``. Only facts are kept:
the parameter name, its documented range, units, the codes of its listed values,
the bit positions of a bitmask, and the ReadOnly / RebootRequired flags. No
description, display name or value label is copied.
"""

from __future__ import annotations

import gzip
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "ardupilot_mcp" / "data" / "params"
ARCHIVE = "https://autotest.ardupilot.org/Parameters/versioned/{vehicle}/stable-{version}/apm.pdef.xml"


def _num(text: str) -> float | int:
    v = float(text)
    return int(v) if v.is_integer() else v


def extract(xml_path: Path) -> dict[str, dict]:
    params: dict[str, dict] = {}
    for p in ET.parse(xml_path).getroot().iter("param"):
        name = p.get("name", "").split(":")[-1]
        if not name:
            continue
        d: dict = {}
        for f in p.findall("field"):
            key, text = f.get("name"), (f.text or "").strip()
            try:
                if key == "Range":
                    lo, hi = text.split()
                    d["r"] = [_num(lo), _num(hi)]
                elif key == "Units":
                    d["u"] = text
                elif key == "Bitmask":
                    d["b"] = sorted({int(item.split(":")[0]) for item in text.split(",") if ":" in item})
                elif key == "ReadOnly" and text.lower() == "true":
                    d["ro"] = 1
                elif key == "RebootRequired" and text.lower() == "true":
                    d["rb"] = 1
            except ValueError:
                continue  # a malformed field is left out rather than guessed
        values = p.find("values")
        if values is not None:
            codes = []
            for v in values.findall("value"):
                try:
                    codes.append(_num(v.get("code", "")))
                except ValueError:
                    continue
            if codes:
                d["v"] = sorted(set(codes))
        params[name] = _merge(params[name], d) if name in params else d
    return params


def _merge(a: dict, b: dict) -> dict:
    """One name documented twice, e.g. Copter's multirotor and heli attitude controllers
    (ATC_RAT_*, AUTOTUNE_AXES) or two battery backends (BATT*_VOLT_PIN in 4.7).

    Design choice: the file is per vehicle, not per frame or backend, so keep what either
    definition documents: the widest range and every listed value code and bit.
    """
    out: dict = {}
    if "r" in a and "r" in b:
        out["r"] = [min(a["r"][0], b["r"][0]), max(a["r"][1], b["r"][1])]
    elif "r" in a or "r" in b:
        out["r"] = a.get("r") or b.get("r")
    if a.get("u") == b.get("u") and "u" in a:
        out["u"] = a["u"]
    for key in ("v", "b"):
        if key in a or key in b:
            out[key] = sorted(set(a.get(key, [])) | set(b.get(key, [])))
    for key in ("ro", "rb"):
        if a.get(key) or b.get(key):
            out[key] = 1
    return out


def main(argv: list[str]) -> int:
    if len(argv) != 4:
        print(__doc__, file=sys.stderr)
        return 2
    vehicle, version, xml_path = argv[1], argv[2], Path(argv[3])
    doc = {
        "vehicle": vehicle,
        "version": version,
        "source": ARCHIVE.format(vehicle=vehicle, version=version),
        "params": extract(xml_path),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    raw = json.dumps(doc, sort_keys=True, separators=(",", ":")).encode()
    out = OUT / f"{vehicle}-{version}.json.gz"
    with open(out, "wb") as fh, gzip.GzipFile(fileobj=fh, mode="wb", mtime=0, filename="") as gz:
        gz.write(raw)
    print(f"{out.name}: {len(doc['params'])} params, {out.stat().st_size} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
