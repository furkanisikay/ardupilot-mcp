"""Parse a user-exported ArduPilot parameter file.

The DataFlash log already carries a near-complete PARM dump, but it is a snapshot
that a truncated/crashed log can cut short. A separate ``.param`` (Mission
Planner) or ``.params`` (QGroundControl) file is the authoritative complete
config, and lets the user compare "as-flown" against "current". Reading a local
text file keeps the server offline/safe.

Handles both common formats:
  Mission Planner ``.param``:  ``WPNAV_SPEED,500``  (comma or whitespace separated)
  QGroundControl ``.params``:  ``1\t1\tWPNAV_SPEED\t500.000000\t9``  (sysid compid NAME VALUE TYPE)
Lines starting with ``#`` or ``//`` are comments.
"""

from __future__ import annotations

import re


def parse_param_file(path: str) -> dict[str, float]:
    out: dict[str, float] = {}
    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith("//"):
                continue
            parts = re.split(r"[,\s]+", line)
            if len(parts) >= 5 and parts[0].lstrip("-").isdigit() and parts[1].lstrip("-").isdigit():
                # QGC: sysid compid NAME VALUE TYPE
                name, value = parts[2], parts[3]
            elif len(parts) >= 2:
                # Mission Planner: NAME VALUE  (ignore any trailing tokens)
                name, value = parts[0], parts[1]
            else:
                continue
            try:
                out[name.upper()] = float(value)
            except ValueError:
                continue
    return out
