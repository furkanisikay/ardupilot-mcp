"""Analyze an ArduPilot .bin log from the command line (no MCP client needed).

A thin convenience wrapper around the same engine the MCP server uses, handy for
quickly diagnosing a log or sanity-checking the tool on real data.

Run:  python examples/analyze.py path/to/flight.bin
"""

from __future__ import annotations

import os
import sys

# Findings contain Unicode (e.g. em-dashes); force UTF-8 stdout so the Windows
# console (cp1252) doesn't mangle them. The MCP server is unaffected (JSON/UTF-8).
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[union-attr]
except (AttributeError, ValueError):
    pass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ardupilot_mcp import diagnose, parse_bin  # noqa: E402
from ardupilot_mcp.tuning import analyze_tuning  # noqa: E402

_SEV_ORDER = {"critical": 0, "warn": 1, "info": 2}


def main(path: str) -> int:
    if not os.path.exists(path):
        print(f"file not found: {path}")
        return 2

    log = parse_bin(path)
    s = log.summary()
    print("=" * 72)
    print(f"  {s.vehicle_type or 'Vehicle'}  {s.firmware_version or ''}".rstrip())
    dur = f"{s.duration_s:.0f}s" if s.duration_s is not None else "unknown"
    print(f"  duration {dur}  |  {len(s.available_messages)} message types  |  "
          f"{sum(s.message_counts.values())} records  |  integrity: {s.integrity.value}")
    if s.flight_modes:
        print(f"  flight modes: {' -> '.join(s.flight_modes)}")
    print("=" * 72)

    report = diagnose(log)
    print(report.summary_text)
    print("-" * 72)
    print(f"FINDINGS  ({report.critical_count} critical, {report.warn_count} warning, "
          f"{report.info_count} info)")
    for f in report.findings:
        when = f"  @ {f.evidence.time_start_s:.0f}s" if f.evidence.time_start_s is not None else ""
        print(f"  [{f.severity.value.upper():8}] {f.check_id:10}{when}: {f.title}")
        print(f"             {f.explanation}")
        if f.recommendation:
            print(f"             -> {f.recommendation}")
    if report.checks_skipped:
        print(f"\n  skipped: {', '.join(f'{c.check_id} ({c.reason})' for c in report.checks_skipped)}")

    recs = analyze_tuning(log)
    if recs:
        print("-" * 72)
        print("TUNING (advisory only):")
        for r in recs:
            print(f"  ({r.area.value}, {r.confidence}) {r.title}")
            if r.suggested_params:
                print(f"        suggested: {r.suggested_params}")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: python examples/analyze.py path/to/flight.bin")
        raise SystemExit(2)
    raise SystemExit(main(sys.argv[1]))
