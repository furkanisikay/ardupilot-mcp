# SPDX-License-Identifier: AGPL-3.0-only
"""Run the real-crash benchmark against locally downloaded logs.

    python -m bench.run --logs DIR [--json OUT]

DIR holds the logs named ``<id>.bin`` (the ``id`` field of bench/real_crashes.json).
Nothing is downloaded: fetch each entry's ``source_url`` yourself. A file whose
sha256 differs from the manifest is reported and skipped, so results always refer
to the labelled flight. Missing files are skipped.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import os
import sys
from dataclasses import asdict

from ardupilot_mcp.orchestrator import diagnose
from ardupilot_mcp.parser import parse_bin

from .scoring import score_report, summarize

MANIFEST = os.path.join(os.path.dirname(__file__), "real_crashes.json")


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--logs", required=True, help="directory with <id>.bin files")
    ap.add_argument("--json", help="write per-log scores and the summary here")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.logs):
        ap.error(f"--logs {args.logs!r} is not a directory")

    with open(MANIFEST, encoding="utf-8") as fh:
        entries = json.load(fh)["entries"]
    scores, skipped = [], []
    for e in entries:
        path = os.path.join(args.logs, f"{e['id']}.bin")
        if not os.path.exists(path):
            skipped.append((e["id"], "missing"))
            continue
        if _sha256(path) != e["sha256"]:
            skipped.append((e["id"], "sha256 mismatch"))
            continue
        try:
            with contextlib.redirect_stdout(io.StringIO()):  # pymavlink prints on corrupt bytes
                report = diagnose(parse_bin(path))
        except Exception as exc:  # noqa: BLE001 - one unreadable log must not abort the run
            skipped.append((e["id"], f"could not analyse: {exc}"))
            continue
        s = score_report(e, report)
        scores.append(s)
        mark = {True: "HIT ", False: "miss", None: "  - "}[s.hit]
        fc = f" false-critical: {', '.join(s.false_criticals)}" if s.false_criticals else ""
        print(f"{mark} {s.id:<12} {s.category:<22} top={s.top_check} onset={s.onset_check}{fc}")
    summary = summarize(scores)
    print(json.dumps(summary, indent=1))
    for sid, why in skipped:
        print(f"skipped {sid}: {why}", file=sys.stderr)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"summary": summary, "scores": [asdict(s) for s in scores]}, fh, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
