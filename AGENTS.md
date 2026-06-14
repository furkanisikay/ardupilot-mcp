# AGENTS.md — rules for AI agents & contributors

This file is the contract for anyone (human or AI: Claude Code, Cursor, Copilot,
Codex, …) working in this repo. Read it before changing anything. `CLAUDE.md`,
`.cursor/rules/` and `.github/copilot-instructions.md` all point here.

## What this is

An **MCP server that diagnoses ArduPilot DataFlash (`.bin`) flight logs**. You give
it a log; an LLM client (Claude Desktop, …) calls our tools and explains, in plain
language, what went wrong — grounded in a deterministic check engine, not guesses.

## Golden rules (do not break these)

1. **Deterministic core, LLM explains.** Every finding is produced by a deterministic
   check (thresholds, math, decoded events) — never by an LLM. The LLM only narrates
   and cites. Do not add "the model decides if it's bad" logic to the server.
2. **Offline & read-only by construction.** The server parses local files only. It must
   never open a network connection, arm/fly a vehicle, or write parameters. This is the
   whole safety story — do not add actuation or network I/O to the package.
3. **No unsupported assumptions.** Every numeric threshold, enum, default and factual
   claim must be backed by an authoritative source (ArduPilot wiki/firmware, MAVLink,
   the MCP spec) and recorded in [`docs/SOURCES.md`](docs/SOURCES.md). Put the source
   URL in a comment next to the constant. If it's a judgement call we chose, label it a
   *design choice*, not a fact.
4. **Keep docs in sync.** If you add/rename a check or an MCP tool you MUST update
   `README.md` and `CHANGELOG.md` (and `docs/SOURCES.md` for any new threshold). A test
   (`tests/test_docs_sync.py`) enforces this — it fails if a check/tool isn't documented.
5. **Tests gate everything.** `ruff`, `mypy` and `pytest` must all pass. New behaviour
   needs tests. CI runs these on every PR and protects `main`.

## Architecture (where things live)

```
ardupilot_mcp/
  parser.py        DataFlash .bin -> FlightLog   (the ONLY pymavlink-dependent module)
  flight_log.py    FlightLog domain model (pure; checks/tests use this, no real .bin needed)
  model.py         Pydantic models (Finding, DiagnosisReport, VehicleProfile, ...)
  checks/          one module per check; @register_check auto-registers it
    base.py        Check ABC + registry (id, requires, vehicles, run())
    util.py        shared numeric helpers (intervals_above, make_finding, ...)
  orchestrator.py  runs all checks -> DiagnosisReport (+ attaches doc references)
  tuning.py        advisory tuning (notch FFT, PID, autotune) — not a check
  profile.py       VehicleProfile (frame, motors, battery cells, power margin)
  param_file.py    parse a user-exported .param / .params file
  ardupilot_meta.py  ArduPilot reference data (modes, ERR/EV enums, frame classes)
  ardupilot_docs.py  curated, verified ArduPilot doc URLs attached to findings
  server.py        FastMCP server + the read-only MCP tools
  __main__.py      `python -m ardupilot_mcp` (stdio)
tests/             pytest; synthetic logs via tests/helpers.py + tests/synth_bin.py
docs/SOURCES.md    every assumption, audited + cited
```

## How to add a check (the main contribution path)

A check is one module + one test. The registry makes it a drop-in.

1. Create `ardupilot_mcp/checks/<name>.py`:
   ```python
   from ..flight_log import FlightLog
   from ..model import Finding, Severity
   from .base import Check, register_check
   from .util import make_finding

   THRESHOLD = 60.0  # source: <authoritative URL for this value>

   @register_check
   class MyCheck(Check):
       id = "my_check"
       title = "My check"
       category = "vibration"          # grouping
       requires = {"VIBE"}             # message types needed (auto-skip if absent)
       vehicles = None                 # or {"copter", "heli"} to gate by vehicle kind
       description = "One line for list_checks."

       def run(self, log: FlightLog) -> list[Finding]:
           ...                         # deterministic; cite numbers in the explanation
   ```
2. Add `"my_check"` to `_CHECK_MODULES` in `ardupilot_mcp/checks/__init__.py`.
3. (Optional) map it to doc references in `CHECK_TOPICS` in `ardupilot_mcp/ardupilot_docs.py`.
4. Write `tests/checks/test_my_check.py`: the skip/empty case, a clean (no-finding)
   case, and one case per severity. Test against synthetic logs (`make_flight_log` /
   `build_series`) — no real `.bin` needed.
5. Update `README.md` (check catalogue) + `CHANGELOG.md`; add any new threshold to
   `docs/SOURCES.md` with its source.
6. Run the commands below until green.

## Commands

```bash
pip install -e ".[dev]"
ruff check ardupilot_mcp tests        # lint
ruff format ardupilot_mcp tests       # format
mypy ardupilot_mcp                    # types
pytest                                # tests
python -m ardupilot_mcp               # run the server (stdio)
python examples/analyze.py log.bin    # diagnose a log from the CLI
```

## Conventions

- Thresholds/facts in **named module-level constants**, each with a source comment.
- Finding `explanation` must cite the concrete numbers (peaks, counts, thresholds);
  numeric evidence goes in `samples=`.
- Be conservative — **avoid false positives**. Prefer skipping to crying wolf. Validate
  new checks against `_reallogs/` (gitignored real logs) so clean flights stay clean.
- `samples` values are numbers, not strings (the Pydantic model enforces it).
- Python ≥ 3.11, full type hints, `from __future__ import annotations`.
