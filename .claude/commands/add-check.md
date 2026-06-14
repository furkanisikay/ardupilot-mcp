---
description: Scaffold a new ArduPilot log-diagnosis check the right way
---

Add a new diagnostic check to this repo following the rules in `AGENTS.md`. The user's
request: $ARGUMENTS

Do all of the following:

1. **Ground it first.** Identify the DataFlash message(s)/parameter(s) involved and the
   exact threshold/behaviour, and find an AUTHORITATIVE source for every number/enum
   (ArduPilot wiki/firmware `LogStructure.h`/`AP_Logger.h`, the parameter docs, or
   MAVLink). Do not invent thresholds. If you have real logs in `_reallogs/`, inspect a
   few to ground the values and find positive/negative cases.

2. **Create `ardupilot_mcp/checks/<name>.py`** with an `@register_check` class:
   `id`, `title`, `category`, `requires` (message types; or `set()` + guard internally),
   optional `vehicles` (e.g. `{"copter","heli"}`), `description`, and a pure deterministic
   `run(self, log) -> list[Finding]`. Put thresholds in named constants WITH a source
   comment. Cite the concrete numbers in each finding's `explanation`; numeric evidence in
   `samples=`. Be conservative to avoid false positives.

3. **Register it:** add `"<name>"` to `_CHECK_MODULES` in `ardupilot_mcp/checks/__init__.py`,
   and (optionally) map it in `CHECK_TOPICS` in `ardupilot_mcp/ardupilot_docs.py`.

4. **Write `tests/checks/test_<name>.py`** using `make_flight_log` / `build_series`
   (synthetic — no real `.bin`): the skipped/empty case, a clean no-finding case, and one
   case per severity it can emit.

5. **Update docs:** add the check to the catalogue in `README.md`, add a `CHANGELOG.md`
   entry, and record any new threshold + its source in `docs/SOURCES.md`.

6. **Verify green:** run `ruff check ardupilot_mcp tests`, `ruff format ardupilot_mcp tests`,
   `mypy ardupilot_mcp`, and `pytest` (including `tests/test_docs_sync.py`). If you have
   `_reallogs/`, confirm the new check flags the intended log and leaves clean flights clean.
