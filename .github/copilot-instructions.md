# Copilot instructions

Full rules: see `AGENTS.md` at the repo root.

`ardupilot-mcp` is an offline, read-only MCP server that diagnoses ArduPilot flight logs
(`.bin` and `.tlog`) with a **deterministic check engine** (the LLM only explains the
grounded findings).

When suggesting code here:

- Keep the engine deterministic — never let a model decide whether something is a fault.
- Never add network access, vehicle actuation, or parameter writes (offline/read-only is
  the safety guarantee).
- Back every threshold/enum/default with an authoritative source (ArduPilot
  wiki/firmware, MAVLink, MCP spec); cite it in a comment and in `docs/SOURCES.md`.
- A new check = one module in `ardupilot_mcp/checks/` (`@register_check`, added to
  `_CHECK_MODULES`) + one test in `tests/checks/`. Run `python scripts/gen_docs.py` and
  add a `CHANGELOG.md` entry.
- Run `ruff check`, `ruff format`, `mypy ardupilot_mcp`, and `pytest`.
