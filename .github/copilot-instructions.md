# Copilot instructions

Full rules: see `AGENTS.md` at the repo root.

`ardupilot-mcp` is an offline, read-only MCP server that diagnoses ArduPilot DataFlash
`.bin` flight logs with a **deterministic check engine** (the LLM only explains the
grounded findings).

When suggesting code here:

- Keep the engine deterministic — never let a model decide whether something is a fault.
- Never add network access, vehicle actuation, or parameter writes (offline/read-only is
  the safety guarantee).
- Back every threshold/enum/default with an authoritative source (ArduPilot
  wiki/firmware, MAVLink, MCP spec); cite it in a comment and in `docs/SOURCES.md`.
- A new check = one module in `ardupilot_mcp/checks/` (`@register_check`, added to
  `_CHECK_MODULES`) + one test in `tests/checks/`. Update `README.md` and `CHANGELOG.md`.
- Run `ruff check`, `ruff format`, `mypy ardupilot_mcp`, and `pytest`.
