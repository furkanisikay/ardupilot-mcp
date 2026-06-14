# CLAUDE.md

The full, tool-agnostic rules for this repo live in **[AGENTS.md](AGENTS.md)** — read it
first. It covers the architecture, the golden rules (deterministic core / offline &
read-only / no unsupported assumptions / keep docs in sync / tests gate everything),
how to add a check, and the dev commands.

Claude-specific notes:

- Use `/add-check` (`.claude/commands/add-check.md`) to scaffold a new diagnostic check
  the right way.
- After any change, run `ruff check`, `ruff format`, `mypy ardupilot_mcp`, and `pytest`.
- When you add or rename a check/tool, update `README.md`, `CHANGELOG.md`, and
  `docs/SOURCES.md` — `tests/test_docs_sync.py` will fail if you don't.
- Never add network access or vehicle actuation: the server is offline and read-only by
  design, and that is its entire safety guarantee.
