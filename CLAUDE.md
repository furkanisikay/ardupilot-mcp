# CLAUDE.md

The full, tool-agnostic rules for this repo live in **[AGENTS.md](AGENTS.md)** — read it
first. It covers the architecture, the golden rules (deterministic core / offline &
read-only / no unsupported assumptions / keep docs in sync / tests gate everything),
how to add a check, and the dev commands.

Claude-specific notes:

- Use `/add-check` (`.claude/commands/add-check.md`) to scaffold a new diagnostic check
  the right way.
- After any change, run `ruff check`, `ruff format`, `mypy ardupilot_mcp`, and `pytest`.
- After changing a tool, a check or the version, run `python scripts/gen_docs.py`. A new
  or renamed tool also needs a row in both READMEs; user-visible changes need a
  `CHANGELOG.md` entry and new thresholds a `docs/SOURCES.md` entry.
- Never add network access or vehicle actuation: the server is offline and read-only by
  design, and that is its entire safety guarantee.
