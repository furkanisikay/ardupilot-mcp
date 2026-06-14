<!-- Thanks for contributing! Keep PRs small and focused. See AGENTS.md / CONTRIBUTING.md. -->

## What & why

<!-- What does this change and why? Link any related issue (e.g. "Fixes #12"). -->

## Checklist

- [ ] `ruff check ardupilot_mcp tests` passes
- [ ] `ruff format ardupilot_mcp tests` applied
- [ ] `mypy ardupilot_mcp` passes
- [ ] `pytest` passes (added/updated tests for the change)
- [ ] New thresholds/enums/facts are backed by an authoritative source (comment + `docs/SOURCES.md`)
- [ ] Docs updated if a check/tool was added or renamed (`README.md`, `CHANGELOG.md`)
- [ ] No network access / vehicle actuation / parameter writes added (offline & read-only)
