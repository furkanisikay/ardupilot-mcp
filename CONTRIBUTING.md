# Contributing to ardupilot-mcp

Thanks for helping make drone-log diagnosis better and more trustworthy! Contributions —
new checks, better thresholds (with sources), bug fixes, docs — are very welcome.

## The one principle to internalise

**The engine is deterministic; the LLM only explains.** Every finding must come from a
deterministic check backed by an authoritative source — never from a model's judgement.
This is what makes the tool trustworthy. See [`AGENTS.md`](AGENTS.md) for the full rules.

## Dev setup

```bash
git clone https://github.com/furkanisikay/ardupilot-mcp
cd ardupilot-mcp
python -m venv .venv && . .venv/bin/activate    # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
pytest                                            # should be all green
```

## The easiest, highest-value contribution: a new check

A check is **one module + one test** — the registry makes it a drop-in. Adding one is
the best way to make the tool catch more real problems. Run `/add-check` in Claude Code,
or follow the step-by-step in [`AGENTS.md`](AGENTS.md#how-to-add-a-check-the-main-contribution-path).

Have a log where the tool got it wrong (missed a fault, or cried wolf)? That's gold —
open an issue with the log (or the relevant message/param values) and what the real cause
was. It directly drives a new or fixed check.

## Before you open a PR

```bash
ruff check ardupilot_mcp tests
ruff format ardupilot_mcp tests
mypy ardupilot_mcp
pytest
```

All four must pass (CI runs them on every PR). Also:

- **Cite your sources.** New thresholds/enums/facts get a source comment AND an entry in
  [`docs/SOURCES.md`](docs/SOURCES.md). If a value is our own judgement, label it a design
  choice.
- **Keep docs in sync.** New/renamed check or tool → update `README.md` and `CHANGELOG.md`
  (`tests/test_docs_sync.py` enforces this).
- **Don't break the safety guarantee.** No network access, no vehicle actuation, no
  parameter writes — the server is offline and read-only by design.
- **Avoid false positives.** Validate against real logs where you can; a diagnostic that
  cries wolf on healthy flights is worse than useless.

## Commit / PR style

Small, focused PRs. Clear commit messages (what + why). Reference an issue if there is one.
By contributing you agree your work is licensed under the project's MIT license.
