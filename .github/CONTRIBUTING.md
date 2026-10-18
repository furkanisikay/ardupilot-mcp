# Contributing

Thank you for helping. The most useful contribution is a log where the tool got it wrong: a fault it missed, or a warning on a healthy flight. Open a [wrong diagnosis](https://github.com/furkanisikay/ardupilot-mcp/issues/new?template=wrong_or_missed_diagnosis.yml) issue with the log and what really happened.

The rules every change follows, including those for AI assistants, are in [AGENTS.md](../AGENTS.md). The main one: deterministic checks produce the findings, and each threshold has a source in [docs/SOURCES.md](../docs/SOURCES.md).

## Set up

```bash
git clone https://github.com/furkanisikay/ardupilot-mcp
cd ardupilot-mcp
python -m venv .venv && . .venv/bin/activate     # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
pre-commit install                                # optional: runs the checks on commit
pytest
```

## Make a change

- **New check:** one module in `ardupilot_mcp/checks/` and one test in `tests/checks/`. Follow [How to add a check](../AGENTS.md#how-to-add-a-check-the-main-contribution-path), or run `/add-check` in Claude Code.
- **New threshold or fact:** a source comment next to the constant and an entry in `docs/SOURCES.md`. Label a value you chose yourself as a design choice.
- **New or renamed tool:** a row in the tools table of both READMEs.
- **Generated files:** run `python scripts/gen_docs.py` after you change a tool, a check, the version or `bench/results.json`. It rewrites `docs/reference.md`, the plugin and registry manifests, and the parts of `README.md`, `README.en.md` and `bench/README.md` between `<!-- gen:NAME -->` markers (the example output, counts and benchmark table). Edit only the text outside the markers. A test fails while any of them is out of date.
- **License header:** every Python file outside `tests/` starts with `# SPDX-License-Identifier: AGPL-3.0-only`.

Run these before you push. CI runs the same commands:

```bash
ruff check ardupilot_mcp tests scripts bench examples
ruff format ardupilot_mcp tests scripts bench examples
mypy ardupilot_mcp
pytest
```

## Open a pull request

- Keep it to one change. Link the issue it fixes.
- Title it in [Conventional Commits](https://www.conventionalcommits.org/) form: `feat: ...`, `fix: ...`, `docs: ...`, `refactor: ...`, `test: ...`, `ci: ...`, `chore: ...`. The title becomes the squash commit and sets the release-notes label.
- Add a line to the `Unreleased` section of `CHANGELOG.md` when the change affects users. A PR that changes `ardupilot_mcp/` without one fails the changelog check unless a maintainer adds the `skip-changelog` label.
- If you have the benchmark logs, put the top-1, top-3 and false-critical change from `python -m bench.run --logs DIR` in the changelog entry.

## Contributor License Agreement

ardupilot-mcp is licensed under the AGPL-3.0 and also offered under a commercial license. To keep that possible, every contribution needs the [Contributor License Agreement](CLA.md). Tick the CLA box in the pull request template; the CLA check fails until it is ticked. You keep the copyright to your work.

## Conduct

Everyone in this project follows the [Code of Conduct](CODE_OF_CONDUCT.md).
