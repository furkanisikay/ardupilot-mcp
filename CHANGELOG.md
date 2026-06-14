# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.2] — 2026-06-14

### Added
- One-click "Add to X" install buttons (Cursor, VS Code, VS Code Insiders, LM Studio) and
  per-client setup snippets for Claude Code, Claude Desktop, Cursor, VS Code, Antigravity
  (IDE + CLI), Windsurf, Cline, Continue, Zed, Goose and LM Studio in both READMEs, plus an
  example-scenarios section. All launch via `uvx ardupilot-mcp`.
- `server.json` — Model Context Protocol registry manifest (PyPI package, stdio transport).
- Claude Code plugin: `.claude-plugin/plugin.json`, `.claude-plugin/marketplace.json`, a
  root `.mcp.json`, and a `/diagnose` command that drives a full log diagnosis with the tools.
- Beginner onboarding in both READMEs: a plain-language opener (what it is, and that it
  plugs into an AI client), a 5-minute Quick start with one client, a "where to get a `.bin`
  log" section, a "try it with no AI app" path (clone + `examples/`), a sample diagnosis
  output, and a Troubleshooting section. `uv`/`uvx` is now explained and linked, and the
  pip-vs-uvx config choice is made explicit.
- A "why this" section comparing the tool, honestly, to Mission Planner Auto Analysis and
  the UAV Log Viewer — what each does, and when to use the others.

### Changed
- README reordered so install/usage comes before architecture/philosophy. Documentation
  only — no code or check behaviour changes.

## [0.1.1] — 2026-06-14

### Changed
- README is now Turkish (primary, `README.md`) with a plain link to the English
  version (`README.en.md`); removed audience/positioning meta-text from the README
  and package/repo descriptions. Documentation only — no code changes.

## [0.1.0] — 2026-06-14

First release: an offline, read-only MCP server that diagnoses ArduPilot DataFlash
`.bin` flight logs with a deterministic check engine; the LLM explains the grounded
findings.

### Added
- **Parser** (`pymavlink` DFReader) → pure `FlightLog` domain model; defensive against
  truncated/corrupt logs; flight-relative timestamps robust to corrupt outliers.
- **16 diagnostic checks** across two families:
  - Flight dynamics: events/errors, EKF health, vibration, power (BAT/CURR), GPS,
    compass, attitude tracking, motor balance, RC-link loss, timing.
  - Configuration & setup: risky parameters, parameter audit (contradictory/disabled/
    implausible values), calibration, configured-but-silent sensors, firmware pre-arm
    messages.
- **Vehicle awareness** (copter/heli/plane/rover) — checks gate to the vehicles they
  apply to.
- **Advisory tuning** — harmonic notch (gyro FFT), PID, autotune.
- **Documentation grounding** — every finding links authoritative ArduPilot docs;
  reports surface version-specific parameter definitions (`apm.pdef.xml`) for 4.x.
- **Physical profile** (`vehicle_profile`) — frame, motor count, battery cells, hover
  throttle → power margin and thrust-to-weight.
- **9 read-only MCP tools**: `analyze_log`, `log_summary`, `vehicle_profile`,
  `list_events`, `query_timeseries`, `get_params`, `load_param_file`, `recommend_tuning`,
  `list_checks`.
- **Parameter completeness** signals + `.param`/`.params` file ingestion.
- `examples/analyze.py` CLI and an MCP stdio smoke test.
- [`docs/SOURCES.md`](docs/SOURCES.md): every threshold/enum/fact audited against
  authoritative sources.

[Unreleased]: https://github.com/furkanisikay/ardupilot-mcp/commits/main
[0.1.2]: https://pypi.org/project/ardupilot-mcp/0.1.2/
[0.1.1]: https://pypi.org/project/ardupilot-mcp/0.1.1/
[0.1.0]: https://pypi.org/project/ardupilot-mcp/0.1.0/
