# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- **Multi-instance sensors were interleaved.** Firmware 4.x logs every IMU, GPS, compass,
  battery and EKF core under one message name (told apart by an instance field). Checks now
  judge each instance on its own samples, using the log's own FMTU `#` declarations. This
  removes false alarms such as a CRITICAL "GPS lost 3D fix" caused by a second receiver,
  a CRITICAL "battery critical voltage" caused by a 2S payload pack on the second monitor,
  and "unstable compass" caused by two healthy compasses with different field strengths.
- **Accelerometer clipping was never detected on 4.x logs**: modern `VIBE` has one `Clip`
  counter per IMU (the check only read the pre-4.0 `Clip0/1/2`). Counter resets are handled.
- `timing`: "long loops" warned whenever `PM.NLon > 0`, i.e. on almost every healthy log. It
  now judges the long-loop percentage per PM window (`NLon/NL`), as ArduPilot's LogAnalyzer
  did, and cites `PM.Ex`.
- Flight modes were decoded with the Copter table for every vehicle (a Plane in MANUAL was
  reported as STABILIZE). Plane, Rover and Sub have their own tables; Copter gains TURTLE.
- `max_altitude_m` mixed AMSL (`POS/GPS.Alt`) with relative altitude; it is now the maximum
  altitude above home (`POS.RelHomeAlt`, else `CTUN.Alt`/`BARO.Alt`).
- `rcin`: RC loss is judged only while armed (a receiver with no transmitter on the bench is
  no longer a CRITICAL), and uses `RCI2.Flags` (no valid input / RC failsafe) when logged —
  most receivers hold the last values on link loss, which the PWM heuristic cannot see.
- `motors`: motor outputs come from `SERVOn_FUNCTION` (or the frame's motor count), so a
  gimbal servo is no longer a "motor imbalance"; only armed time is averaged, so a real
  imbalance is no longer diluted by disarmed samples.
- `query_timeseries`: when a field was missing from some records, the time window was
  silently not applied and values no longer lined up with `times_s`. Values are now aligned
  by record (`null` where a record lacks the field) and unknown fields are reported.
- `sensors` recognises `GPS1_TYPE` (4.6 renamed `GPS_TYPE`); compass health is per compass.
- NaN/inf samples no longer poison max/mean comparisons (they are dropped from series).

### Added
- `power`: flight-controller board voltage (`POWR.Vcc`) — CRITICAL below 4.6 V while armed,
  WARN on more than 0.3 V of ripple. Per-battery thresholds (`BATT2_*` for the second pack).
- `compass`: throttle-correlated interference estimate (% field change at full throttle, the
  quantity CompassMot shows), graded with ArduPilot's 30 % / 60 % bands.
- `ekf`: per-core evaluation; full severity only while the core was primary (`XKF4.PI`),
  standby-core strain is INFO, and an EKF lane switch is reported.
- `integrity`: "Recording ended while armed" — the log stops with no disarm, the signature of
  an in-flight power loss or crash.
- `query_timeseries(..., instance=)` and `available_instances` for multi-instance messages.
- `FlightLog.instances()`, `series(..., instance=)`, `armed_intervals()`, `armed_mask()`.
- `tests/test_scenarios.py`: 24 labelled fault-injection flights written in Copter-4.6.3's
  exact log format (dual GPS, dual compass, three IMUs, two EKF cores) run through the full
  pipeline; a clean flight that raises a single warning fails CI.

### Changed
- **Parsing is ~20x faster and uses ~4x less memory.** Messages are decoded into numpy
  columns straight from pymavlink's own index instead of one Python object per record (a
  20-minute log with 400 Hz IMU: 26 s / 1.6 GB -> 1.2 s / 0.4 GB). The decoder is checked
  field-for-field against pymavlink and falls back to it for unusual layouts. Raw sensor
  dumps no check reads (`ISBH/ISBD/ACC/GYR`) are not loaded.
- `analyze_log` lists each finding once (in `findings`); `results[]` now carries
  `finding_count` instead of a second copy, roughly halving the payload sent to the LLM.
- The server keeps at most two parsed logs in memory (LRU) instead of every log ever opened.
- A half-written final record (what unplugging the battery after landing leaves) is dropped
  without flagging the log as truncated.

## [0.1.3] — 2026-10-17

### Fixed
- Pinned `mcp` to `>=1.27,<2`. mcp 2.0 (July 2026) removed `mcp.server.fastmcp`, so fresh
  installs (`uvx ardupilot-mcp`, `pip install ardupilot-mcp`) crashed on startup with
  `ModuleNotFoundError`.

### Added
- `.oss-scanner/Dockerfile` and `.oss-scanner/threat_model.md` for Anthropic's OSS Scanner.
- A security contact email in `SECURITY.md`.

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
[0.1.3]: https://pypi.org/project/ardupilot-mcp/0.1.3/
[0.1.2]: https://pypi.org/project/ardupilot-mcp/0.1.2/
[0.1.1]: https://pypi.org/project/ardupilot-mcp/0.1.1/
[0.1.0]: https://pypi.org/project/ardupilot-mcp/0.1.0/
