# ardupilot-mcp

[![CI](https://github.com/furkanisikay/ardupilot-mcp/actions/workflows/ci.yml/badge.svg)](https://github.com/furkanisikay/ardupilot-mcp/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/ardupilot-mcp.svg)](https://pypi.org/project/ardupilot-mcp/)
[![Python](https://img.shields.io/pypi/pyversions/ardupilot-mcp.svg)](https://pypi.org/project/ardupilot-mcp/)
[![License: AGPL-3.0](https://img.shields.io/badge/license-AGPL--3.0-blue.svg)](LICENSE)

[Türkçe](README.md) · English

<!-- mcp-name: io.github.furkanisikay/ardupilot-mcp -->

An MCP server that diagnoses ArduPilot flight logs. Add it to an AI client such as Claude Desktop, Cursor or VS Code, give it a `.bin` or `.tlog` file, and the client explains in plain language what happened in the flight: vibration, a low battery, GPS loss, a lost motor, a bad setting.

Deterministic checks produce the findings. [docs/SOURCES.md](docs/SOURCES.md) records the source of every threshold, and the model only explains the findings. The server reads local files only. It opens no network connection, sends no command to a vehicle and writes no parameter.

## Quick start

With Claude Desktop:

1. Install [uv](https://docs.astral.sh/uv/getting-started/installation/).
2. In Claude Desktop, open Settings → Developer → Edit Config and add:
   ```json
   { "mcpServers": { "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] } } }
   ```
3. Quit Claude Desktop completely and open it again.
4. Ask: *"Analyze the log at C:\logs\00000042.BIN. Why did it crash?"*

Claude Code takes one line: `claude mcp add ardupilot-mcp -- uvx ardupilot-mcp`. Settings for Cursor, VS Code, Windsurf, Cline, Zed, LM Studio and other clients, troubleshooting, and a way to try it without a client are in [docs/clients.md](docs/clients.md).

## Which log?

- **`.bin`** is the flight controller's own log and gives the most detailed diagnosis. It is in the `APM/LOGS/` folder on the SD card; Mission Planner downloads it under DataFlash Logs → Download DataFlash Log Via Mavlink.
- **`.tlog`** is the telemetry your ground station recorded. It carries a few samples per second and the primary sensor only, so the diagnosis is coarser. Checks that would mistake a radio dropout for a vehicle fault do not run on it.

## Example

An excerpt of `python examples/analyze.py demo.bin`. A client tells you the same findings in your own words.

```text
First anomaly while armed: Elevated VibeZ vibration @ 9s.
6 critical, 2 warning, 3 info finding(s).
[CRITICAL] vibration @ 9s: Elevated VibeZ vibration
           VibeZ vibration peaks at 72 m/s^2 (>60); 20% of samples exceed 60 m/s^2.
[CRITICAL] attitude  @ 20s: Attitude tracking failure (roll)
           Roll failed to track the demand for 3.0 s, error up to 35 deg (sustained >25 deg).
[CRITICAL] power     @ 25s: Battery reached critical voltage
           Pack voltage fell to 13.22 V, at or below the critical-voltage level (13.50 V).
```

## Tools

Every tool is read-only.

| Tool | What it does |
|---|---|
| `analyze_log` | Runs every check and returns severity-ranked findings with official ArduPilot doc links. |
| `incident_timeline` | Finds the first anomaly and the impact and puts the events in time order. It makes no causal claims. |
| `forum_report` | Writes a Markdown reply to paste into discuss.ardupilot.org. It contains no GPS positions. |
| `compare_logs` | Shows the parameters, firmware and metrics that changed between two flights. |
| `log_summary` | Vehicle, firmware, duration, flight modes and log integrity. |
| `vehicle_profile` | Frame, motor count, battery, hover throttle and power margin. |
| `list_events` | Timeline of the `ERR`, `MODE`, `EV` and `MSG` records. |
| `query_timeseries` | The raw series behind a finding, one instance at a time for multi-sensor messages. |
| `get_params` | The log's parameters with the range, units and values documented for its firmware release. |
| `load_param_file` | Reads an exported `.param` file. |
| `recommend_tuning` | Harmonic notch, PID and autotune advice. It applies nothing. |
| `list_checks` | The registered checks. |

Tool arguments and all 18 checks are in [docs/reference.md](docs/reference.md). `scripts/gen_docs.py` generates that file from the code, and CI fails when it is out of date.

## How it works

The parser turns a log into a columnar `FlightLog`. Registered checks read it: a check whose data is not in the log is skipped, and one that does not fit the vehicle type does not run. The ERR record, failsafe mode change and firmware message of one event merge into one finding. Findings that start after the impact go to the end of the list, so the report opens with the first anomaly before the crash.

4.x firmware logs every IMU, GPS, compass, battery and EKF core under one message name, and the checks judge each one separately. Pre-4.1 logs, which write secondary sensors under their own names, are folded into the same structure.

The server runs on the MCP Python SDK 2. It speaks the 2026-07-28 protocol revision and the older initialize handshake (2024-11-05 to 2025-11-25) that most desktop clients use today.

## Accuracy and limits

The engine is measured on 49 real crash logs from the ArduPilot forum, each labelled with the cause that a developer or an experienced helper stated after reading the log. Current scores and the method are in [bench/README.md](bench/README.md).

- Circular drift from a compass heading error (toilet bowl) is not detected yet.
- A log with corrupted format definitions fails with a clear error instead of a partial result.
- A `.tlog` has no ERR records, batch sampler, fast IMU or mode-change reason.
- Findings support a decision. They do not decide whether an aircraft is fit to fly.

## Other tools

[UAV Log Viewer](https://plot.ardupilot.org/) and Mission Planner's [Auto Analysis](https://ardupilot.org/planner/docs/common-diagnosing-problems-using-logs.html) show the data and leave the interpretation to you. ardupilot-mcp interprets it and answers follow-up questions. To see a signal on a graph or replay the flight in 3D, UAV Log Viewer is the better tool.

## Contributing

[CONTRIBUTING.md](.github/CONTRIBUTING.md) covers the dev setup, adding a check and the PR process. [AGENTS.md](AGENTS.md) holds the rules AI assistants follow. Report vulnerabilities as described in [SECURITY.md](.github/SECURITY.md). Changes are listed in [CHANGELOG.md](CHANGELOG.md).

## License

[GNU AGPL-3.0](LICENSE). If you modify the code and offer it as a network service, you must give the users of that service your modified source. If the AGPL terms do not work for you, write to contact@furkanisikay.com for a commercial license. The packages published on PyPI up to 0.1.3 are MIT licensed and stay so; the source from this change on, and every later release, is AGPL-3.0.

The data in `ardupilot_mcp/data/params/` consists of facts extracted from ArduPilot's GPLv3 parameter definitions (`apm.pdef.xml`): ranges, units, value codes and bitmask bits. No description text is copied.
