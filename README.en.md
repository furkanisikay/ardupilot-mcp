# ardupilot-mcp

[![CI](https://github.com/furkanisikay/ardupilot-mcp/actions/workflows/ci.yml/badge.svg)](https://github.com/furkanisikay/ardupilot-mcp/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/ardupilot-mcp.svg)](https://pypi.org/project/ardupilot-mcp/)
[![Python](https://img.shields.io/pypi/pyversions/ardupilot-mcp.svg)](https://pypi.org/project/ardupilot-mcp/)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

[🇹🇷 Türkçe](README.md) · 🇬🇧 **English**

<!-- mcp-name: io.github.furkanisikay/ardupilot-mcp -->

**Did your ArduPilot drone crash or fly badly?** The flight controller saves a black-box file for every flight — a `.bin` flight log. This tool reads that file and lets an AI chat assistant explain, in **plain language**, what went wrong: low battery, bad vibration, GPS loss, a wiring fault, bad tuning… Backed by a deterministic check engine, not vibes.

**What kind of thing is this?** It has no window or app of its own. It plugs into an AI chat app — like **Claude Desktop, Cursor, VS Code** (these are called "MCP clients"). You install one of those first, add this tool to it, then ask in plain language. (MCP = Model Context Protocol, the standard "plug" those apps use to talk to tools.)

## Why this (when Mission Planner and UAV Log Viewer exist)?

Mission Planner's [Auto Analysis](https://ardupilot.org/planner/docs/common-diagnosing-problems-using-logs.html) and the [UAV Log Viewer](https://plot.ardupilot.org/) already exist and are good — but they do a different job. In short: those tools **show you the data and you interpret it**; this one **interprets the data, explains it in plain language, and answers your questions.**

| | MP Auto Analysis | UAV Log Viewer | **ardupilot-mcp** |
|---|:---:|:---:|:---:|
| Output | fixed pass/fail list | graphs + 3D flight replay | plain-language explanation + chat |
| Ask "why did it crash?" and get a reasoned answer | — | — | **✓** |
| Natural-language follow-ups ("vibration 15–30 s?") | — | — | **✓** |
| Config/setup faults (param audit, calibration, wiring, pre-arm) | partly | — | **✓** |
| Physical reasoning (power margin, thrust-to-weight) | — | — | **✓** |
| Official ArduPilot doc link on every finding | — | — | **✓** |
| Graph / 3D map visualization | — | **✓** | — |
| Instant, no setup | **✓** (built into MP) | **✓** (web) | needs an AI client + uv |

**When to use the others:** to eyeball a signal on a graph or replay the flight path in 3D → **UAV Log Viewer**. For a quick pass/fail when Mission Planner is already open → **MP Auto Analysis**. This tool doesn't replace them; it exists to **explain "why did this happen?" and answer your follow-ups**, grounded in a deterministic engine + the official docs, and to cover the **config/setup and physical** side alongside the flight signals. It also runs fully offline and read-only.

## Quick start (5 minutes)

The shortest path — with Claude Desktop:

1. **Install Claude Desktop:** <https://claude.ai/download>
2. **Install uv** — a small, free program that downloads and runs this tool for you (`uvx` comes with it):
   - Windows (PowerShell): `powershell -c "irm https://astral.sh/uv/install.ps1 | iex"`
   - macOS / Linux: `curl -LsSf https://astral.sh/uv/install.sh | sh`
   - Docs: <https://docs.astral.sh/uv/getting-started/installation/> · Did it work: `uv --version` should print a version.
3. **In Claude Desktop** go to Settings → Developer → Edit Config; paste this and save:
   ```json
   { "mcpServers": { "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] } } }
   ```
   Then **fully quit and reopen** Claude Desktop — closing the window is not enough; quit it from the system tray too.
4. **Get a `.bin` log** (see *Where do I get my flight log?*). No log handy? Make a demo one with *Try it with no AI app*.
5. **In the chat box, type:** *"Analyze the log at C:\logs\flight.bin — why did the drone crash?"* (replace the path with your file).

To use pip instead of uv, or to add a different app (Cursor, VS Code, Cline, LM Studio…) → [Add to your client](#add-to-your-client).

## Where do I get my flight log (`.bin`)?

Your flight controller saves a `.bin` DataFlash log for every flight. To get it onto your computer:

- **Easiest — Mission Planner:** connect over USB → **Flight Data** → **DataFlash Logs** → **Download DataFlash Log Via Mavlink** → pick the flight → save it (e.g. to `C:\logs\`). QGroundControl works too.
- **Or from the SD card:** the controller's SD card usually has an `APM/LOGS` or `/LOGS` folder; the file looks like `00000042.BIN`.

Note the full path — that's what you give the AI. More detail: [ArduPilot — Downloading and Analyzing Data Logs](https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html).

## Try it with no AI app

To see the engine work without setting up an AI app, get the source (the example scripts aren't in the pip package — you need a clone):

```bash
git clone https://github.com/furkanisikay/ardupilot-mcp
cd ardupilot-mcp
pip install -e .
python examples/generate_demo_log.py demo.bin   # makes a realistic crash log
python examples/analyze.py demo.bin              # prints the full diagnosis in your terminal
```

When ready, swap `demo.bin` for your own `.bin`.

## What the answer looks like

Trimmed from the `analyze.py demo.bin` run above. In an AI client the same findings are retold in your own language, conversationally:

```
ArduCopter V4.5.7 — 30 s flight — 8 critical, 2 warning, 3 info

[CRITICAL] vibration @ 9s : VibeZ vibration peaked at 72 m/s² (>60); 20% of samples exceed the limit.
[CRITICAL] attitude  @ 20s: roll failed to track the demand for 3 s, error up to 35° — loss of control / mechanical fault.
[CRITICAL] gps       @ 22s: GPS lost 3D fix between 22–24 s — position aiding gone.
[CRITICAL] power     @ 30s: pack voltage fell to 13.22 V (critical 13.50 V failsafe) — land immediately.
...
TUNING (advice only): suggests an 80 Hz harmonic notch.
```

Each finding carries the relevant official ArduPilot doc link and concrete numbers (time, value, threshold).

## Add to your client

One click:

[![Add to Cursor](https://img.shields.io/badge/Add_to-Cursor-000000?style=flat-square&logo=cursor&logoColor=white)](cursor://anysphere.cursor-deeplink/mcp/install?name=ardupilot-mcp&config=eyJjb21tYW5kIjoidXZ4IiwiYXJncyI6WyJhcmR1cGlsb3QtbWNwIl19)
[![Install in VS Code](https://img.shields.io/badge/VS_Code-Install_Server-0098FF?style=flat-square&logo=visualstudiocode&logoColor=white)](https://insiders.vscode.dev/redirect/mcp/install?name=ardupilot-mcp&config=%7B%22command%22%3A%22uvx%22%2C%22args%22%3A%5B%22ardupilot-mcp%22%5D%7D)
[![Install in VS Code Insiders](https://img.shields.io/badge/VS_Code_Insiders-Install_Server-24bfa5?style=flat-square&logo=visualstudiocode&logoColor=white)](https://insiders.vscode.dev/redirect/mcp/install?name=ardupilot-mcp&config=%7B%22command%22%3A%22uvx%22%2C%22args%22%3A%5B%22ardupilot-mcp%22%5D%7D&quality=insiders)
[![Add to LM Studio](https://files.lmstudio.ai/deeplink/mcp-install-light.svg)](https://lmstudio.ai/install-mcp?name=ardupilot-mcp&config=eyJhcmR1cGlsb3QtbWNwIjp7ImNvbW1hbmQiOiJ1dngiLCJhcmdzIjpbImFyZHVwaWxvdC1tY3AiXX19)

**Setup (one time).** Every config below uses the `uvx ardupilot-mcp` command.

- **Recommended — uv:** install it with the one-liner in *Quick start* step 2 above. With uv installed, the configs work as-is and you do **not** need to `pip install` anything.
- **Alternative — pip:** if you ran `pip install ardupilot-mcp`, then in every config below replace `"command": "uvx", "args": ["ardupilot-mcp"]` with `"command": "ardupilot-mcp"` (no args). Both work; just don't mix them.

You also need Python (3.11+). Check with `python --version` in a terminal — it should print 3.11 or higher. Don't have it? Install from <https://python.org/downloads> (on Windows, tick **"Add Python to PATH"** during setup). If you use the uv path, uv can fetch a suitable Python for you, so you may not need to install Python separately.

> **Important:** after adding or editing the config you must **reload the app**. For config-file apps (Claude Desktop, Cursor, VS Code, Windsurf, LM Studio, Zed) **fully quit and reopen** — closing the window is not enough; it stays running in the system tray / menu bar. Then check the tool list.

<details>
<summary><b>Claude Code</b></summary>

```bash
claude mcp add ardupilot-mcp -- uvx ardupilot-mcp
```

Add `--scope user` to use it across all your projects, or `--scope project` to share it with your team (writes `.mcp.json` into the repo). Verify with `claude mcp list`.

Or as a plugin (bundles the MCP server + a `/diagnose` command):

```
/plugin marketplace add furkanisikay/ardupilot-mcp
/plugin install ardupilot-mcp@ardupilot-tools
```
</details>

<details>
<summary><b>Claude Desktop</b></summary>

Settings → Developer → Edit Config to open `claude_desktop_config.json`, then fully quit and restart Claude Desktop:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

<details>
<summary><b>Cursor</b></summary>

The **Add to Cursor** button above; or `~/.cursor/mcp.json` (project-local: `.cursor/mcp.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

<details>
<summary><b>VS Code / VS Code Insiders</b></summary>

The button above; or `.vscode/mcp.json`:

```json
{
  "servers": {
    "ardupilot-mcp": { "type": "stdio", "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

Command line: `code --add-mcp '{"name":"ardupilot-mcp","command":"uvx","args":["ardupilot-mcp"]}'`
</details>

<details>
<summary><b>Antigravity (IDE + CLI)</b></summary>

`~/.gemini/config/mcp_config.json` (Windows: `%USERPROFILE%\.gemini\config\mcp_config.json`) — the IDE and the CLI share this file:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

IDE: agent panel **...** → **MCP Servers** → **View raw config** → paste → **Refresh**. CLI: run `agy`, then `/mcp` to confirm.
</details>

<details>
<summary><b>Windsurf</b></summary>

`~/.codeium/windsurf/mcp_config.json` (Windows: `%USERPROFILE%\.codeium\windsurf\mcp_config.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

Cascade → MCP panel → **View raw config** → paste → **Refresh**.
</details>

<details>
<summary><b>Cline</b></summary>

Cline panel → **MCP Servers** → **Configure** → `cline_mcp_settings.json`:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"], "disabled": false, "autoApprove": [] }
  }
}
```
</details>

<details>
<summary><b>Continue</b></summary>

Create `.continue/mcpServers/ardupilot-mcp.yaml` at your workspace root:

```yaml
name: ArduPilot MCP
version: 0.0.1
schema: v1
mcpServers:
  - name: ArduPilot MCP
    type: stdio
    command: uvx
    args:
      - ardupilot-mcp
```
</details>

<details>
<summary><b>Zed</b></summary>

`settings.json` (under `context_servers`):

```json
{
  "context_servers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"], "env": {} }
  }
}
```
</details>

<details>
<summary><b>Goose</b></summary>

```bash
goose configure
# Add Extension → Command-line Extension → command: uvx ardupilot-mcp
```
</details>

<details>
<summary><b>LM Studio (open-source / local models)</b></summary>

The **Add to LM Studio** button above; or `~/.lmstudio/mcp.json` (Windows: `%USERPROFILE%\.lmstudio\mcp.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

> **Model-agnostic:** ardupilot-mcp speaks MCP over stdio, so it works in any MCP-capable client regardless of the underlying model — including fully open-source / local models (e.g. local GGUF models in LM Studio; Continue / Cline / Goose pointed at an Ollama or LM Studio endpoint). No Anthropic/OpenAI account is required; the client's chosen model just needs tool-calling support.

### If it doesn't work

- **Tool never appears / "command not found: uvx"** → uv isn't installed or isn't found. Install uv (link above), or run `pip install ardupilot-mcp` and change the config to `"command": "ardupilot-mcp"`.
- **Edited the config but it's still missing** → **fully** quit and reopen the app (closing the window isn't enough; it stays in the tray / menu bar).
- **Did I edit the right file?** → use the exact path listed in your client's section above; make sure the JSON has no missing commas/braces.
- **"File not found"** → give the **full path** including the `.bin` extension; on Windows wrap the path in double quotes.
- **Is the engine itself working?** → confirm independently with *Try it with no AI app* above (`python examples/analyze.py demo.bin`).

## Tools (all read-only)

| Tool | What it does |
|------|--------------|
| `analyze_log(path)` | Run the full check suite → severity-ranked findings + summary. Each finding carries **official ArduPilot doc links**, and the report includes version-specific parameter docs for 4.x firmware. The headline. |
| `log_summary(path)` | Vehicle, firmware, duration, message counts, flight modes, max altitude, integrity. |
| `vehicle_profile(path)` | **Physical/architecture profile**: frame & type, motor count, battery cells/capacity, hover throttle, **power margin** and thrust-to-weight. For physically-grounded reasoning. |
| `list_events(path, kinds?, start_s?, end_s?)` | Decoded `ERR`/`MODE`/`EV`/`MSG` timeline. |
| `query_timeseries(path, message_type, fields, start_s?, end_s?, max_points?)` | Decimated numeric series for drilling into a finding. |
| `get_params(path, name_glob?)` | Parameter values from the log's PARM dump (e.g. `INS_HNTCH_*`), plus the total count, a version-specific metadata link to interpret them, and a note if the snapshot looks incomplete. |
| `load_param_file(path, name_glob?)` | Read a user-exported `.param`/`.params` file — the authoritative complete config, for when the log's snapshot is truncated or to compare flown-vs-current. |
| `recommend_tuning(path, area?)` | Advisory tuning (harmonic notch from gyro FFT, PID, autotune). **Recommendation only — never applied.** |
| `list_checks()` | The registered diagnostic checks (extensible catalogue). |

## Example scenarios

Once the server is added, ask your client in plain language; it calls the right tools itself:

- **"Why did `C:\logs\flight.bin` crash?"** — runs the full check suite (`analyze_log`) and explains the severity-ranked findings with their doc links.
- **"Was this airframe powerful enough for its motors?"** — `vehicle_profile` derives the power margin and thrust-to-weight from frame / motor count / battery cells / hover throttle.
- **"Was vibration high between 15 s and 30 s?"** — `query_timeseries` pulls the `VIBE` series and compares it against the thresholds.
- **"Why was arming blocked?"** — `list_events` decodes the `ERR`/`MSG` timeline and pre-arm warnings.
- **"Show my `INS_HNTCH_*` params and suggest a harmonic notch."** — `get_params` + `recommend_tuning` (advice only, never applied).
- **"Compare this `.param` file against the log's settings."** — `load_param_file` checks the flown config against the authoritative file.

If you use **Claude Code** (a different app from Claude Desktop) **and installed the plugin** (the Claude Code section above — the plain MCP-server setup does **not** add `/diagnose`), one command does it all: **`/diagnose C:\logs\flight.bin`** runs the end-to-end diagnosis using the tools above in order. In every other client, just ask in plain language instead.

## Why log diagnosis?

Reading an ArduPilot `.bin` is an expert bottleneck: you filter `ERR` rows, compare `ATT.DesRoll` vs `ATT.Roll` to spot a mechanical failure, and eyeball vibration / EKF / battery clipping. This server runs those checks deterministically and lets the model narrate the result in plain language.

## How it works

```
.bin ──▶ parser.py (pymavlink DFReader) ──▶ FlightLog (pure domain model)
                                               │
                                               ▼
                       checks/ registry ──▶ Check plugins (one per concern)
                                               │
                                               ▼
                    orchestrator.diagnose() ──▶ DiagnosisReport (structured)
                                               │
                                               ▼
                       server.py (FastMCP, stdio) ──▶ LLM narrates the findings
```

- **Core principle:** the deterministic engine is the single source of truth; the LLM only *explains* and cites — it never decides.
- **`FlightLog`** is a pure, `pymavlink`-free domain model, so checks and their tests run against synthetic logs without a real `.bin`.
- **Checks** are independent, registry-registered plugins (`@register_check`). Adding one is a single new module. The orchestrator skips a check cleanly when its data wasn't logged, when it doesn't apply to the vehicle kind (e.g. motor-balance on a heli/plane), and isolates any check that raises.
- The check catalogue has two families. **Flight dynamics:** events/errors, EKF health, vibration, power (BAT/CURR), GPS (fix/sats/HDOP, scoped to the armed window), compass, attitude tracking, motor balance, RC-link loss, timing. **Configuration & setup** (because many crashes are setup mistakes, not flight events): risky parameters (disabled arming checks/failsafes), a **parameter audit** of contradictory/disabled/implausible values, calibration (large/zero compass offsets), configured-but-silent sensors (wiring/connection faults), and the firmware's own pre-arm/startup warnings. Plus advisory tuning.
- It is **vehicle-aware** (copter / heli / plane / rover): multirotor-specific checks don't run on a heli's swashplate servos or a plane's control surfaces. Validated against 40 real forum logs across Copter 3.2–4.6, Plane, QuadPlane, Heli and Rover.
- Findings are **documentation-grounded**: each links the authoritative ArduPilot doc page, and reports surface the firmware-version-specific parameter definitions (`apm.pdef.xml`) for 4.x. The server stays offline/deterministic and hands these *pointers* to the LLM, which has its own web access to read and cite them.
- The **physical profile** (`vehicle_profile`) turns the log into airframe facts — frame, motor count, battery cells, and especially the **power margin** (hover throttle → thrust-to-weight). On a real 12 kg quad that crashed maxing its motors, it reports "underpowered, hovers at 69 %, only 31 % headroom" — the actual cause.

## Safety

This release is **offline and read-only**. No MAVLink connection, no arming, no parameter writes, no actuation. Tuning output is advice only. Live-vehicle features (with a proper safety gateway: SITL-first, human-in-the-loop, kill switch) are explicitly out of scope and deferred.

## Sources & assumptions

Every numeric threshold, enum, default and behavioural claim in the code is audited against authoritative sources (ArduPilot wiki/firmware, MAVLink, the MCP spec) in [`docs/SOURCES.md`](docs/SOURCES.md) — each marked *confirmed* (with a citation), *heuristic* (our conservative choice), or *design choice* (our severity cut-off). That audit found and fixed eight genuine errors (e.g. a wrong heli FRAME_CLASS set, mis-numbered event ids, an EKF field that isn't an innovation test ratio).

## Development

Contributing: see [CONTRIBUTING.md](CONTRIBUTING.md) and [AGENTS.md](AGENTS.md) (the rules for humans and AI assistants).

```bash
pip install -e ".[dev]"
ruff check ardupilot_mcp tests
mypy ardupilot_mcp
pytest
```

Checks are tested against synthetic in-memory logs (`tests/helpers.py`) and a synthetic DataFlash writer (`tests/synth_bin.py`) gives the parser a real, hermetic `.bin` to round-trip — no large binary fixtures committed.

## Known limitations

- **Toilet-bowl / compass-heading faults** aren't detected yet — they need circular-position-drift analysis (planned), not just field-magnitude stability.
- A **genuinely corrupt log** (e.g. a bit-flipped message format) fails to parse with a clear error rather than partial recovery.
- Large logs (3–4 MB / ~100k records) take ~12–15 s to parse; the MCP server caches a parsed log by path+mtime, so only the first tool call pays that cost.

## Roadmap

1. **This release:** ArduPilot offline `.bin` diagnosis + tuning advice + extensible check framework.
2. PX4 / ULog support (separate parser + rules; shared diagnosis abstraction).
3. Live MAVLink connection (SITL-first) with a full safety gateway.
4. Community check-sharing; confirmation-gated tuning *apply*.

## License

MIT
