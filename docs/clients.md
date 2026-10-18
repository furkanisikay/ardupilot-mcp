# Client setup

Every configuration below starts the server with `uvx ardupilot-mcp`. Install [uv](https://docs.astral.sh/uv/getting-started/installation/) once and the configurations work as written; uv also fetches a suitable Python (3.11 or newer).

If you prefer pip, run `pip install ardupilot-mcp` and replace `"command": "uvx", "args": ["ardupilot-mcp"]` with `"command": "ardupilot-mcp"` in the configuration. Use one of the two, not both.

After you add or change a configuration, quit the client completely and open it again. Closing the window is not enough for Claude Desktop, Cursor, VS Code, Windsurf, LM Studio or Zed, because they keep running in the system tray or menu bar.

## One-click install

[![Add to Cursor](https://img.shields.io/badge/Add_to-Cursor-000000?style=flat-square&logo=cursor&logoColor=white)](cursor://anysphere.cursor-deeplink/mcp/install?name=ardupilot-mcp&config=eyJjb21tYW5kIjoidXZ4IiwiYXJncyI6WyJhcmR1cGlsb3QtbWNwIl19)
[![Install in VS Code](https://img.shields.io/badge/VS_Code-Install_Server-0098FF?style=flat-square&logo=visualstudiocode&logoColor=white)](https://insiders.vscode.dev/redirect/mcp/install?name=ardupilot-mcp&config=%7B%22command%22%3A%22uvx%22%2C%22args%22%3A%5B%22ardupilot-mcp%22%5D%7D)
[![Install in VS Code Insiders](https://img.shields.io/badge/VS_Code_Insiders-Install_Server-24bfa5?style=flat-square&logo=visualstudiocode&logoColor=white)](https://insiders.vscode.dev/redirect/mcp/install?name=ardupilot-mcp&config=%7B%22command%22%3A%22uvx%22%2C%22args%22%3A%5B%22ardupilot-mcp%22%5D%7D&quality=insiders)
[![Add to LM Studio](https://files.lmstudio.ai/deeplink/mcp-install-light.svg)](https://lmstudio.ai/install-mcp?name=ardupilot-mcp&config=eyJhcmR1cGlsb3QtbWNwIjp7ImNvbW1hbmQiOiJ1dngiLCJhcmdzIjpbImFyZHVwaWxvdC1tY3AiXX19)

## Configuration by client

<details>
<summary>Claude Code</summary>

```bash
claude mcp add ardupilot-mcp -- uvx ardupilot-mcp
```

Add `--scope user` to use it in every project, or `--scope project` to share it with your team through `.mcp.json`. Check it with `claude mcp list`.

The plugin adds the server and a `/diagnose` command:

```
/plugin marketplace add furkanisikay/ardupilot-mcp
/plugin install ardupilot-mcp@ardupilot-tools
```
</details>

<details>
<summary>Claude Desktop</summary>

Settings → Developer → Edit Config opens `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

<details>
<summary>Cursor</summary>

`~/.cursor/mcp.json`, or `.cursor/mcp.json` for one project:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

<details>
<summary>VS Code and VS Code Insiders</summary>

`.vscode/mcp.json`:

```json
{
  "servers": {
    "ardupilot-mcp": { "type": "stdio", "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

From the command line: `code --add-mcp '{"name":"ardupilot-mcp","command":"uvx","args":["ardupilot-mcp"]}'`
</details>

<details>
<summary>Antigravity (IDE and CLI)</summary>

The IDE and the CLI share `~/.gemini/config/mcp_config.json` (Windows: `%USERPROFILE%\.gemini\config\mcp_config.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

In the IDE: agent panel ... → MCP Servers → View raw config, paste, then Refresh. In the CLI, run `agy` and check with `/mcp`.
</details>

<details>
<summary>Windsurf</summary>

`~/.codeium/windsurf/mcp_config.json` (Windows: `%USERPROFILE%\.codeium\windsurf\mcp_config.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```

Cascade → MCP panel → View raw config, paste, then Refresh.
</details>

<details>
<summary>Cline</summary>

Cline panel → MCP Servers → Configure opens `cline_mcp_settings.json`:

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"], "disabled": false, "autoApprove": [] }
  }
}
```
</details>

<details>
<summary>Continue</summary>

`.continue/mcpServers/ardupilot-mcp.yaml` in the workspace:

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
<summary>Zed</summary>

`settings.json`, under `context_servers`:

```json
{
  "context_servers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"], "env": {} }
  }
}
```
</details>

<details>
<summary>Goose</summary>

```bash
goose configure
# Add Extension → Command-line Extension → command: uvx ardupilot-mcp
```
</details>

<details>
<summary>LM Studio</summary>

`~/.lmstudio/mcp.json` (Windows: `%USERPROFILE%\.lmstudio\mcp.json`):

```json
{
  "mcpServers": {
    "ardupilot-mcp": { "command": "uvx", "args": ["ardupilot-mcp"] }
  }
}
```
</details>

The server speaks MCP over stdio, so it works with any client that supports MCP, including local models in LM Studio or clients pointed at Ollama. The model only needs tool calling. You need no Anthropic or OpenAI account.

## What to ask

- "Why did `C:\logs\flight.bin` crash?" runs `analyze_log` and explains the findings.
- "What failed first?" uses `incident_timeline`.
- "Was this quad powerful enough?" uses `vehicle_profile` for the power margin.
- "Was vibration high between 15 and 30 seconds?" pulls `VIBE` with `query_timeseries`.
- "Show my `INS_HNTCH_*` parameters and suggest a notch." combines `get_params` and `recommend_tuning`.
- "What changed since last week's flight?" compares two logs with `compare_logs`.
- "Write a reply for my forum thread." returns a `forum_report` you review before posting.

In Claude Code with the plugin installed, `/diagnose C:\logs\flight.bin` runs the whole diagnosis.

## Try it without a client

The example scripts are not in the pip package, so clone the repository:

```bash
git clone https://github.com/furkanisikay/ardupilot-mcp
cd ardupilot-mcp
pip install -e .
python examples/generate_demo_log.py demo.bin   # a synthetic crash log
python examples/analyze.py demo.bin              # the full diagnosis in your terminal
```

Replace `demo.bin` with your own log when you are ready.

## Troubleshooting

| Symptom | Fix |
|---|---|
| The tools never appear, or `command not found: uvx` | Install uv, or install with pip and set `"command": "ardupilot-mcp"`. |
| You changed the configuration and nothing happened | Quit the client completely; closing the window leaves it running. |
| The client reports invalid configuration | Check the file path for your client above and look for a missing comma or brace in the JSON. |
| "File not found" | Give the full path including the extension. On Windows, put the path in double quotes. |
| You are not sure the engine works | Run the commands under [Try it without a client](#try-it-without-a-client). |
