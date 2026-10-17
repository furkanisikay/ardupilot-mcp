# Threat model

## What this project does and where untrusted input enters

ardupilot-mcp is an offline, read-only MCP server that parses ArduPilot DataFlash flight logs (`.bin`) and parameter files (`.param`) and returns structured diagnoses to an LLM client (Claude Desktop, Cursor, VS Code and similar). It runs locally over stdio by default.

Treat the following as adversarial:

- **Log and parameter file contents.** Logs come from crashed or third-party vehicles and are shared on forums; assume a malicious `.bin` or `.param` file. Crashes, hangs, unbounded memory use and quadratic blow-ups in `parser.py`, `param_file.py`, `flight_log.py` and the checks are in scope.
- **Tool arguments chosen by the LLM.** The model can be steered by prompt injection (for example text embedded in a log's MSG records or in a document the user pasted). Assume every tool argument (`path`, `name_glob`, time windows, `area`) is attacker-controlled.
- **Text returned to the LLM.** Strings copied from the log (MSG, PARM names, firmware strings) flow back into the model's context. Report cases where log content can inject instructions without being clearly marked as data.

## Components that matter most / least

- Most: `ardupilot_mcp/server.py` (tool surface, path handling, cache), `parser.py`, `param_file.py`, `flight_log.py`.
- Medium: `checks/`, `orchestrator.py`, `tuning.py`, `profile.py` (numeric logic on parsed data; DoS and crashes matter, wrong diagnoses are not security bugs).
- Low: `ardupilot_docs.py`, `ardupilot_meta.py` (static tables).
- Out of scope: `pymavlink`, `mcp` and `numpy` themselves (report only if ardupilot-mcp uses them unsafely), `examples/`, `tests/`.

## How to exercise it

- The image is built with `pip install -e ".[dev]"`; `pytest` runs offline.
- `/src/.oss-scanner/demo.bin` is a pre-generated crash log. `python examples/generate_demo_log.py <out.bin>` makes more, and `tests/synth_bin.py` builds synthetic logs for targeted cases.
- `python examples/analyze.py <log.bin>` runs the full diagnosis from the command line.
- `python examples/mcp_smoke_test.py` starts the server over stdio and drives it as an MCP client.

## How we rate severity

- Critical: code execution, or writing/modifying files, from a crafted log or tool call.
- High: reading files outside what the user intended (for example an injected `path` that leaks `~/.ssh` or other secrets through tool output), or any way to send data over the network.
- Medium: crash, hang or memory exhaustion from a crafted log; prompt injection through log-derived strings.
- Low: information leaks about the local environment (absolute paths, usernames) in error messages.

## Anything to leave alone

- The tool is read-only by design and has no vehicle link; do not report the absence of authentication on the stdio transport.
- Running the server over a network transport is outside intended use (see SECURITY.md).
- Diagnostic accuracy (a wrong or missed finding) is not a security issue.
