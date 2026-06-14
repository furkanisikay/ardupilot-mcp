# Security policy

## Design: offline & read-only

`ardupilot-mcp` is an **offline, read-only** tool by design. It parses local `.bin` /
`.param` files and exposes only read-only MCP tools (`readOnlyHint=true`). It does **not**
connect to a vehicle, open network sockets, arm/fly anything, or write parameters. It
cannot move a drone. Keep it that way — do not add network I/O or actuation.

If you run it over a network transport (not the default stdio) you are outside the tool's
intended use; follow the MCP authorization guidance and bind to localhost.

## Reporting a vulnerability

Please report security issues privately via GitHub's **"Report a vulnerability"** (Security
→ Advisories) on this repository, rather than opening a public issue. We'll acknowledge and
work with you on a fix and disclosure timeline.

## Scope notes

- Parsing untrusted `.bin`/`.param` files: the parser is defensive (it tolerates truncated
  and corrupt logs), but report any input that causes a crash or hang.
- Findings are advisory. Never treat this tool's output as a substitute for a qualified
  operator's judgement before flying.
