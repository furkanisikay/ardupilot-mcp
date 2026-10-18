# Security policy

## Supported versions

Security fixes go into the latest release on [PyPI](https://pypi.org/project/ardupilot-mcp/). Upgrade before you report.

## Report a vulnerability

Report privately, not in a public issue:

- GitHub: Security → [Report a vulnerability](https://github.com/furkanisikay/ardupilot-mcp/security/advisories/new)
- Email: contact@furkanisikay.com

The maintainer confirms the report and agrees a fix and a disclosure date with you.

## What the server does

ardupilot-mcp reads local `.bin`, `.tlog` and `.param` files and exposes read-only MCP tools (`readOnlyHint=true`). It opens no network socket, connects to no vehicle and writes no parameter, so it cannot move an aircraft. A change that adds network access or actuation is out of scope for this project.

The default transport is stdio. If you run the server over a network transport, follow the MCP authorization guidance and bind it to localhost.

## In scope

- A crafted log or parameter file that crashes the parser, hangs it or exhausts memory.
- A tool argument that reads files outside what the user asked for.
- Text from a log that reaches the model without being marked as data.

The threat model in [`.oss-scanner/threat_model.md`](../.oss-scanner/threat_model.md) rates these.

A wrong or missed diagnosis is not a security issue; report it as a [wrong diagnosis](https://github.com/furkanisikay/ardupilot-mcp/issues/new?template=wrong_or_missed_diagnosis.yml). The tool supports a pilot's judgement and does not replace it.
