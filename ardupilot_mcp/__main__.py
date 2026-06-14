"""Entry point: ``python -m ardupilot_mcp`` (or the ``ardupilot-mcp`` script).

Runs the MCP server over stdio — the transport Claude Desktop and other local
MCP clients use.
"""

from __future__ import annotations

from .server import mcp


def main() -> None:
    mcp.run()  # stdio transport by default


if __name__ == "__main__":
    main()
