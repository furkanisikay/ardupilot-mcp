# SPDX-License-Identifier: AGPL-3.0-only
"""Entry point: ``python -m ardupilot_mcp`` (or the ``ardupilot-mcp`` script).

Runs the MCP server over stdio — the transport Claude Desktop and other local
MCP clients use.
"""

from __future__ import annotations

import sys

from . import __version__


def main(argv: list[str] | None = None) -> None:
    args = sys.argv[1:] if argv is None else argv
    if "--version" in args:
        print(f"ardupilot-mcp {__version__}")
        return
    from .server import mcp

    mcp.run(transport="stdio")


if __name__ == "__main__":
    main()
