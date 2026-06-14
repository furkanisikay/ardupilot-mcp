"""Docs-sync gate: documentation cannot drift from the code.

Fails if an MCP tool isn't documented in the README, or a registered check has no
test — so adding a tool/check forces updating the docs and tests.
"""

from __future__ import annotations

import asyncio
import os

from ardupilot_mcp.checks import get_checks
from ardupilot_mcp.server import mcp

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(name: str) -> str:
    with open(os.path.join(_ROOT, name), encoding="utf-8") as fh:
        return fh.read()


def test_every_mcp_tool_is_documented_in_readme():
    readme = _read("README.md")
    tools = [t.name for t in asyncio.run(mcp.list_tools())]
    assert tools, "expected the server to expose tools"
    missing = [name for name in tools if name not in readme]
    assert not missing, f"MCP tools missing from README.md tools table: {missing}"


def test_every_check_has_a_test_file():
    checks_dir = os.path.join(_ROOT, "tests", "checks")
    missing = [c.id for c in get_checks() if not os.path.exists(os.path.join(checks_dir, f"test_{c.id}.py"))]
    assert not missing, f"registered checks without a tests/checks/test_<id>.py: {missing}"


def test_every_check_has_id_title_and_description():
    # Each check must be self-describing for list_checks / the catalogue.
    bad = [c.id for c in get_checks() if not (c.id and c.title and c.description)]
    assert not bad, f"checks missing id/title/description: {bad}"
