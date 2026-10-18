"""End to end over the MCP protocol: a client talks to the server in-process.

Covers both eras a client may use: the initialize handshake (2024-11-05 to
2025-11-25, what most desktop clients speak today) and the stateless 2026-07-28
revision.
"""

from __future__ import annotations

import asyncio

import pytest
from mcp import Client
from mcp.types.version import LATEST_HANDSHAKE_VERSION

from ardupilot_mcp import __version__
from ardupilot_mcp.server import mcp
from tests.synth_bin import build_bin


@pytest.fixture
def log_path(tmp_path):
    path = tmp_path / "flight.bin"
    vibe = [
        (
            "VIBE",
            {
                "TimeUS": i * 100_000,
                "VibeX": 10,
                "VibeY": 10,
                "VibeZ": 70,
                "Clip0": 0,
                "Clip1": 0,
                "Clip2": 0,
            },
        )
        for i in range(40)
    ]
    build_bin(str(path), vibe)
    return str(path)


# The revision the docs promise, pinned: a newer SDK default must not silently replace it.
@pytest.mark.parametrize("mode", ["legacy", "2026-07-28"])
def test_a_client_lists_read_only_tools_and_gets_a_structured_report(mode, log_path):
    async def session() -> tuple[str, list, dict]:
        async with Client(mcp, mode=mode) as client:
            tools = (await client.list_tools()).tools
            result = await client.call_tool("analyze_log", {"path": log_path})
            assert result.structured_content is not None
            return client.protocol_version, tools, result.structured_content

    version, tools, report = asyncio.run(session())
    assert version == (LATEST_HANDSHAKE_VERSION if mode == "legacy" else mode)
    assert {t.name for t in tools} >= {"analyze_log", "incident_timeline", "forum_report"}
    assert all(t.annotations and t.annotations.read_only_hint for t in tools)
    assert all(t.annotations.idempotent_hint and t.annotations.open_world_hint is False for t in tools)
    assert report["log_summary"]["path"].endswith("flight.bin")
    assert any(f["check_id"] == "vibration" for f in report["findings"])


def test_the_server_identifies_itself():
    async def info():
        async with Client(mcp, mode="legacy") as client:
            return client.server_info

    server = asyncio.run(info())
    assert server is not None
    assert (server.name, server.version) == ("ardupilot-mcp", __version__)
