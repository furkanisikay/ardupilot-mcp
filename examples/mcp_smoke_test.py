"""End-to-end MCP smoke test: spawn the server over stdio and drive it as a client.

Proves the package works as a real MCP server (initialize -> list tools -> call
tools), exactly as Claude Desktop would. Generates a demo crash log first.

Run:  python examples/mcp_smoke_test.py
"""

from __future__ import annotations

import asyncio
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from mcp import ClientSession, StdioServerParameters  # noqa: E402
from mcp.client.stdio import stdio_client  # noqa: E402

from examples.generate_demo_log import build_crash_log  # noqa: E402


async def main() -> int:
    log_path = os.path.join(tempfile.gettempdir(), "ardupilot_mcp_demo.bin")
    build_crash_log(log_path)

    params = StdioServerParameters(
        command=sys.executable,  # the current (venv) interpreter
        args=["-m", "ardupilot_mcp"],
        cwd=REPO,
    )

    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            init = await session.initialize()
            print(f"connected to server: {init.serverInfo.name} v{init.serverInfo.version}")

            tools = (await session.list_tools()).tools
            print(f"\ntools ({len(tools)}):")
            for t in tools:
                ro = t.annotations.readOnlyHint if t.annotations else None
                print(f"  - {t.name:18} readOnly={ro}")

            print("\ncall analyze_log:")
            res = await session.call_tool("analyze_log", {"path": log_path})
            report = res.structuredContent
            assert report is not None, "expected structuredContent"
            print(f"  critical={report['critical_count']} warn={report['warn_count']} info={report['info_count']}")
            for f in report["findings"][:5]:
                refs = ", ".join(r["url"].rsplit("/", 1)[-1] for r in f.get("references", []))
                print(f"    [{f['severity']}] {f['check_id']}: {f['title']}")
                if refs:
                    print(f"        docs: {refs}")
            print(f"  report references: {len(report.get('references', []))} doc links")

            print("\ncall vehicle_profile:")
            res = await session.call_tool("vehicle_profile", {"path": log_path})
            prof = res.structuredContent
            print(f"  {prof.get('frame_class_name')} ({prof.get('frame_type_name')}), "
                  f"{prof.get('motor_count')} motors, {prof.get('battery_cells')}S, "
                  f"hover {prof.get('hover_throttle')}, margin {prof.get('power_margin_pct')}%")
            for a in prof.get("assessment", []):
                print(f"    • {a}")

            print("\ncall recommend_tuning:")
            res = await session.call_tool("recommend_tuning", {"path": log_path})
            for r in res.structuredContent["recommendations"]:
                print(f"    ({r['area']}) {r['title']}")

            print("\ncall list_checks:")
            res = await session.call_tool("list_checks", {})
            print(f"    {res.structuredContent['total']} checks: "
                  f"{[c['check_id'] for c in res.structuredContent['checks']]}")

    print("\nMCP smoke test OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
