"""Docs-sync gate: documentation cannot drift from the code.

Fails if an MCP tool isn't documented in the READMEs, a registered check has no test,
a generated file or region (docs/reference.md, the manifests, the README example, counts
and benchmark table) is out of date, or a source file lacks its license identifier.
"""

from __future__ import annotations

import asyncio
import os
from pathlib import Path

import pytest

from ardupilot_mcp.checks import get_checks
from ardupilot_mcp.server import mcp
from scripts import gen_docs

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(name: str) -> str:
    with open(os.path.join(_ROOT, name), encoding="utf-8") as fh:
        return fh.read()


def test_every_mcp_tool_is_documented_in_readme():
    tools = [t.name for t in asyncio.run(mcp.list_tools())]
    assert tools, "expected the server to expose tools"
    for readme in ("README.md", "README.en.md"):
        text = _read(readme)
        missing = [name for name in tools if f"`{name}`" not in text]
        assert not missing, f"MCP tools missing from the {readme} tools table: {missing}"


def test_generated_files_are_current():
    stale = [str(p.relative_to(gen_docs.ROOT)) for p in gen_docs.stale()]
    assert not stale, f"out of date: {stale}; run `python scripts/gen_docs.py`"


def test_every_source_file_has_a_license_identifier():
    root = Path(_ROOT)
    missing = [
        str(p.relative_to(root))
        for d in ("ardupilot_mcp", "scripts", "bench", "examples")
        for p in sorted((root / d).rglob("*.py"))
        if "SPDX-License-Identifier: AGPL-3.0-only"
        not in "".join(p.read_text(encoding="utf-8").splitlines(True)[:2])
    ]
    assert not missing, f"add `# SPDX-License-Identifier: AGPL-3.0-only` as the first line of: {missing}"


def test_every_check_has_a_test_file():
    checks_dir = os.path.join(_ROOT, "tests", "checks")
    missing = [c.id for c in get_checks() if not os.path.exists(os.path.join(checks_dir, f"test_{c.id}.py"))]
    assert not missing, f"registered checks without a tests/checks/test_<id>.py: {missing}"


def test_every_check_has_id_title_and_description():
    # Each check must be self-describing for list_checks / the catalogue.
    bad = [c.id for c in get_checks() if not (c.id and c.title and c.description)]
    assert not bad, f"checks missing id/title/description: {bad}"


def test_replace_region_rewrites_only_between_markers():
    text = "a <!-- gen:n -->old<!-- /gen:n --> b"
    assert gen_docs.replace_region(text, "n", "new") == "a <!-- gen:n -->new<!-- /gen:n --> b"


@pytest.mark.parametrize(
    "text",
    [
        "no markers at all",
        "<!-- gen:n -->only the start",
        "<!-- gen:n -->x<!-- /gen:n --> <!-- gen:n -->y<!-- /gen:n -->",
        "<!-- /gen:n -->end before start<!-- gen:n -->",
    ],
)
def test_replace_region_fails_on_missing_duplicate_or_reversed_markers(text):
    with pytest.raises(ValueError, match="gen:n"):
        gen_docs.replace_region(text, "n", "new")


def test_readme_example_is_real_demo_output():
    example = gen_docs.render_example()
    assert example.startswith("\n```text\nFirst anomaly")
    for check_id in gen_docs.EXAMPLE_CHECKS:
        assert f"[CRITICAL] {check_id}" in example
