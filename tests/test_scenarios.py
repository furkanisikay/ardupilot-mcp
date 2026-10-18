"""End-to-end regression gate over labelled fault-injection scenarios.

Each scenario is written in Copter-4.6.3's exact DataFlash format (see
``tests/scenarios``), parsed by the real parser and diagnosed by the full check
registry. A scenario passes only when every expected finding fires within its
severity band AND nothing else reaches WARN or above — so a clean flight that
raises a single false warning fails CI.
"""

from __future__ import annotations

import pytest

from ardupilot_mcp.model import Finding
from ardupilot_mcp.orchestrator import diagnose
from ardupilot_mcp.parser import parse_bin
from tests.scenarios.flights import SCENARIOS, Expect, Scenario, write_flight

_RANK = {"info": 0, "warn": 1, "critical": 2}


def _matches(f: Finding, e: Expect) -> bool:
    return (
        f.check_id == e.check
        and _RANK[e.min_severity] <= f.severity.rank <= _RANK[e.max_severity]
        and e.title.lower() in f.title.lower()
    )


@pytest.mark.parametrize("scenario", SCENARIOS, ids=[s.name for s in SCENARIOS])
def test_scenario(scenario: Scenario, tmp_path) -> None:
    path = str(tmp_path / f"{scenario.name}.bin")
    write_flight(path, scenario.inject, duration=scenario.duration)
    report = diagnose(parse_bin(path))

    assert not [r for r in report.results if r.status.value == "error"], "a check raised"
    missed = [e for e in scenario.expect if not any(_matches(f, e) for f in report.findings)]
    assert not missed, f"expected findings did not fire: {missed}"
    false_alarms = [
        f"{f.severity.value} {f.check_id}: {f.title}"
        for f in report.findings
        if f.severity.rank >= _RANK["warn"] and not any(_matches(f, e) for e in scenario.expect)
    ]
    assert not false_alarms, f"false alarms: {false_alarms}"


def test_clean_flight_summary_is_sane(tmp_path) -> None:
    path = str(tmp_path / "clean.bin")
    write_flight(path, {})
    log = parse_bin(path)
    assert log.meta.vehicle_kind == "copter"
    assert log.meta.flight_modes == ["STABILIZE", "LOITER"]
    # Above home, not AMSL (the vehicle sits at ~584 m AMSL).
    assert log.meta.max_altitude_m is not None and 19.0 < log.meta.max_altitude_m < 21.0
    assert log.instances("VIBE") == [0, 1, 2]
    assert log.instances("GPS") == [0, 1]
