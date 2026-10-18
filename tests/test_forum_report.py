"""Tests for the forum paste report."""

from __future__ import annotations

import numpy as np

from ardupilot_mcp.forum_report import MAX_FINDINGS, build_forum_report, sparkline
from ardupilot_mcp.orchestrator import diagnose
from ardupilot_mcp.timeline import build_timeline
from tests.helpers import build_series, make_flight_log


def _report(log):
    rep = diagnose(log)
    return build_forum_report(log, rep, build_timeline(log, rep.findings))


def _crash_log():
    n = 200
    vibe = build_series(
        0.0, 10.0, VibeX=[5.0] * 150 + [70.0] * 50, VibeY=[5.0] * n, VibeZ=[5.0] * n, Clip=[0] * n
    )
    gps = build_series(
        0.0,
        5.0,
        Lat=[-35.3632621] * 100,
        Lng=[149.1652374] * 100,
        NSats=[14] * 100,
        HDop=[0.7] * 100,
        Status=[3] * 100,
    )
    ev = [{"TimeUS": 0, "Id": 10}, {"TimeUS": 1_000_000, "Id": 28}]
    return make_flight_log(
        {"VIBE": vibe, "GPS": gps, "EV": ev},
        params={"FRAME_CLASS": 1, "FRAME_TYPE": 1},
        vehicle_kind="copter",
        path="/home/someone/private/flight 12.bin",
    )


def test_report_has_the_sections_and_a_plot():
    md = _report(_crash_log())
    assert md.startswith("### Log review: `flight 12.bin`")
    assert "#### What went wrong first" in md and "#### Findings" in md
    assert "VIBE.VibeX" in md and "```" in md


def test_report_never_contains_positions_or_the_full_path():
    md = _report(_crash_log())
    assert "35.36" not in md and "149.16" not in md
    assert "/home/someone" not in md


def test_clean_log_says_so():
    log = make_flight_log({"ATT": build_series(0.0, 10.0, Roll=[0.0] * 50, DesRoll=[0.0] * 50)})
    md = _report(log)
    assert "No warnings" in md


def test_findings_are_capped_and_the_rest_named():
    from ardupilot_mcp.checks.util import make_finding
    from ardupilot_mcp.model import Severity

    log = make_flight_log({})
    rep = diagnose(log)
    extra = [make_finding("config", Severity.WARN, f"Problem {i}", "x") for i in range(MAX_FINDINGS + 3)]
    md = build_forum_report(log, rep.model_copy(update={"findings": extra}), None)
    assert f"{MAX_FINDINGS}. **WARN: Problem {MAX_FINDINGS - 1}**" in md
    assert f"{MAX_FINDINGS + 1}. **" not in md
    assert "_3 more:_" in md and f"Problem {MAX_FINDINGS + 2}" in md


def test_sparkline_spans_lowest_to_highest():
    line = sparkline(np.array([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]), width=8)
    assert line == "▁▂▃▄▅▆▇█"
    assert sparkline(np.array([3.0, 3.0, 3.0])) == "▁▁▁"
    assert sparkline(np.array([np.nan])) == ""


def test_yaw_through_north_is_not_drawn_as_a_spike():
    from ardupilot_mcp.checks.util import make_finding
    from ardupilot_mcp.forum_report import _plot
    from ardupilot_mcp.model import Severity

    yaw = [358.0, 359.0, 0.5, 1.5, 2.5] * 4
    log = make_flight_log({"ATT": build_series(0.0, 2.0, Yaw=yaw, DesYaw=yaw)})
    f = make_finding(
        "attitude", Severity.WARN, "Attitude tracking failure (yaw)", "x", time_start_s=2.0, time_end_s=8.0
    )
    lines = "\n".join(_plot(log, f))
    # Unwrapped, the heading stays within a few degrees instead of spanning 0..359.
    assert " 359" not in lines.split("ATT.Yaw")[1]


def test_battery_plot_reads_only_that_pack():
    from ardupilot_mcp.checks.util import make_finding
    from ardupilot_mcp.forum_report import _plot
    from ardupilot_mcp.model import Severity
    from tests.helpers import interleave_instances

    packs = {
        0: build_series(0.0, 10.0, Volt=[16.0] * 100, Curr=[10.0] * 100),
        1: build_series(0.0, 10.0, Volt=[8.0] * 100, Curr=[1.0] * 100),
    }
    log = make_flight_log({"BAT": interleave_instances("Inst", packs)})
    f = make_finding("power", Severity.WARN, "Voltage sag under load (battery 2)", "x", time_start_s=2.0)
    f.evidence.samples["battery_instance"] = 1
    lines = "\n".join(_plot(log, f))
    assert "BAT.Volt[1]" in lines and " 16" not in lines


def test_demand_and_actual_share_one_scale():
    from ardupilot_mcp.checks.util import make_finding
    from ardupilot_mcp.forum_report import _plot
    from ardupilot_mcp.model import Severity

    n = 50
    log = make_flight_log(
        {"ATT": build_series(0.0, 10.0, DesRoll=[i / 10 for i in range(n)], Roll=[i * 1.2 for i in range(n)])}
    )
    f = make_finding("attitude", Severity.WARN, "Attitude tracking failure (roll)", "x", time_start_s=1.0)
    plot = _plot(log, f)
    des = next(line for line in plot if line.startswith("ATT.DesRoll"))
    # The demand peaks at 4.9 deg against a 58.8 deg actual: it stays near the bottom.
    assert "█" not in des


def test_only_after_impact_findings_do_not_claim_every_check_passed():
    from ardupilot_mcp.checks.util import make_finding
    from ardupilot_mcp.model import Severity

    log = make_flight_log({})
    g = make_finding("vibration", Severity.CRITICAL, "Elevated VibeX vibration", "x", time_start_s=5.0)
    g.after_impact = True
    md = build_forum_report(log, diagnose(log).model_copy(update={"findings": [g]}))
    assert "Every check that had data passed" not in md and "No warnings before the impact" in md


def test_a_log_without_power_data_lists_power_as_not_checked():
    md = _report(make_flight_log({}))
    assert "Battery & power:" in md


def test_flight_long_conditions_appear_once_in_their_own_section():
    from ardupilot_mcp.checks.util import make_finding
    from ardupilot_mcp.model import IncidentTimeline, Severity

    log = make_flight_log({})
    rep = diagnose(log)
    cond = make_finding("config", Severity.WARN, "Arming safety checks disabled", "x")
    event = make_finding("rcin", Severity.WARN, "RC signal loss", "x", time_start_s=10.0)
    tl = IncidentTimeline(path="x", onset_s=10.0, onset_basis="RC signal loss", conditions=[cond.title])
    md = build_forum_report(log, rep.model_copy(update={"findings": [cond, event]}), tl)
    assert md.count("Arming safety checks disabled") == 1
    assert "1. **WARN: RC signal loss**" in md
