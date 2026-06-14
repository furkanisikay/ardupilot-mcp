"""Tests for the MCP tool layer (calling the tool functions directly)."""

from __future__ import annotations

import pytest

from ardupilot_mcp.model import (
    CheckCatalog,
    DiagnosisReport,
    EventTimeline,
    LogSummary,
    ParamResult,
    TimeseriesResult,
)
from ardupilot_mcp.server import (
    analyze_log,
    get_params,
    list_checks,
    list_events,
    log_summary,
    query_timeseries,
)
from tests.synth_bin import build_bin


@pytest.fixture
def sample_log(tmp_path):
    msgs = []
    for i in range(40):
        t = i * 0.05
        msgs.append(
            (
                "VIBE",
                {
                    "TimeUS": int(t * 1e6),
                    "VibeX": 10,
                    "VibeY": 10,
                    "VibeZ": 70,
                    "Clip0": i * 5,
                    "Clip1": 0,
                    "Clip2": 0,
                },
            )
        )
        msgs.append(
            (
                "GPS",
                {
                    "TimeUS": int(t * 1e6),
                    "Status": 3,
                    "NSats": 12,
                    "HDop": 0.8,
                    "Alt": 30.0,
                    "Spd": 1.0,
                    "Yaw": 0.0,
                },
            )
        )
    msgs.append(("MODE", {"TimeUS": 0, "Mode": 0, "ModeNum": 0, "Rsn": 1}))
    msgs.append(("ERR", {"TimeUS": 500_000, "Subsys": 6, "ECode": 1}))
    p = str(tmp_path / "s.bin")
    build_bin(
        p,
        msgs,
        params={"INS_HNTCH_ENABLE": 1.0, "INS_HNTCH_FREQ": 80.0, "ATC_RAT_RLL_P": 0.135},
        firmware="ArduCopter V4.5.7 (synthetic)",
    )
    return p


def test_analyze_log_returns_report_with_findings(sample_log):
    rep = analyze_log(sample_log)
    assert isinstance(rep, DiagnosisReport)
    assert rep.critical_count >= 1
    assert any(f.check_id == "vibration" for f in rep.findings)


def test_log_summary(sample_log):
    s = log_summary(sample_log)
    assert isinstance(s, LogSummary)
    assert s.vehicle_type == "ArduCopter"
    assert "VIBE" in s.available_messages


def test_list_events_and_filter(sample_log):
    tl = list_events(sample_log)
    assert isinstance(tl, EventTimeline)
    assert any(e.kind == "ERR" for e in tl.events)
    only_err = list_events(sample_log, kinds=["ERR"])
    assert only_err.events and all(e.kind == "ERR" for e in only_err.events)


def test_query_timeseries_downsamples(sample_log):
    ts = query_timeseries(sample_log, "VIBE", ["VibeZ"], max_points=10)
    assert isinstance(ts, TimeseriesResult)
    assert ts.sample_count == 40
    assert ts.downsampled is True
    assert len(ts.times_s) <= 10
    assert len(ts.series["VibeZ"]) == len(ts.times_s)


def test_query_timeseries_missing_type(sample_log):
    ts = query_timeseries(sample_log, "NOPE", ["X"])
    assert ts.times_s == [] and "not present" in (ts.note or "")


def test_get_params_glob(sample_log):
    pr = get_params(sample_log, "INS_HNTCH_*")
    assert isinstance(pr, ParamResult)
    assert set(pr.params) == {"INS_HNTCH_ENABLE", "INS_HNTCH_FREQ"}
    assert pr.matched == 2


def test_list_checks_includes_vibration():
    cat = list_checks()
    assert isinstance(cat, CheckCatalog)
    ids = {c.check_id for c in cat.checks}
    assert "vibration" in ids
    assert cat.total >= 1


def test_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        analyze_log("nope_does_not_exist.bin")
