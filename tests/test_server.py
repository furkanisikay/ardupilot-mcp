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


# ---- query_timeseries: instances and alignment (in-memory logs) ----


@pytest.fixture
def mem_log(monkeypatch):
    from ardupilot_mcp import server
    from tests.helpers import build_series, interleave_instances, make_flight_log

    vibe = interleave_instances(
        "IMU",
        {
            0: build_series(0.0, 10.0, VibeX=[1.0] * 20, Clip=[0] * 20),
            1: build_series(0.0, 10.0, VibeX=[9.0] * 20, Clip=[0] * 20),
        },
    )
    att = build_series(0.0, 10.0, Roll=list(range(10)))
    del att[3]["Roll"]  # one record without the field
    log = make_flight_log({"VIBE": vibe, "ATT": att})
    monkeypatch.setattr(server, "_load", lambda _p: log)
    return log


def test_timeseries_returns_one_instance(mem_log):
    r = query_timeseries("x.bin", "VIBE", ["VibeX"])
    assert r.available_instances == [0, 1]
    assert r.instance == 0
    assert set(r.series["VibeX"]) == {1.0}
    r1 = query_timeseries("x.bin", "VIBE", ["VibeX"], instance=1)
    assert set(r1.series["VibeX"]) == {9.0}
    assert len(r1.times_s) == 20


def test_timeseries_stays_aligned_and_windowed_with_missing_field(mem_log):
    r = query_timeseries("x.bin", "ATT", ["Roll"], start_s=0.2, end_s=0.5)
    assert r.times_s == pytest.approx([0.2, 0.3, 0.4, 0.5])
    assert r.series["Roll"] == [2.0, None, 4.0, 5.0]


def test_timeseries_reports_unknown_field(mem_log):
    r = query_timeseries("x.bin", "ATT", ["Nope"])
    assert r.note and "Nope" in r.note and "Roll" in r.note
    assert set(r.series["Nope"]) == {None}


def test_report_lists_each_finding_once(sample_log):
    rep = analyze_log(sample_log)
    assert all(r.findings == [] for r in rep.results)
    assert sum(r.finding_count for r in rep.results) == len(rep.findings)
