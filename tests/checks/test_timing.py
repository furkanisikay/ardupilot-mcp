"""Tests for the scheduler timing & log-gaps check."""

from __future__ import annotations

from ardupilot_mcp.checks.timing import TimingCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log

# -- empty / skipped-equivalent ------------------------------------------------


def test_empty_log_no_findings():
    # requires=set(), so the check always RUNs; with no relevant messages it
    # must produce no findings.
    log = make_flight_log({})
    result = TimingCheck().execute(log)
    assert result.status == CheckStatus.RAN
    assert result.findings == []


def test_no_relevant_messages_returns_empty():
    # A message type we do not look at must not trigger anything.
    recs = build_series(0.0, 5.0, Alt=[10.0] * 10)
    findings = TimingCheck().run(make_flight_log({"GPS": recs}))
    assert findings == []


# -- logging gaps --------------------------------------------------------------


def test_evenly_spaced_att_no_findings():
    # 50 Hz ATT, perfectly even -> no gap finding.
    recs = build_series(0.0, 50.0, Roll=[0.0] * 200, Pitch=[0.0] * 200)
    findings = TimingCheck().run(make_flight_log({"ATT": recs}))
    assert findings == []


def test_att_gap_inserted_emits_warn():
    # 50 Hz ATT in two segments with a 2.0 s offset between them -> one big gap.
    seg1 = build_series(0.0, 50.0, Roll=[0.0] * 100, Pitch=[0.0] * 100)
    # First segment spans t=0..~1.98 s (100 samples @ 50 Hz). Start segment 2 at
    # 4.0 s so the gap between last of seg1 (~1.98 s) and first of seg2 is ~2.02 s.
    seg2 = build_series(4.0, 50.0, Roll=[0.0] * 100, Pitch=[0.0] * 100)
    recs = seg1 + seg2
    findings = TimingCheck().run(make_flight_log({"ATT": recs}))

    gap_findings = [f for f in findings if "gap" in f.title.lower()]
    assert gap_findings, "expected a logging-gap finding"
    f = gap_findings[0]
    assert f.severity == Severity.WARN
    assert f.evidence.message_types == ["ATT"]
    # The gap should be ~2.0 s; nominal dt 0.02 s.
    assert f.evidence.samples["gap_s"] >= 2.0
    assert abs(f.evidence.samples["nominal_dt_s"] - 0.02) < 1e-3
    # Gap starts at the last sample of segment 1 (~1.98 s).
    assert 1.9 <= f.evidence.samples["gap_start_s"] <= 2.0


def test_multiple_gaps_capped_at_three():
    # Build four segments separated by 3 s offsets -> three gaps, all reported.
    recs: list[dict] = []
    for i in range(4):
        recs += build_series(i * 5.0, 50.0, Roll=[0.0] * 50, Pitch=[0.0] * 50)
    findings = TimingCheck().run(make_flight_log({"ATT": recs}))
    gap_findings = [f for f in findings if "gap" in f.title.lower()]
    # Three inter-segment gaps, cap is three.
    assert len(gap_findings) == 3
    assert all(f.severity == Severity.WARN for f in gap_findings)
    # Reported largest-first; each gap is ~3.0 s here.
    assert all(f.evidence.samples["gap_s"] >= 2.9 for f in gap_findings)


def test_gap_source_prefers_imu_over_att():
    # When both IMU and ATT exist, IMU is the chosen source. Put the gap only in
    # IMU; the finding must cite IMU.
    imu1 = build_series(0.0, 100.0, GyrX=[0.0] * 100, AccX=[0.0] * 100)
    imu2 = build_series(3.0, 100.0, GyrX=[0.0] * 100, AccX=[0.0] * 100)
    att = build_series(0.0, 50.0, Roll=[0.0] * 200, Pitch=[0.0] * 200)
    findings = TimingCheck().run(make_flight_log({"IMU": imu1 + imu2, "ATT": att}))
    gap_findings = [f for f in findings if "gap" in f.title.lower()]
    assert gap_findings, "expected a gap finding from IMU"
    assert gap_findings[0].evidence.message_types == ["IMU"]


def test_small_jitter_below_threshold_no_findings():
    # A 0.3 s gap in a 5 Hz stream: nominal dt 0.2 s, threshold = max(0.5, 2.0)=0.5;
    # 0.3 s < 0.5 s floor -> not flagged.
    seg1 = build_series(0.0, 5.0, Roll=[0.0] * 10, Pitch=[0.0] * 10)
    # seg1 ends ~1.8 s; start seg2 at 2.1 s -> gap ~0.3 s.
    seg2 = build_series(2.1, 5.0, Roll=[0.0] * 10, Pitch=[0.0] * 10)
    findings = TimingCheck().run(make_flight_log({"ATT": seg1 + seg2}))
    assert findings == []


# -- PM scheduler overruns -----------------------------------------------------


def test_pm_long_loops_emits_warn():
    pm = build_series(0.0, 1.0, NLon=[0.0, 2.0, 3.0, 0.0, 1.0], Load=[30.0] * 5)
    findings = TimingCheck().run(make_flight_log({"PM": pm}))
    pm_findings = [f for f in findings if "long loop" in f.title.lower()]
    assert pm_findings, "expected a PM long-loop finding"
    f = pm_findings[0]
    assert f.severity == Severity.WARN
    assert f.evidence.message_types == ["PM"]
    # 0 + 2 + 3 + 0 + 1 = 6 long loops.
    assert f.evidence.samples["long_loops"] == 6


def test_pm_zero_long_loops_no_finding():
    pm = build_series(0.0, 1.0, NLon=[0.0] * 5, Load=[30.0] * 5)
    findings = TimingCheck().run(make_flight_log({"PM": pm}))
    assert [f for f in findings if "long loop" in f.title.lower()] == []


def test_pm_without_nlon_field_no_finding():
    # PM present but no NLon field -> guarded, no PM finding.
    pm = build_series(0.0, 1.0, Load=[30.0] * 5)
    findings = TimingCheck().run(make_flight_log({"PM": pm}))
    assert [f for f in findings if "long loop" in f.title.lower()] == []


def test_gap_and_pm_combined():
    # Both signals present at once -> both kinds of finding emitted.
    seg1 = build_series(0.0, 50.0, Roll=[0.0] * 100, Pitch=[0.0] * 100)
    seg2 = build_series(4.0, 50.0, Roll=[0.0] * 100, Pitch=[0.0] * 100)
    pm = build_series(0.0, 1.0, NLon=[5.0, 0.0], Load=[40.0, 40.0])
    findings = TimingCheck().run(make_flight_log({"ATT": seg1 + seg2, "PM": pm}))
    titles = " ".join(f.title.lower() for f in findings)
    assert "gap" in titles
    assert "long loop" in titles


# -- registration --------------------------------------------------------------


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("timing") is not None
