"""Tests for the GPS quality check."""

from __future__ import annotations

from ardupilot_mcp.checks.gps import GpsCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _log(gps_records):
    return make_flight_log({"GPS": gps_records})


def _clean_records(n=50):
    """A healthy GPS series: solid 3D fix, plenty of sats, low HDOP."""
    return build_series(
        0.0,
        5.0,
        Status=[3] * n,
        NSats=[12] * n,
        HDop=[0.8] * n,
    )


def test_skipped_when_no_gps():
    log = make_flight_log({})
    result = GpsCheck().execute(log)
    assert result.status == CheckStatus.SKIPPED
    assert "GPS" in (result.skipped_reason or "")


def test_clean_flight_no_findings():
    findings = GpsCheck().run(_log(_clean_records()))
    assert findings == []


def test_critical_fix_loss_mid_flight():
    # Status goes 3 -> 1 mid-log (fix achieved, then lost).
    status = [3] * 20 + [1] * 10 + [3] * 20
    recs = build_series(
        0.0,
        5.0,
        Status=status,
        NSats=[12] * 50,
        HDop=[0.8] * 50,
    )
    findings = GpsCheck().run(_log(recs))
    fix = [f for f in findings if "lost 3d fix" in f.title.lower()]
    assert fix, "expected a fix-loss finding"
    assert fix[0].severity == Severity.CRITICAL
    assert fix[0].evidence.samples["min_Status_after_fix"] == 1
    assert fix[0].evidence.samples["loss_windows"] == 1
    # The loss window should start after the first-fix block (t = 20/5 = 4.0s).
    assert fix[0].evidence.time_start_s is not None
    assert fix[0].evidence.time_start_s >= 4.0


def test_warn_never_achieved_3d_fix():
    # Status never reaches 3.
    recs = build_series(
        0.0,
        5.0,
        Status=[1] * 20 + [2] * 20,
        NSats=[8] * 40,
        HDop=[1.0] * 40,
    )
    findings = GpsCheck().run(_log(recs))
    fix = [f for f in findings if "never achieved" in f.title.lower()]
    assert fix, "expected a never-3D-fix finding"
    assert fix[0].severity == Severity.WARN
    assert fix[0].evidence.samples["max_Status"] == 2


def test_no_fix_finding_when_fix_held():
    # Status starts at 2 then climbs to 3 and stays — fix acquired, never lost.
    status = [2] * 5 + [3] * 45
    recs = build_series(
        0.0,
        5.0,
        Status=status,
        NSats=[11] * 50,
        HDop=[0.9] * 50,
    )
    findings = GpsCheck().run(_log(recs))
    assert not [f for f in findings if "fix" in f.title.lower()]


def test_warn_low_satellites():
    # Min NSats = 5 -> WARN (below 6, at/above 4).
    nsats = [12] * 30 + [5] * 5 + [12] * 15
    recs = build_series(
        0.0,
        5.0,
        Status=[3] * 50,
        NSats=nsats,
        HDop=[0.8] * 50,
    )
    findings = GpsCheck().run(_log(recs))
    sats = [f for f in findings if "satellite" in f.title.lower()]
    assert sats, "expected a low-satellite finding"
    assert sats[0].severity == Severity.WARN
    assert sats[0].evidence.samples["min_NSats"] == 5


def test_critical_low_satellites():
    # Min NSats = 3 -> CRITICAL (below 4).
    nsats = [12] * 30 + [3] * 5 + [12] * 15
    recs = build_series(
        0.0,
        5.0,
        Status=[3] * 50,
        NSats=nsats,
        HDop=[0.8] * 50,
    )
    findings = GpsCheck().run(_log(recs))
    sats = [f for f in findings if "satellite" in f.title.lower()]
    assert sats, "expected a low-satellite finding"
    assert sats[0].severity == Severity.CRITICAL
    assert sats[0].evidence.samples["min_NSats"] == 3


def test_warn_high_hdop():
    # HDop peaks at 3.0 -> WARN (above 2.0, at/below 5.0).
    hdop = [0.8] * 40 + [3.0] * 5 + [0.8] * 5
    recs = build_series(
        0.0,
        5.0,
        Status=[3] * 50,
        NSats=[12] * 50,
        HDop=hdop,
    )
    findings = GpsCheck().run(_log(recs))
    hd = [f for f in findings if "hdop" in f.title.lower()]
    assert hd, "expected an HDOP finding"
    assert hd[0].severity == Severity.WARN
    assert hd[0].evidence.samples["max_HDop"] == 3.0


def test_critical_high_hdop():
    # HDop peaks at 7.5 -> CRITICAL (above 5.0).
    hdop = [0.8] * 40 + [7.5] * 5 + [0.8] * 5
    recs = build_series(
        0.0,
        5.0,
        Status=[3] * 50,
        NSats=[12] * 50,
        HDop=hdop,
    )
    findings = GpsCheck().run(_log(recs))
    hd = [f for f in findings if "hdop" in f.title.lower()]
    assert hd, "expected an HDOP finding"
    assert hd[0].severity == Severity.CRITICAL
    assert hd[0].evidence.samples["max_HDop"] == 7.5


def test_one_finding_per_concern():
    # All three concerns present: a mid-flight fix loss + low sats during it,
    # and (separately) high HDOP while a valid 3D fix is held at the end — HDOP
    # only counts during a fix (no-fix HDOP is a sentinel, not dilution).
    status = [3] * 20 + [1] * 10 + [3] * 20
    nsats = [12] * 20 + [3] * 10 + [12] * 20
    hdop = [0.8] * 40 + [6.0] * 10  # high HDOP at the end, while Status == 3
    recs = build_series(
        0.0,
        5.0,
        Status=status,
        NSats=nsats,
        HDop=hdop,
    )
    findings = GpsCheck().run(_log(recs))
    # Exactly three findings: one fix, one sats, one hdop.
    assert len(findings) == 3
    titles = " | ".join(f.title.lower() for f in findings)
    assert "fix" in titles
    assert "satellite" in titles
    assert "hdop" in titles
    assert all(f.severity == Severity.CRITICAL for f in findings)


def test_pre_arm_acquisition_ignored():
    # Sats climb 0 -> 12 during pre-arm acquisition, then stay healthy after the
    # ARMED event. The pre-arm zeros must NOT be flagged as an in-flight failure.
    nsats = [0] * 20 + [12] * 30  # 0..2s = pre-arm, 2..5s = armed & healthy
    status = [1] * 20 + [3] * 30
    recs = build_series(0.0, 10.0, Status=status, NSats=nsats, HDop=[0.8] * 50)
    ev = [{"TimeUS": 2_000_000, "Id": 10}]  # ARMED at t=2s
    log = make_flight_log({"GPS": recs, "EV": ev})
    findings = GpsCheck().run(log)
    assert findings == [], f"pre-arm acquisition should be ignored, got {[f.title for f in findings]}"


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("gps") is not None
