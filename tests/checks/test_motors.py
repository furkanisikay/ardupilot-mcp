"""Tests for the motors check."""

from __future__ import annotations

from ardupilot_mcp.checks.motors import MotorsCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _log(rcou_records):
    return make_flight_log({"RCOU": rcou_records})


def test_skipped_when_no_rcou():
    log = make_flight_log({})
    result = MotorsCheck().execute(log)
    assert result.status == CheckStatus.SKIPPED
    assert "RCOU" in (result.skipped_reason or "")


def test_clean_balanced_no_findings():
    # All four motors balanced around 1500, no saturation -> nothing to report.
    recs = build_series(
        0.0,
        10.0,
        C1=[1500.0] * 50,
        C2=[1505.0] * 50,
        C3=[1495.0] * 50,
        C4=[1500.0] * 50,
    )
    findings = MotorsCheck().run(_log(recs))
    assert findings == []


def test_inactive_channels_ignored():
    # C1 active ~1500; C2..C4 idle at ~1000 (below ACTIVE_MIN_PWM). With only one
    # active motor there is no balance to assess and no saturation -> no findings.
    recs = build_series(
        0.0,
        10.0,
        C1=[1500.0] * 50,
        C2=[1000.0] * 50,
        C3=[1000.0] * 50,
        C4=[1000.0] * 50,
    )
    findings = MotorsCheck().run(_log(recs))
    assert findings == []


def test_warn_imbalance():
    # C1 mean ~1700 vs others ~1500 -> spread ~200 PWM (>150, <300) -> WARN.
    recs = build_series(
        0.0,
        10.0,
        C1=[1700.0] * 50,
        C2=[1500.0] * 50,
        C3=[1500.0] * 50,
        C4=[1500.0] * 50,
    )
    findings = MotorsCheck().run(_log(recs))
    imb = [f for f in findings if "imbalance" in f.title.lower()]
    assert imb, "expected an imbalance finding"
    assert imb[0].severity == Severity.WARN
    assert imb[0].evidence.samples["mean_C1"] == 1700.0
    assert 190.0 <= imb[0].evidence.samples["imbalance_pwm"] <= 210.0


def test_critical_imbalance():
    # C1 mean ~1850 vs others ~1500 -> spread ~350 PWM (>300) -> CRITICAL.
    recs = build_series(
        0.0,
        10.0,
        C1=[1850.0] * 50,
        C2=[1500.0] * 50,
        C3=[1500.0] * 50,
        C4=[1500.0] * 50,
    )
    findings = MotorsCheck().run(_log(recs))
    imb = [f for f in findings if "imbalance" in f.title.lower()]
    assert imb, "expected an imbalance finding"
    assert imb[0].severity == Severity.CRITICAL
    assert imb[0].evidence.samples["mean_C1"] == 1850.0
    assert imb[0].evidence.samples["imbalance_pwm"] >= 300.0


def test_critical_saturation_all_pinned():
    # All four motors pinned at 1990 -> 100% above 1950 -> CRITICAL saturation.
    # (They are balanced with one another, so the only findings are saturation.)
    recs = build_series(
        0.0,
        10.0,
        C1=[1990.0] * 50,
        C2=[1990.0] * 50,
        C3=[1990.0] * 50,
        C4=[1990.0] * 50,
    )
    findings = MotorsCheck().run(_log(recs))
    sat = [f for f in findings if "saturation" in f.title.lower()]
    assert sat, "expected saturation findings"
    assert all(f.severity == Severity.CRITICAL for f in sat)
    # One saturation finding per active motor.
    assert len(sat) == 4
    c1 = next(f for f in sat if "C1" in f.title)
    assert c1.evidence.samples["frac_above_1950_C1"] == 1.0
    # No imbalance finding since all means are equal.
    assert not any("imbalance" in f.title.lower() for f in findings)


def test_warn_saturation_partial():
    # C1 above 1950 for 30/50 samples (60%? no -> use 18/50=36% for WARN band).
    # 18 samples saturated -> 36% (>20%, <50%) -> WARN.
    c1 = [1990.0] * 18 + [1500.0] * 32
    recs = build_series(
        0.0,
        10.0,
        C1=c1,
        C2=[1500.0] * 50,
        C3=[1500.0] * 50,
        C4=[1500.0] * 50,
    )
    findings = MotorsCheck().run(_log(recs))
    sat = [f for f in findings if "saturation" in f.title.lower()]
    assert sat, "expected a saturation finding"
    assert sat[0].severity == Severity.WARN
    assert abs(sat[0].evidence.samples["frac_above_1950_C1"] - 0.36) < 1e-6


def test_no_saturation_below_threshold():
    # C1 saturated for only 5/50 = 10% (<20%) -> no saturation finding.
    c1 = [1990.0] * 5 + [1500.0] * 45
    recs = build_series(
        0.0,
        10.0,
        C1=c1,
        C2=[1500.0] * 50,
        C3=[1500.0] * 50,
        C4=[1500.0] * 50,
    )
    findings = MotorsCheck().run(_log(recs))
    assert not any("saturation" in f.title.lower() for f in findings)


def test_eight_motor_frame_supported():
    # An octocopter: C1..C8 all active and balanced -> no findings.
    recs = build_series(
        0.0,
        10.0,
        C1=[1500.0] * 30,
        C2=[1500.0] * 30,
        C3=[1500.0] * 30,
        C4=[1500.0] * 30,
        C5=[1500.0] * 30,
        C6=[1500.0] * 30,
        C7=[1500.0] * 30,
        C8=[1500.0] * 30,
    )
    findings = MotorsCheck().run(_log(recs))
    assert findings == []


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("motors") is not None
