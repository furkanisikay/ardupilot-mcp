"""Tests for the EKF / estimator health check."""

from __future__ import annotations

from ardupilot_mcp.checks.ekf import EKFCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _xkf4(records):
    return make_flight_log({"XKF4": records})


def _nkf4(records):
    return make_flight_log({"NKF4": records})


def test_no_ekf_message_returns_empty():
    # Neither XKF4 nor NKF4 present: the check runs but produces nothing.
    log = make_flight_log({"VIBE": build_series(0.0, 10.0, VibeX=[1.0] * 10)})
    result = EKFCheck().execute(log)
    assert result.status == CheckStatus.RAN
    assert result.findings == []


def test_run_returns_empty_when_no_ekf():
    assert EKFCheck().run(make_flight_log({})) == []


def test_clean_flight_no_findings():
    # All variance ratios comfortably low -> no findings.
    recs = build_series(
        0.0,
        5.0,
        SV=[0.2] * 50,
        SP=[0.15] * 50,
        SH=[0.25] * 50,
        SM=[0.3] * 50,
        SVT=[0.1] * 50,
    )
    assert EKFCheck().run(_xkf4(recs)) == []


def test_critical_velocity_variance_xkf4():
    # SV ramps up to 1.3 -> the velocity sensor was being rejected -> CRITICAL.
    sv = [0.2] * 30 + [round(0.2 + i * (1.1 / 19), 3) for i in range(20)]  # ends ~1.3
    recs = build_series(
        0.0,
        10.0,
        SV=sv,
        SP=[0.2] * 50,
        SH=[0.2] * 50,
        SM=[0.2] * 50,
        SVT=[0.2] * 50,
    )
    findings = EKFCheck().run(_xkf4(recs))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "velocity" in f.title]
    assert crit, "expected a critical EKF velocity finding"
    assert crit[0].evidence.samples["max_SV"] > 1.0
    assert crit[0].evidence.message_types == ["XKF4"]
    # Timing window is populated from when the ratio first crossed the warn line.
    assert crit[0].evidence.time_start_s is not None
    assert crit[0].evidence.time_end_s is not None


def test_warn_position_variance():
    # SP peaks at 0.9: above WARN (0.8) but never crosses the 1.0 rejection line.
    sp = [0.3] * 40 + [0.9] * 10
    recs = build_series(
        0.0,
        10.0,
        SV=[0.2] * 50,
        SP=sp,
        SH=[0.2] * 50,
        SM=[0.2] * 50,
        SVT=[0.2] * 50,
    )
    findings = EKFCheck().run(_xkf4(recs))
    pos = [f for f in findings if "position" in f.title]
    assert pos, "expected a position finding"
    assert pos[0].severity == Severity.WARN
    assert pos[0].evidence.samples["max_SP"] == 0.9
    # No critical anywhere in this flight.
    assert not any(f.severity == Severity.CRITICAL for f in findings)


def test_height_and_mag_critical():
    # Two independent sensors over 1.0 -> two critical findings.
    recs = build_series(
        0.0,
        5.0,
        SV=[0.2] * 20,
        SP=[0.2] * 20,
        SH=[0.3] * 15 + [1.4] * 5,
        SM=[0.3] * 15 + [1.2] * 5,
        SVT=[0.2] * 20,
    )
    findings = EKFCheck().run(_xkf4(recs))
    height = [f for f in findings if "height" in f.title]
    mag = [f for f in findings if "magnetometer" in f.title]
    assert height and height[0].severity == Severity.CRITICAL
    assert mag and mag[0].severity == Severity.CRITICAL
    assert height[0].evidence.samples["max_SH"] == 1.4
    assert mag[0].evidence.samples["max_SM"] == 1.2


def test_nkf4_fallback_used_when_no_xkf4():
    # Legacy EKF2 log: only NKF4 present, velocity (SV) test ratio over 1.0.
    # (SVT is intentionally not analysed — in NKF4 it is a tilt-error metric, not
    # a normalised innovation test ratio.)
    recs = build_series(
        0.0,
        5.0,
        SV=[0.3] * 15 + [1.5] * 5,
        SP=[0.2] * 20,
        SH=[0.2] * 20,
        SM=[0.2] * 20,
    )
    findings = EKFCheck().run(_nkf4(recs))
    vel = [f for f in findings if "velocity" in f.title]
    assert vel, "expected a velocity finding from NKF4"
    assert vel[0].severity == Severity.CRITICAL
    assert vel[0].evidence.message_types == ["NKF4"]
    assert vel[0].evidence.samples["max_SV"] == 1.5


def test_xkf4_preferred_over_nkf4():
    # When both exist, XKF4 wins and NKF4 is ignored.
    xkf = build_series(0.0, 5.0, SV=[0.2] * 20)  # clean XKF4
    nkf = build_series(0.0, 5.0, SV=[1.9] * 20)  # noisy NKF4 (should be ignored)
    log = make_flight_log({"XKF4": xkf, "NKF4": nkf})
    findings = EKFCheck().run(log)
    assert findings == [], "XKF4 (clean) must take precedence over NKF4"


def test_missing_fields_are_skipped():
    # Only SV logged; other sensor fields absent -> no spurious findings for them.
    recs = build_series(0.0, 10.0, SV=[0.1] * 30)
    assert EKFCheck().run(_xkf4(recs)) == []


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("ekf") is not None
