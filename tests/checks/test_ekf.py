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
    # SV ramps up to 1.3 and stays above 1.0 for 1.5 s -> the velocity sensor was being
    # rejected for longer than the firmware's EKF failsafe waits -> CRITICAL.
    sv = [0.2] * 20 + [round(0.2 + i * (1.1 / 14), 3) for i in range(15)] + [1.3] * 15
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


def test_brief_rejection_is_a_warning():
    # SV above 1.0 for 0.3 s: a rejection shorter than the 1 s the firmware's EKF failsafe
    # waits for (EKF_CHECK_ITERATIONS_MAX, ekf_check.cpp) is a warning, not critical.
    sv = [0.2] * 47 + [1.6] * 3
    recs = build_series(0.0, 10.0, SV=sv, SP=[0.2] * 50, SH=[0.2] * 50, SM=[0.2] * 50)
    vel = [f for f in EKFCheck().run(_xkf4(recs)) if "velocity" in f.title]
    assert [f.severity for f in vel] == [Severity.WARN]
    assert "brief" in vel[0].explanation


def test_rejection_on_the_ground_is_ignored():
    # SH far above 1.0 before arming (the vehicle on the bench) is not a flight problem.
    sh = [3.0] * 30 + [0.2] * 70
    recs = build_series(0.0, 10.0, SV=[0.2] * 100, SP=[0.2] * 100, SH=sh, SM=[0.2] * 100)
    log = make_flight_log({"XKF4": recs, "EV": [{"TimeUS": 4_000_000, "Id": 10}]})
    assert [f for f in EKFCheck().run(log) if "height" in f.title] == []


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


def test_height_critical_and_mag_alone_warning():
    # Height and compass over 1.0: height is critical; the compass, rejected while velocity and
    # position stay trusted, is a warning (the EKF check never acts on it alone).
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
    assert mag and mag[0].severity == Severity.WARN
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


def test_compass_rejected_alone_is_a_warning():
    # Benchmark logs 144427_0, 128873_2: SM over 1 for seconds while SV/SP stay low; the
    # firmware's EKF check never acts on the compass alone (ekf_over_threshold).
    n = 50
    recs = build_series(
        0.0, 10.0, SV=[0.2] * n, SP=[0.2] * n, SH=[0.1] * n, SM=[0.2] * 10 + [1.6] * 30 + [0.2] * 10
    )
    f = [x for x in EKFCheck().run(_xkf4(recs)) if "SM" in x.title]
    assert [x.severity for x in f] == [Severity.WARN]
    assert "never acts on the compass alone" in f[0].explanation


def test_compass_rejected_with_velocity_stays_critical():
    n = 50
    sm = [0.2] * 10 + [1.6] * 30 + [0.2] * 10
    sv = [0.2] * 10 + [0.9] * 30 + [0.2] * 10  # over FS_EKF_THRESH 0.8 at the same time
    recs = build_series(0.0, 10.0, SV=sv, SP=[0.2] * n, SH=[0.1] * n, SM=sm)
    f = [x for x in EKFCheck().run(_xkf4(recs)) if "SM" in x.title]
    assert [x.severity for x in f] == [Severity.CRITICAL]
