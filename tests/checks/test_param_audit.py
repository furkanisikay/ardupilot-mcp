"""Tests for the parameter audit check."""

from __future__ import annotations

from ardupilot_mcp.checks.param_audit import ParamAuditCheck
from ardupilot_mcp.model import Severity
from tests.helpers import make_flight_log


def _run(params, kind="copter"):
    return ParamAuditCheck().run(make_flight_log(params=params, vehicle_kind=kind))


def _titles(findings):
    return [f.title for f in findings]


def test_empty_params_no_findings():
    assert _run({}) == []


def test_accel_limit_disabled():
    f = _run({"ATC_ACCEL_P_MAX": 0.0})
    assert any("acceleration limit disabled" in t.lower() for t in _titles(f))
    assert f[0].severity == Severity.WARN


def test_accel_limit_nonzero_ok():
    assert _run({"ATC_ACCEL_P_MAX": 110000.0}) == []


def test_rate_gain_zero_warn():
    f = _run({"ATC_RAT_RLL_P": 0.0})
    assert any("rate gain is zero" in t.lower() for t in _titles(f))
    assert f[0].severity == Severity.WARN


def test_rate_gain_low_info():
    f = _run({"ATC_RAT_RLL_P": 0.027})
    assert f and f[0].severity == Severity.INFO and "low" in f[0].title.lower()


def test_rate_gain_high_info():
    f = _run({"ATC_RAT_RLL_P": 0.6})
    assert f and f[0].severity == Severity.INFO and "high" in f[0].title.lower()


def test_rate_gain_default_ok():
    assert _run({"ATC_RAT_RLL_P": 0.135, "ATC_RAT_PIT_P": 0.135}) == []


def test_battery_threshold_contradiction():
    f = _run({"BATT_CRT_VOLT": 14.0, "BATT_LOW_VOLT": 13.5})  # crit >= low
    assert any("critical voltage not below low" in t.lower() for t in _titles(f))


def test_battery_threshold_ok_and_disabled():
    assert _run({"BATT_CRT_VOLT": 13.2, "BATT_LOW_VOLT": 14.0}) == []  # correct order
    assert _run({"BATT_CRT_VOLT": 0.0, "BATT_LOW_VOLT": 14.0}) == []  # 0 = disabled, not a contradiction


def test_mot_spin_arm_above_min():
    f = _run({"MOT_SPIN_ARM": 0.15, "MOT_SPIN_MIN": 0.10})
    assert any("spin-arm above spin-min" in t.lower() for t in _titles(f))


def test_mot_pwm_zero_is_not_flagged():
    # 0/0 means "use the RC range" — a valid config, not an inverted range.
    assert _run({"MOT_PWM_MIN": 0.0, "MOT_PWM_MAX": 0.0}) == []


def test_mot_pwm_inverted():
    f = _run({"MOT_PWM_MIN": 1950.0, "MOT_PWM_MAX": 1100.0})
    assert any("pwm range inverted" in t.lower() for t in _titles(f))


def test_notch_enabled_unconfigured():
    f = _run({"INS_HNTCH_ENABLE": 1.0, "INS_HNTCH_FREQ": 0.0})
    assert any("notch enabled but unconfigured" in t.lower() for t in _titles(f))
    assert _run({"INS_HNTCH_ENABLE": 1.0, "INS_HNTCH_FREQ": 80.0}) == []


def test_fence_enabled_no_boundary():
    f = _run({"FENCE_ENABLE": 1.0, "FENCE_ALT_MAX": 0.0, "FENCE_RADIUS": 0.0})
    assert any("geofence enabled with no boundary" in t.lower() for t in _titles(f))
    assert _run({"FENCE_ENABLE": 1.0, "FENCE_ALT_MAX": 100.0}) == []


def test_multirotor_rules_skipped_on_rover():
    # Rate/accel/motor rules are multirotor-only; a rover with those params isn't flagged for them.
    f = _run({"ATC_RAT_RLL_P": 0.0, "ATC_ACCEL_P_MAX": 0.0}, kind="rover")
    assert f == []


def test_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("param_audit") is not None
