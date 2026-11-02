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


def test_helicopter_default_rate_gain_is_not_flagged():
    # Heli's default ATC_RAT_RLL_P is 0.024 (AC_AttitudeControl_Heli.h); it flies on FF.
    log = make_flight_log(params={"ATC_RAT_RLL_P": 0.024, "ATC_RAT_PIT_P": 0.024}, vehicle_kind="heli")
    assert not [f for f in ParamAuditCheck().run(log) if "rate gain" in f.title]


def _bat_log(volts, params):
    from tests.helpers import build_series

    bat = build_series(0.0, 10.0, Volt=volts, VoltR=volts, Curr=[10.0] * len(volts))
    return make_flight_log(
        {"BAT": bat, "EV": [{"TimeUS": 0, "Id": 10}]}, params=params, vehicle_kind="copter"
    )


def test_voltage_compensation_set_for_a_smaller_pack_never_acts():
    # A 12S pack (~47 -> 39 V) with MOT_BAT_VOLT_MAX left at 6S (25.2 V): the firmware clamps
    # the voltage to 25.2 V (Thrust_Linearization), so compensation never acts.
    volts = [47.0 - i * 0.02 for i in range(400)]
    log = _bat_log(volts, {"MOT_BAT_VOLT_MAX": 25.2, "MOT_BAT_VOLT_MIN": 22.2})
    f = [x for x in ParamAuditCheck().run(log) if "Voltage compensation" in x.title]
    assert [x.severity for x in f] == [Severity.WARN] and "never active" in f[0].title


def test_voltage_compensation_set_for_a_bigger_pack_is_pinned():
    # A 6S pack with MOT_BAT_VOLT_* for 12S: clamped at the floor, gains raised all flight.
    volts = [25.0 - i * 0.01 for i in range(400)]
    log = _bat_log(volts, {"MOT_BAT_VOLT_MAX": 50.4, "MOT_BAT_VOLT_MIN": 39.6})
    f = [x for x in ParamAuditCheck().run(log) if "Voltage compensation" in x.title]
    assert len(f) == 1 and "pinned" in f[0].title


def test_voltage_compensation_matching_the_pack_is_fine():
    volts = [25.0 - i * 0.01 for i in range(400)]
    log = _bat_log(volts, {"MOT_BAT_VOLT_MAX": 25.2, "MOT_BAT_VOLT_MIN": 19.8})
    assert not [x for x in ParamAuditCheck().run(log) if "Voltage compensation" in x.title]


def _plane_log(aspd, params):
    from tests.helpers import build_series

    ctun = build_series(0.0, 10.0, As=aspd)
    stat = build_series(0.0, 10.0, isFlying=[1] * len(aspd))
    return make_flight_log(
        {"CTUN": ctun, "STAT": stat, "EV": [{"TimeUS": 0, "Id": 10}]},
        params=params,
        vehicle_type="ArduPlane",
        vehicle_kind="plane",
    )


def test_plane_flying_far_above_airspeed_max():
    # A wing at ~40 m/s with AIRSPEED_MAX 28 and SCALING_SPEED 15 (a real forum crash):
    # the surfaces get SCALING_SPEED / airspeed of their deflection (calc_speed_scaler).
    params = {"AIRSPEED_MAX": 28.0, "AIRSPEED_MIN": 12.0, "SCALING_SPEED": 15.0}
    log = _plane_log([20.0] * 100 + [40.0] * 200, params)
    f = [x for x in ParamAuditCheck().run(log) if x.title.startswith("Flew far above")]
    assert [x.severity for x in f] == [Severity.WARN]
    assert f[0].evidence.samples["median_speed_scaler"] == 0.38  # 15 / 40, above the 0.27 floor


def test_plane_inside_its_airspeed_range_is_fine():
    params = {"AIRSPEED_MAX": 32.0, "AIRSPEED_MIN": 12.0, "SCALING_SPEED": 24.0}
    log = _plane_log([24.0] * 290 + [33.0] * 10, params)
    assert not [x for x in ParamAuditCheck().run(log) if x.title.startswith("Flew far above")]


def test_plane_airspeed_max_below_min():
    log = _plane_log([0.0] * 50, {"AIRSPEED_MAX": 8.0, "AIRSPEED_MIN": 9.0})
    f = [x for x in ParamAuditCheck().run(log) if x.title == "Airspeed maximum not above the minimum"]
    assert [x.severity for x in f] == [Severity.WARN]


def test_pre_4_5_airspeed_parameter_names():
    params = {"ARSPD_FBW_MAX": 22.0, "ARSPD_FBW_MIN": 9.0, "SCALING_SPEED": 13.0}
    log = _plane_log([28.0] * 300, params)
    f = [x for x in ParamAuditCheck().run(log) if x.title.startswith("Flew far above")]
    assert f and f[0].title == "Flew far above ARSPD_FBW_MAX"


def test_gps_and_rangefinder_height_sources_are_noted():
    for src in (2, 3):
        log = make_flight_log(params={"EK3_SRC1_POSZ": src, "AHRS_EKF_TYPE": 3}, vehicle_kind="copter")
        f = [x for x in ParamAuditCheck().run(log) if x.title == "EKF height source is not the barometer"]
        assert [x.severity for x in f] == [Severity.INFO], src
    baro = make_flight_log(params={"EK3_SRC1_POSZ": 1}, vehicle_kind="copter")
    assert not [x for x in ParamAuditCheck().run(baro) if "height source" in x.title]


def test_pinned_compensation_gain_follows_the_firmware_formula():
    # 4S pack with 6S settings: r = 19.8 / 25.2, expo 0.65 -> gain 1 / (r*0.35 + 0.65*r^2) = 1.48.
    volts = [16.4 - i * 0.005 for i in range(400)]
    log = _bat_log(volts, {"MOT_BAT_VOLT_MAX": 25.2, "MOT_BAT_VOLT_MIN": 19.8})
    f = [x for x in ParamAuditCheck().run(log) if "pinned" in x.title]
    assert f and "1.48x" in f[0].explanation
