"""Tests for the ESC power & temperature check."""

from __future__ import annotations

from ardupilot_mcp.checks.esc import EscCheck
from ardupilot_mcp.model import Severity
from tests.helpers import build_series, interleave_instances, make_flight_log

N = 300  # 30 s at 10 Hz
PARAMS = {"FRAME_CLASS": 1, "MOT_SPIN_MIN": 0.08, "MOT_PWM_MIN": 1000, "MOT_PWM_MAX": 2000}


def _flight(curr, volt, out, *, params=None, esc=None):
    bat = build_series(0.0, 10.0, Volt=volt, Curr=curr)
    rcou = build_series(0.0, 10.0, **{f"C{n}": out for n in range(1, 5)})
    msgs = {"BAT": bat, "RCOU": rcou, "EV": [{"TimeUS": 0, "Id": 10}]}
    if esc is not None:
        msgs["ESC"] = esc
    return make_flight_log(msgs, params=params or PARAMS, vehicle_kind="copter")


def test_no_battery_or_outputs_no_findings():
    assert EscCheck().run(make_flight_log({}, vehicle_kind="copter")) == []


def test_motors_lose_power_while_commanded():
    # The 132194 forum crash: 8.6 A -> 0.2 A while RCOU still asks ~1300 us, voltage holds
    # (rises once unloaded), the log keeps running.
    curr = [8.5] * 150 + [0.2] * 150
    volt = [14.9] * 150 + [15.1] * 150
    findings = EscCheck().run(_flight(curr, volt, [1300] * N))
    assert [(f.title, f.severity) for f in findings] == [
        ("Motors lost power while still commanded", Severity.CRITICAL)
    ]
    assert findings[0].evidence.time_start_s == 15.0
    assert findings[0].evidence.samples["volt_after"] == 15.1


def test_landing_with_motors_at_idle_is_not_a_power_cut():
    curr = [8.5] * 150 + [0.2] * 150
    out = [1400] * 150 + [1050] * 150  # outputs at spin-min after touchdown
    assert EscCheck().run(_flight(curr, [14.9] * N, out)) == []


def test_battery_disconnect_is_left_to_the_power_check():
    curr = [8.5] * 150 + [0.0] * 150
    volt = [14.9] * 150 + [0.5] * 150  # the pack itself is gone
    assert EscCheck().run(_flight(curr, volt, [1300] * N)) == []


def test_a_single_low_current_sample_is_a_glitch():
    curr = [8.5] * 150 + [0.2] * 3 + [8.5] * 147
    assert EscCheck().run(_flight(curr, [14.9] * N, [1300] * N)) == []


def _esc(temps_by_inst):
    return interleave_instances(
        "Instance",
        {
            i: build_series(0.0, 10.0, Temp=t, RPM=[0.0] + [5000.0] * (len(t) - 1))
            for i, t in temps_by_inst.items()
        },
    )


def _temp_log(temps_by_inst):
    msgs = {"ESC": _esc(temps_by_inst), "EV": [{"TimeUS": 0, "Id": 10}]}
    return make_flight_log(msgs, params=PARAMS, vehicle_kind="copter")


def test_hot_escs_within_the_protection_range_are_noted():
    # Heated from 30 to 110 degC: inside BLHeli_S's 80-140 degC protection-threshold range.
    temps = [30.0] * 100 + [110.0] * 200
    f = [x for x in EscCheck().run(_temp_log({0: temps, 1: [30.0] * N})) if x.title == "Hot ESCs"]
    assert [x.severity for x in f] == [Severity.INFO]
    assert f[0].evidence.samples["hottest_held_temp_c"] == 110.0


def test_escs_above_every_threshold_warn():
    temps = [30.0] * 100 + [150.0] * 200
    f = [x for x in EscCheck().run(_temp_log({0: temps})) if x.title == "Hot ESCs"]
    assert [x.severity for x in f] == [Severity.WARN]


def test_a_brief_temperature_spike_is_ignored():
    temps = [30.0] * 150 + [150.0] * 10 + [30.0] * 140  # 1 s
    assert EscCheck().run(_temp_log({0: temps})) == []


def test_temperature_already_high_with_the_motor_stopped_is_not_judged():
    # One log reads 140-160 degC from power-on and never rises with load.
    findings = EscCheck().run(_temp_log({0: [159.0] * N}))
    assert [f.title for f in findings] == ["ESC temperature telemetry not judged"]
    assert findings[0].severity == Severity.INFO


def _quadplane(curr, volt, out, params=None):
    p = {
        "Q_ENABLE": 1,
        "Q_M_SPIN_MIN": 0.15,
        "Q_M_PWM_MIN": 1000,
        "Q_M_PWM_MAX": 2000,
        **{f"SERVO{n}_FUNCTION": 33 + n - 9 for n in range(9, 13)},
        **(params or {}),
    }
    bat = build_series(0.0, 10.0, Volt=volt, Curr=curr)
    rcou = build_series(0.0, 10.0, **{f"C{n}": out for n in range(9, 13)})
    return make_flight_log(
        {"BAT": bat, "RCOU": rcou, "EV": [{"TimeUS": 0, "Id": 10}]}, params=p, vehicle_kind="plane"
    )


def test_quadplane_vtol_motors_losing_power():
    # Forum log 142142_0: 46 A -> 0 A while the VTOL motors (SERVO9-12) were driven at 1800+ us.
    findings = EscCheck().run(_quadplane([46.0] * 150 + [0.0] * 150, [22.9] * 150 + [24.2] * 150, [1850] * N))
    assert [(f.title, f.severity) for f in findings] == [
        ("Motors lost power while still commanded", Severity.CRITICAL)
    ]


def test_plane_without_vtol_motors_is_not_judged_for_a_power_cut():
    findings = EscCheck().run(_quadplane([46.0] * 150 + [0.0] * 150, [22.9] * N, [1850] * N, {"Q_ENABLE": 0}))
    assert findings == []
