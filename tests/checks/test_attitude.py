"""Tests for the attitude tracking check."""

from __future__ import annotations

from ardupilot_mcp.checks.attitude import AttitudeTrackingCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _log(att_records, *, rcou_records=None):
    messages = {"ATT": att_records}
    if rcou_records is not None:
        messages["RCOU"] = rcou_records
    return make_flight_log(messages)


def test_skipped_when_no_att():
    log = make_flight_log({})
    result = AttitudeTrackingCheck().execute(log)
    assert result.status == CheckStatus.SKIPPED
    assert "ATT" in (result.skipped_reason or "")


def test_clean_flight_no_findings():
    # Perfect tracking: actual == desired on every axis -> no findings.
    n = 100
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[5.0] * n,
        Roll=[5.0] * n,
        DesPitch=[-3.0] * n,
        Pitch=[-3.0] * n,
        DesYaw=[90.0] * n,
        Yaw=[90.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    assert findings == []


def test_small_tracking_error_no_findings():
    # A few degrees of lag everywhere is normal -> nothing flagged.
    n = 100
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[10.0] * n,
        Roll=[12.0] * n,  # 2 deg
        DesPitch=[0.0] * n,
        Pitch=[5.0] * n,  # 5 deg
        DesYaw=[0.0] * n,
        Yaw=[8.0] * n,  # 8 deg
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    assert findings == []


def test_critical_sustained_roll_divergence():
    # Roll diverges from DesRoll by ~35 deg for 2 s (20 samples @ 10 Hz),
    # then recovers -> CRITICAL roll finding.
    n = 50
    des = [0.0] * n
    roll = [0.0] * 10 + [35.0] * 20 + [0.0] * 20  # 2 s at 35 deg error
    recs = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "roll" in f.title.lower()]
    assert crit, "expected a critical roll finding"
    f = crit[0]
    assert f.evidence.samples["peak_err_deg_roll"] >= 25
    assert f.evidence.samples["worst_duration_s_roll"] >= 1.0
    # Worst window should sit inside the divergence (1.0 s .. 2.9 s region).
    assert f.evidence.time_start_s is not None
    assert f.evidence.time_start_s >= 1.0


def test_warn_brief_spike():
    # 35 deg error but for only 0.3 s (3 samples @ 10 Hz) -> WARN, not CRITICAL.
    n = 50
    des = [0.0] * n
    roll = [0.0] * 20 + [35.0] * 3 + [0.0] * 27
    recs = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    roll_findings = [f for f in findings if "roll" in f.title.lower()]
    assert roll_findings, "expected a roll finding"
    assert roll_findings[0].severity == Severity.WARN
    assert not any(f.severity == Severity.CRITICAL for f in findings)
    assert roll_findings[0].evidence.samples["peak_err_deg_roll"] >= 30


def test_warn_sustained_moderate_band():
    # ~20 deg sustained error for 2 s -> WARN (15-25 band), not CRITICAL.
    n = 50
    des = [0.0] * n
    pitch = [0.0] * 10 + [20.0] * 20 + [0.0] * 20
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=[0.0] * n,
        DesPitch=des,
        Pitch=pitch,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    pitch_findings = [f for f in findings if "pitch" in f.title.lower()]
    assert pitch_findings, "expected a pitch finding"
    assert pitch_findings[0].severity == Severity.WARN
    assert not any(f.severity == Severity.CRITICAL for f in findings)
    assert 15 <= pitch_findings[0].evidence.samples["peak_err_deg_pitch"] <= 25


def test_yaw_error_is_wrapped():
    # Desired yaw 350, actual yaw 5 -> naive diff is -345 but wrapped is +15,
    # which is small -> NO finding. Confirms circular wrapping works.
    n = 100
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=[0.0] * n,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[350.0] * n,
        Yaw=[5.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    assert findings == [], "wrapped yaw error of 15 deg must not trigger a finding"


def test_yaw_wrapped_large_error_critical():
    # Desired 0, actual 200 -> wrapped error is -160 (abs 160), sustained 2 s.
    n = 50
    des = [0.0] * n
    yaw = [0.0] * 10 + [200.0] * 20 + [0.0] * 20
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=[0.0] * n,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=des,
        Yaw=yaw,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "yaw" in f.title.lower()]
    assert crit, "expected a critical yaw finding"
    # 200 wrapped -> 160, not 200.
    assert crit[0].evidence.samples["peak_err_deg_yaw"] == 160.0


def test_motor_saturation_enrichment():
    # Sustained critical roll divergence WITH saturated motors -> the explanation
    # mentions underpowered, and the sample flag is set.
    n = 50
    des = [0.0] * n
    roll = [0.0] * 10 + [35.0] * 20 + [0.0] * 20
    att = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    rcou = build_series(
        0.0,
        10.0,
        C1=[1980.0] * n,
        C2=[1975.0] * n,
        C3=[1990.0] * n,
        C4=[1985.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(att, rcou_records=rcou))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "roll" in f.title.lower()]
    assert crit, "expected a critical roll finding"
    f = crit[0]
    assert f.evidence.samples.get("motors_saturated") == 1.0
    # The enrichment adds a distinct RCOU sentence on top of the base message.
    assert "rcou" in f.explanation.lower()
    assert "saturated" in f.explanation.lower()
    assert "RCOU" in f.evidence.message_types


def test_no_motor_saturation_no_enrichment():
    # Same divergence but motors well below saturation -> no enrichment.
    n = 50
    des = [0.0] * n
    roll = [0.0] * 10 + [35.0] * 20 + [0.0] * 20
    att = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    rcou = build_series(
        0.0,
        10.0,
        C1=[1500.0] * n,
        C2=[1500.0] * n,
        C3=[1500.0] * n,
        C4=[1500.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(att, rcou_records=rcou))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "roll" in f.title.lower()]
    assert crit
    assert "motors_saturated" not in crit[0].evidence.samples
    # No RCOU enrichment sentence when motors are not saturated.
    assert "rcou" not in crit[0].explanation.lower()
    assert "RCOU" not in crit[0].evidence.message_types


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("attitude") is not None


def test_plane_yaw_is_not_judged():
    # ArduPlane logs DesYaw as a 0 placeholder, so heading != 0 is not an error.
    n = 100
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=[0.0] * n,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[120.0] * n,
    )
    log = make_flight_log({"ATT": recs}, vehicle_type="ArduPlane", vehicle_kind="plane")
    assert AttitudeTrackingCheck().run(log) == []


def test_ground_time_after_landing_is_not_judged():
    # Upside down on the ground after LAND_COMPLETE: not an in-flight tracking failure.
    n = 300
    roll = [0.0] * 200 + [170.0] * 100
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    ev = [{"TimeUS": 1_000_000, "Id": 10}, {"TimeUS": 3_000_000, "Id": 28}, {"TimeUS": 19_500_000, "Id": 18}]
    log = make_flight_log({"ATT": recs, "EV": ev}, vehicle_kind="copter")
    assert AttitudeTrackingCheck().run(log) == []


def test_in_flight_divergence_still_critical_with_land_detector():
    n = 300
    roll = [0.0] * 100 + [40.0] * 30 + [0.0] * 170
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    ev = [{"TimeUS": 1_000_000, "Id": 10}, {"TimeUS": 3_000_000, "Id": 28}, {"TimeUS": 25_000_000, "Id": 18}]
    log = make_flight_log({"ATT": recs, "EV": ev}, vehicle_kind="copter")
    f = AttitudeTrackingCheck().run(log)
    assert [x.severity for x in f] == [Severity.CRITICAL]


def test_a_missing_demand_sample_does_not_shift_demand_against_actual():
    # Roll tracks a square-wave demand exactly; one NaN DesRoll used to offset the
    # two series by a sample and turn the steps into a tracking error.
    n = 600
    roll = [30.0 if (i // 5) % 2 == 0 else -30.0 for i in range(n)]
    des = list(roll)
    des[40] = float("nan")
    recs = build_series(
        0.0,
        50.0,
        Roll=roll,
        DesRoll=des,
        Pitch=[0.0] * n,
        DesPitch=[0.0] * n,
        Yaw=[0.0] * n,
        DesYaw=[0.0] * n,
    )
    assert AttitudeTrackingCheck().run(_log(recs)) == []


def test_motor_saturation_is_judged_in_the_divergence_window_only():
    n = 1000  # 20 s at 50 Hz; roll diverges 30 deg from 12 s on
    roll = [0.0] * 600 + [30.0] * 400
    att = build_series(
        0.0,
        50.0,
        Roll=roll,
        DesRoll=[0.0] * n,
        Pitch=[0.0] * n,
        DesPitch=[0.0] * n,
        Yaw=[0.0] * n,
        DesYaw=[0.0] * n,
    )
    # Motors at 1500 for 12 s, pinned at 2000 for the last 8 s: the whole-flight mean is
    # below saturation, the window's mean above it.
    c1 = [1500.0] * 600 + [2000.0] * 400
    rcou = build_series(0.0, 50.0, C1=c1, C2=c1, C3=c1, C4=c1)
    findings = AttitudeTrackingCheck().run(_log(att, rcou_records=rcou))
    crit = [f for f in findings if f.severity is Severity.CRITICAL]
    assert crit and crit[0].evidence.samples.get("motors_saturated") == 1.0


def test_a_one_sample_spike_still_sees_the_motor_outputs_around_it():
    n = 500  # 10 s at 50 Hz, a single 40 deg roll sample at 5 s
    roll = [0.0] * n
    roll[250] = 40.0
    att = build_series(
        0.0,
        50.0,
        Roll=roll,
        DesRoll=[0.0] * n,
        Pitch=[0.0] * n,
        DesPitch=[0.0] * n,
        Yaw=[0.0] * n,
        DesYaw=[0.0] * n,
    )
    c1 = [2000.0] * 100  # 10 Hz, every motor pinned
    rcou = build_series(0.03, 10.0, C1=c1, C2=c1, C3=c1, C4=c1)
    (finding,) = AttitudeTrackingCheck().run(_log(att, rcou_records=rcou))
    assert finding.evidence.samples.get("motors_saturated") == 1.0


def _plane_banked_turn(mode_num: int):
    # 5 s banked at 40 deg while ATT.DesRoll stays 0.
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * 100,
        Roll=[0.0] * 30 + [40.0] * 50 + [0.0] * 20,
        DesPitch=[0.0] * 100,
        Pitch=[0.0] * 100,
        DesYaw=[0.0] * 100,
        Yaw=[0.0] * 100,
    )
    mode = [{"TimeUS": 0, "Mode": mode_num, "ModeNum": mode_num, "Rsn": 1}]
    return make_flight_log({"ATT": recs, "MODE": mode}, vehicle_type="ArduPlane", vehicle_kind="plane")


def test_plane_stabilize_banked_turn_is_not_a_tracking_failure():
    # ArduPlane STABILIZE writes nav_roll_cd = 0 while the pilot banks with stick mixing
    # (mode_stabilize.cpp): DesRoll is no target, so the bank is no error.
    for mode in (0, 2, 3, 4):  # MANUAL, STABILIZE, TRAINING, ACRO
        assert AttitudeTrackingCheck().run(_plane_banked_turn(mode)) == [], mode


def test_plane_fbwa_banked_against_zero_target_is_a_failure():
    # FBWA does command the bank angle, so the same divergence is a tracking failure.
    findings = AttitudeTrackingCheck().run(_plane_banked_turn(5))
    assert [f.severity for f in findings] == [Severity.CRITICAL]


def _copter(n, **series):
    fields = {"DesRoll": 0.0, "Roll": 0.0, "DesPitch": 0.0, "Pitch": 0.0, "DesYaw": 0.0, "Yaw": 0.0}
    cols = {k: series.get(k, [v] * n) for k, v in fields.items()}
    return make_flight_log({"ATT": build_series(0.0, 10.0, **cols)}, vehicle_kind="copter")


def test_copter_lean_error_over_30_deg_for_1_s_is_critical_and_named_by_axis():
    # Copter's parachute check: 30 deg off the target for 1 s is a loss of control.
    n = 50
    f = AttitudeTrackingCheck().run(_copter(n, Roll=[0.0] * 10 + [35.0] * 10 + [0.0] * 30))
    assert [(x.severity, x.title) for x in f] == [(Severity.CRITICAL, "Attitude tracking failure (roll)")]
    assert f[0].evidence.samples["peak_err_deg_lean"] == 35.0
    assert f[0].evidence.samples["peak_err_deg_roll"] == 35.0
    assert "loss-of-control" in f[0].explanation


def test_copter_lean_error_under_30_deg_is_a_warning():
    # 28 deg for 3 s is over the old 25 deg line but under the firmware's 30 deg.
    n = 60
    f = AttitudeTrackingCheck().run(_copter(n, Pitch=[0.0] * 10 + [28.0] * 30 + [0.0] * 20))
    assert [(x.severity, x.title) for x in f] == [
        (Severity.WARN, "Sustained attitude tracking error (pitch)")
    ]


def test_copter_roll_and_pitch_give_one_lean_finding():
    n = 50
    f = AttitudeTrackingCheck().run(
        _copter(n, Roll=[0.0] * 10 + [40.0] * 20 + [0.0] * 20, Pitch=[0.0] * 10 + [35.0] * 20 + [0.0] * 20)
    )
    assert len(f) == 1 and f[0].severity == Severity.CRITICAL
    assert f[0].evidence.samples["peak_err_deg_lean"] > 40.0


def test_copter_heading_divergence_is_at_most_a_warning():
    # The motor mixer gives yaw whatever room roll and pitch leave, and the crash check
    # ignores heading: a held lean with a lost heading is not a loss of control.
    n = 50
    f = AttitudeTrackingCheck().run(_copter(n, Yaw=[0.0] * 10 + [60.0] * 30 + [0.0] * 10))
    assert [(x.severity, x.title) for x in f] == [(Severity.WARN, "Heading not held (yaw)")]
    assert f[0].evidence.samples["peak_err_deg_yaw"] == 60.0


def test_copter_near_90_deg_pitch_euler_differences_are_not_errors():
    # At -85 deg pitch, roll 30 / yaw -30 is within 3 deg of roll 0 / yaw 0: Euler roll and
    # yaw turn about nearly the same axis there, so their differences mean nothing.
    n = 50
    log = _copter(
        n,
        DesPitch=[-85.0] * n,
        Pitch=[-85.0] * n,
        Roll=[0.0] * 10 + [30.0] * 30 + [0.0] * 10,
        Yaw=[0.0] * 10 + [-30.0] * 30 + [0.0] * 10,
    )
    assert AttitudeTrackingCheck().run(log) == []


def test_copter_heading_is_not_judged_while_the_lean_error_is_over_30_deg():
    # The controller scales its yaw correction down above AC_ATTITUDE_THRUST_ERROR_ANGLE.
    n = 50
    f = AttitudeTrackingCheck().run(
        _copter(n, Roll=[0.0] * 10 + [90.0] * 30 + [0.0] * 10, Yaw=[0.0] * 10 + [90.0] * 30 + [0.0] * 10)
    )
    assert [x.title for x in f] == ["Attitude tracking failure (roll)"]


def _plane_with_airspeed(airspeed):
    n = 100
    att = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=[0.0] * 50 + [40.0] * 20 + [0.0] * 30,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    ctun = build_series(0.0, 10.0, As=airspeed)
    return make_flight_log(
        {"ATT": att, "CTUN": ctun},
        vehicle_type="ArduPlane",
        vehicle_kind="plane",
        params={"AIRSPEED_MAX": 22.0, "AIRSPEED_MIN": 12.0, "SCALING_SPEED": 15.0},
    )


def test_plane_loss_of_control_above_airspeed_max_names_the_speed_and_surface_scaling():
    # Flown at 30 m/s on AIRSPEED_MAX 22 and SCALING_SPEED 15: the surfaces get 15/30 = 0.5x.
    f = [
        x
        for x in AttitudeTrackingCheck().run(_plane_with_airspeed([30.0] * 100))
        if x.severity == Severity.CRITICAL
    ]
    assert f and f[0].evidence.samples["airspeed_at_onset"] == 30.0
    assert f[0].evidence.samples["speed_scaler_at_onset"] == 0.5
    assert "above AIRSPEED_MAX (22 m/s)" in f[0].explanation


def test_plane_speed_note_needs_a_flight_spent_above_airspeed_max():
    # Above AIRSPEED_MAX only in the dive that followed the loss of control: no note.
    speeds = [15.0] * 50 + [30.0] * 10 + [15.0] * 40
    f = [
        x
        for x in AttitudeTrackingCheck().run(_plane_with_airspeed(speeds))
        if x.severity == Severity.CRITICAL
    ]
    assert f and "airspeed_at_onset" not in f[0].evidence.samples


def _plane_roll(des, roll, modes=None):
    n = len(des)
    att = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    msgs = {"ATT": att}
    if modes:
        msgs["MODE"] = [{"TimeUS": 0, "ModeNum": m, "Mode": m} for m in modes]
    return make_flight_log(msgs, vehicle_type="ArduPlane", vehicle_kind="plane")


def test_plane_lag_after_a_demand_step_is_a_warning():
    # A turn reversal: the demand jumps from +45 to -45 deg and the roll follows over 1.6 s.
    des = [45.0] * 20 + [-45.0] * 40
    roll = [45.0] * 20 + [45.0 - 90.0 * min(1.0, i / 16) for i in range(40)]
    f = AttitudeTrackingCheck().run(_plane_roll(des, roll, modes=[10]))
    assert [(x.severity, x.title) for x in f] == [
        (Severity.WARN, "Attitude lagged a step in the demand (roll)")
    ]
    assert "RLL2SRV_TCONST" in f[0].explanation


def test_plane_divergence_without_a_demand_step_stays_critical():
    des = [0.0] * 60
    roll = [0.0] * 20 + [40.0] * 12 + [0.0] * 28
    f = AttitudeTrackingCheck().run(_plane_roll(des, roll, modes=[10]))
    assert [x.severity for x in f] == [Severity.CRITICAL]


def test_plane_long_divergence_after_a_step_stays_critical():
    des = [45.0] * 20 + [-45.0] * 60
    roll = [45.0] * 80  # never follows the reversal
    f = AttitudeTrackingCheck().run(_plane_roll(des, roll, modes=[10]))
    assert [x.severity for x in f] == [Severity.CRITICAL]


def test_plane_vtol_mode_step_is_not_excused():
    des = [45.0] * 20 + [-45.0] * 40
    roll = [45.0] * 20 + [45.0 - 90.0 * min(1.0, i / 16) for i in range(40)]
    f = AttitudeTrackingCheck().run(_plane_roll(des, roll, modes=[19]))  # QLOITER
    assert [x.severity for x in f] == [Severity.CRITICAL]
