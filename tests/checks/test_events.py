"""Tests for the events check (errors, failsafes & mode/event timeline)."""

from __future__ import annotations

from ardupilot_mcp.checks.events import EventsCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def test_always_runs_but_empty_when_no_event_messages():
    # requires=set() -> never SKIPPED; with no ERR/MODE/EV it RAN with no findings.
    log = make_flight_log({})
    result = EventsCheck().execute(log)
    assert result.status == CheckStatus.RAN
    assert result.findings == []


def test_clean_flight_no_err_findings_only_info_timeline():
    # A clean flight: a normal mode timeline and lifecycle events, no ERR onsets.
    mode = build_series(0.0, 1.0, ModeNum=[0.0, 5.0, 6.0])  # STABILIZE -> LOITER -> RTL
    ev = build_series(0.0, 1.0, Id=[10.0, 16.0, 11.0])  # ARMED, TAKEOFF, DISARMED
    findings = EventsCheck().run(make_flight_log({"MODE": mode, "EV": ev}))
    # No errors -> no WARN/CRITICAL.
    assert not any(f.severity in (Severity.WARN, Severity.CRITICAL) for f in findings)
    assert all(f.severity == Severity.INFO for f in findings)


def test_err_critical_subsystem_battery_failsafe():
    # Subsys 6 == FAILSAFE_BATT is in CRITICAL_ERR_SUBSYSTEMS -> CRITICAL.
    err = build_series(0.0, 1.0, Subsys=[6.0], ECode=[1.0])
    findings = EventsCheck().run(make_flight_log({"ERR": err}))
    crit = [f for f in findings if f.severity == Severity.CRITICAL]
    assert crit, "expected a critical battery-failsafe finding"
    f = crit[0]
    assert "FAILSAFE_BATT" in f.title
    assert "ECode 1" in f.title
    assert f.evidence.samples["Subsys"] == 6
    assert f.evidence.samples["ECode"] == 1
    assert "ERR" in f.evidence.message_types


def test_err_noncritical_subsystem_is_warn():
    # Subsys 3 == COMPASS is not in CRITICAL_ERR_SUBSYSTEMS -> WARN.
    err = build_series(0.0, 1.0, Subsys=[3.0], ECode=[2.0])
    findings = EventsCheck().run(make_flight_log({"ERR": err}))
    warns = [f for f in findings if f.severity == Severity.WARN]
    assert warns, "expected a warn-level compass error finding"
    f = warns[0]
    assert "COMPASS" in f.title
    assert f.evidence.samples["Subsys"] == 3
    assert f.evidence.samples["ECode"] == 2
    assert not any(x.severity == Severity.CRITICAL for x in findings)


def test_err_ecode_zero_is_cleared_no_finding():
    # ECode == 0 clears an error and must NOT produce a finding.
    err = build_series(0.0, 1.0, Subsys=[6.0], ECode=[0.0])
    findings = EventsCheck().run(make_flight_log({"ERR": err}))
    assert findings == []


def test_err_onset_then_clear_emits_only_onset():
    # Onset (ECode 1) then clear (ECode 0): exactly one finding for the onset.
    err = build_series(0.0, 1.0, Subsys=[6.0, 6.0], ECode=[1.0, 0.0])
    findings = EventsCheck().run(make_flight_log({"ERR": err}))
    err_findings = [f for f in findings if "FAILSAFE_BATT" in f.title]
    assert len(err_findings) == 1
    assert err_findings[0].severity == Severity.CRITICAL
    assert err_findings[0].evidence.time_start_s == 0.0


def test_mode_timeline_single_info_with_names_and_dedupe():
    # Consecutive duplicate (two STABILIZE) collapses; names decoded from ModeNum.
    mode = build_series(0.0, 1.0, ModeNum=[0.0, 0.0, 5.0, 6.0])
    findings = EventsCheck().run(make_flight_log({"MODE": mode}))
    info = [f for f in findings if f.severity == Severity.INFO and "mode" in f.title.lower()]
    assert len(info) == 1
    f = info[0]
    assert "STABILIZE -> LOITER -> RTL" in f.explanation
    # 4 raw records, 3 distinct consecutive modes after dedupe.
    assert f.evidence.samples["mode_changes"] == 3
    assert f.evidence.samples["mode_records"] == 4


def test_mode_timeline_accepts_string_mode_field():
    # When ModeNum is absent, a string Mode field is used verbatim.
    mode = [
        {"TimeUS": 0, "Mode": "STABILIZE"},
        {"TimeUS": 1_000_000, "Mode": "AUTO"},
    ]
    findings = EventsCheck().run(make_flight_log({"MODE": mode}))
    info = [f for f in findings if f.severity == Severity.INFO]
    assert info
    assert "STABILIZE -> AUTO" in info[0].explanation


def test_ev_summary_counts_key_lifecycle_events():
    # ARMED(10), AUTO_ARMED(15), LAND_COMPLETE(18), DISARMED(11).
    ev = build_series(0.0, 1.0, Id=[10.0, 15.0, 18.0, 11.0])
    findings = EventsCheck().run(make_flight_log({"EV": ev}))
    info = [f for f in findings if f.severity == Severity.INFO and "event" in f.title.lower()]
    assert len(info) == 1
    f = info[0]
    assert f.evidence.samples["ev_count"] == 4
    assert f.evidence.samples["ev_ARMED"] == 1
    assert f.evidence.samples["ev_AUTO_ARMED"] == 1
    assert f.evidence.samples["ev_DISARMED"] == 1


def test_ev_summary_is_single_finding_even_with_many_events():
    # Many EV records still collapse to exactly one INFO finding.
    ids = [10.0, 25.0, 41.0, 60.0, 11.0]  # ARMED, SET_HOME, FENCE_ENABLE, EKF_ALT_RESET, DISARMED
    ev = build_series(0.0, 2.0, Id=ids)
    findings = EventsCheck().run(make_flight_log({"EV": ev}))
    ev_findings = [f for f in findings if "EV" in f.evidence.message_types]
    assert len(ev_findings) == 1
    assert ev_findings[0].evidence.samples["ev_count"] == 5


def test_combined_err_mode_ev_all_present():
    err = build_series(0.0, 1.0, Subsys=[7.0], ECode=[1.0])  # FAILSAFE_GPS -> CRITICAL
    mode = build_series(0.0, 1.0, ModeNum=[0.0, 6.0])  # STABILIZE -> RTL
    ev = build_series(0.0, 1.0, Id=[10.0, 11.0])  # ARMED, DISARMED
    findings = EventsCheck().run(make_flight_log({"ERR": err, "MODE": mode, "EV": ev}))
    sevs = {f.severity for f in findings}
    assert Severity.CRITICAL in sevs  # the GPS failsafe
    assert Severity.INFO in sevs  # mode timeline + ev summary
    titles = {f.title for f in findings}
    assert any("FAILSAFE_GPS" in t for t in titles)


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("events") is not None


def test_ekf_primary_is_a_lane_switch_not_an_error():
    # ArduCopter/ekf_check.cpp logs ERR EKF_PRIMARY with the new primary core as ECode:
    # core 0 is a switch too, not a "cleared" error.
    errs = [{"TimeUS": 5_000_000, "Subsys": 24, "ECode": 1}, {"TimeUS": 9_000_000, "Subsys": 24, "ECode": 0}]
    findings = [f for f in EventsCheck().run(make_flight_log({"ERR": errs})) if "EKF primary" in f.title]
    assert [f.title for f in findings] == ["EKF primary core changed to 1", "EKF primary core changed to 0"]
    assert all(f.severity == Severity.INFO for f in findings)


def test_a_flight_mode_error_names_the_mode_it_could_not_enter():
    log = make_flight_log({"ERR": [{"TimeUS": 1_000_000, "Subsys": 10, "ECode": 6}]})
    (finding,) = EventsCheck().run(log)
    assert "could not switch to RTL" in finding.explanation


def test_a_corrupt_err_record_is_skipped():
    log = make_flight_log({"ERR": [{"TimeUS": 1_000_000, "Subsys": float("nan"), "ECode": float("nan")}]})
    assert EventsCheck().run(log) == []


def test_a_failed_switch_to_mode_zero_is_reported():
    log = make_flight_log({"ERR": [{"TimeUS": 1_000_000, "Subsys": 10, "ECode": 0}]})
    (finding,) = EventsCheck().run(log)
    assert "could not switch to STABILIZE" in finding.explanation


def _disarm_log(method: int, land_events: list[dict]):
    ev = [
        {"TimeUS": 0, "Id": 10},
        {"TimeUS": 2_000_000, "Id": 28},
        *land_events,
        {"TimeUS": 20_000_000, "Id": 11},
    ]
    arm = [{"TimeUS": 0, "ArmState": 1, "Method": 2}, {"TimeUS": 20_000_000, "ArmState": 0, "Method": method}]
    return make_flight_log({"EV": ev, "ARM": arm}, vehicle_kind="copter")


def test_failsafe_disarm_in_flight_is_a_warning():
    # ARM.Method 7 = battery failsafe (AP_Arming::Method): the autopilot stopped the motors aloft.
    f = [x for x in EventsCheck().run(_disarm_log(7, [])) if x.title.startswith("Disarmed in flight")]
    assert [(x.title, x.severity) for x in f] == [("Disarmed in flight (battery failsafe)", Severity.WARN)]


def test_switch_disarm_in_flight_is_info():
    f = [x for x in EventsCheck().run(_disarm_log(2, [])) if x.title.startswith("Disarmed in flight")]
    assert [x.severity for x in f] == [Severity.INFO]


def test_switch_disarm_right_after_touchdown_is_not_in_flight():
    # LAND_COMPLETE_MAYBE (17) a second before: the pilot disarmed on the ground.
    land = [{"TimeUS": 19_000_000, "Id": 17}]
    assert not [
        x for x in EventsCheck().run(_disarm_log(2, land)) if x.title.startswith("Disarmed in flight")
    ]


def test_disarm_is_not_judged_without_a_land_detector_record():
    ev = [{"TimeUS": 0, "Id": 10}, {"TimeUS": 20_000_000, "Id": 11}]
    arm = [{"TimeUS": 20_000_000, "ArmState": 0, "Method": 7}]
    log = make_flight_log({"EV": ev, "ARM": arm}, vehicle_kind="copter")
    assert not [x for x in EventsCheck().run(log) if x.title.startswith("Disarmed in flight")]


def test_land_events_written_by_the_disarm_itself_do_not_hide_it():
    # The firmware writes LAND_COMPLETE (18) and LAND_COMPLETE_MAYBE (17) at the disarm instant,
    # in the air too; only land events before the disarm put it on the ground.
    land = [{"TimeUS": 20_000_000, "Id": 18}, {"TimeUS": 20_000_000, "Id": 17}]
    f = [x for x in EventsCheck().run(_disarm_log(7, land)) if x.title.startswith("Disarmed in flight")]
    assert [x.severity for x in f] == [Severity.WARN]


def _vibe_trigger_log(vibe_z, clips):
    from tests.helpers import build_series, make_flight_log

    n = len(vibe_z)
    vibe = build_series(0.0, 10.0, VibeX=[5.0] * n, VibeY=[5.0] * n, VibeZ=vibe_z, Clip=clips)
    err = [{"TimeUS": 8_000_000, "Subsys": 29, "ECode": 1}]
    return make_flight_log({"VIBE": vibe, "ERR": err, "EV": [{"TimeUS": 0, "Id": 10}]})


def test_vibration_compensation_with_low_vibration_points_at_the_height_estimate():
    # Forum logs 145303 (oscillating PSC) and 145599 (poor GPS): peak <= 15 m/s^2, no clips.
    from ardupilot_mcp.checks.events import EventsCheck

    f = [
        x for x in EventsCheck().run(_vibe_trigger_log([8.0] * 100, [0] * 100)) if "FAILSAFE_VIBE" in x.title
    ]
    assert (
        f and "vibration did not trigger it" in f[0].explanation and "VibeZ peak 8 m/s^2" in f[0].explanation
    )


def test_vibration_compensation_with_high_vibration_says_so():
    from ardupilot_mcp.checks.events import EventsCheck

    vz = [8.0] * 60 + [70.0] * 40
    clips = [0] * 60 + list(range(40))
    f = [x for x in EventsCheck().run(_vibe_trigger_log(vz, clips)) if "FAILSAFE_VIBE" in x.title]
    assert f and "Vibration was high" in f[0].explanation and "did not trigger" not in f[0].explanation


def test_disarm_by_an_arm_switch_on_the_yaw_stick_is_a_warning():
    from tests.helpers import make_flight_log

    ev = [{"TimeUS": 0, "Id": 10}, {"TimeUS": 1_000_000, "Id": 28}]
    arm = [{"TimeUS": 0, "ArmState": 1, "Method": 2}, {"TimeUS": 30_000_000, "ArmState": 0, "Method": 2}]
    log = make_flight_log({"EV": ev, "ARM": arm}, params={"RC4_OPTION": 153})
    f = [x for x in EventsCheck().run(log) if x.title.startswith("Disarmed in flight")]
    assert [x.severity for x in f] == [Severity.WARN] and "yaw stick" in f[0].explanation
