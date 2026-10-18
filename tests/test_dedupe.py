"""Tests for merging one event's echoes into the finding that measured it."""

from __future__ import annotations

import re

from ardupilot_mcp.ardupilot_meta import ERR_SUBSYSTEMS
from ardupilot_mcp.checks.firmware import FAILSAFE_REASONS, VERDICTS
from ardupilot_mcp.checks.util import make_finding
from ardupilot_mcp.dedupe import MERGE_SLACK_S, VERDICT_TOPICS, merge_duplicates, topic
from ardupilot_mcp.model import Severity

_SUBSYS = {name: n for n, name in ERR_SUBSYSTEMS.items()}
_REASON = {label: n for n, label in FAILSAFE_REASONS.items()}


def _f(check: str, sev: Severity, title: str, t0: float | None, t1: float | None = None):
    """A finding as its check produces it, structured samples included."""
    samples: dict[str, float] = {}
    if check == "events" and (m := re.match(r"Error: (\w+) \(ECode (\d+)\)", title)):
        samples = {"Subsys": _SUBSYS[m.group(1)], "ECode": int(m.group(2))}
    if check == "firmware" and title.startswith("Mode changed by "):
        samples = {"mode_reason": _REASON[title.removeprefix("Mode changed by ")]}
    return make_finding(
        check, sev, title, "x", time_start_s=t0, time_end_s=t1, message_types=[check.upper()], samples=samples
    )


def test_failsafe_echoes_join_the_measured_finding_and_raise_its_severity():
    rc = _f("rcin", Severity.WARN, "RC signal loss", 10.0, 14.0)
    err = _f("events", Severity.CRITICAL, "Error: FAILSAFE_RADIO (ECode 1)", 10.5)
    mode = _f("firmware", Severity.WARN, "Mode changed by radio failsafe", 10.6)
    out = merge_duplicates([rc, err, mode])
    assert out == [rc]
    assert rc.severity == Severity.CRITICAL
    assert rc.related == [
        "events: Error: FAILSAFE_RADIO (ECode 1) at t=10.5s",
        "firmware: Mode changed by radio failsafe at t=10.6s",
    ]
    assert "EVENTS" in rc.evidence.message_types


def test_gcs_failsafe_echoes_collapse_into_one_finding():
    verdict = _f("firmware", Severity.WARN, "Firmware triggered a GCS failsafe", 30.0)
    err = _f("events", Severity.CRITICAL, "Error: FAILSAFE_GCS (ECode 1)", 30.1)
    mode = _f("firmware", Severity.WARN, "Mode changed by GCS failsafe", 30.2)
    out = merge_duplicates([verdict, err, mode])
    assert out == [verdict] and verdict.severity == Severity.CRITICAL


def test_echo_far_from_the_event_stays_separate():
    rc = _f("rcin", Severity.WARN, "RC signal loss", 10.0, 14.0)
    late = _f("events", Severity.CRITICAL, "Error: FAILSAFE_RADIO (ECode 1)", 14.0 + MERGE_SLACK_S + 1)
    assert merge_duplicates([rc, late]) == [rc, late]


def test_different_topics_never_merge():
    rc = _f("rcin", Severity.WARN, "RC signal loss", 10.0, 14.0)
    batt = _f("events", Severity.CRITICAL, "Error: FAILSAFE_BATT (ECode 1)", 11.0)
    assert merge_duplicates([rc, batt]) == [rc, batt]


def test_echoes_without_a_measurement_collapse_into_the_first():
    a = _f("events", Severity.CRITICAL, "Error: CRASH_CHECK (ECode 1)", 50.0)
    b = _f("firmware", Severity.CRITICAL, "Firmware detected a crash and disarmed", 50.1)
    c = _f("firmware", Severity.CRITICAL, "Mode changed by crash failsafe", 50.2)
    out = merge_duplicates([a, b, c])
    assert out == [a] and len(a.related) == 2


def test_an_info_finding_never_hosts_a_failsafe():
    lane = _f("ekf", Severity.INFO, "EKF primary core changed (lane switch)", 100.0, 120.0)
    err = _f("events", Severity.CRITICAL, "Error: EKFCHECK (ECode 2)", 110.0)
    out = merge_duplicates([lane, err])
    assert out == [lane, err] and lane.severity == Severity.INFO


def test_battery_failsafe_joins_battery_findings_only():
    batt = _f("power", Severity.WARN, "Battery reached low-voltage threshold", 200.0, 260.0)
    vcc = _f("power", Severity.WARN, "Board voltage (Vcc) dipped", 205.0)
    err = _f("events", Severity.CRITICAL, "Error: FAILSAFE_BATT (ECode 1)", 210.0)
    out = merge_duplicates([batt, vcc, err])
    assert out == [batt, vcc] and batt.related and not vcc.related
    assert topic(vcc) is None


def test_the_most_severe_nearby_measurement_hosts_the_echo():
    warn = _f("ekf", Severity.WARN, "EKF velocity variance (SV)", 30.0, 31.0)
    crit = _f("ekf", Severity.CRITICAL, "EKF position variance (SP)", 32.0, 33.0)
    err = _f("events", Severity.CRITICAL, "Error: FAILSAFE_EKFINAV (ECode 1)", 31.5)
    merge_duplicates([warn, crit, err])
    assert crit.related and not warn.related


def test_findings_without_a_time_are_left_alone():
    rc = _f("rcin", Severity.WARN, "RC signal loss", None)
    err = _f("events", Severity.CRITICAL, "Error: FAILSAFE_RADIO (ECode 1)", None)
    assert merge_duplicates([rc, err]) == [rc, err]


def test_battery_failsafe_trails_the_threshold_by_batt_low_timer():
    # ArduCopter 4.6.3 SITL: threshold crossed at 342.4 s, FAILSAFE_BATT at 352.5 s.
    batt = _f("power", Severity.WARN, "Battery reached low-voltage threshold", 342.4)
    err = _f("events", Severity.CRITICAL, "Error: FAILSAFE_BATT (ECode 1)", 352.5)
    assert merge_duplicates([batt, err]) == [batt] and batt.severity == Severity.CRITICAL
    # With a 2 s timer the same gap is a separate event.
    batt2 = _f("power", Severity.WARN, "Battery reached low-voltage threshold", 342.4)
    err2 = _f("events", Severity.CRITICAL, "Error: FAILSAFE_BATT (ECode 1)", 352.5)
    assert merge_duplicates([batt2, err2], {"BATT_LOW_TIMER": 2}) == [batt2, err2]


def test_an_aggregated_echo_does_not_stretch_the_host_window():
    # A mode change repeated through the flight spans first..last occurrence; the host keeps
    # its own measured window (a stretched one reads as a flight-long condition).
    rc = _f("rcin", Severity.WARN, "RC signal loss", 10.0, 14.0)
    mode = _f("firmware", Severity.WARN, "Mode changed by radio failsafe", 10.6, 300.0)
    assert merge_duplicates([rc, mode]) == [rc]
    assert rc.evidence.time_end_s == 14.0


def test_separate_events_do_not_chain_into_one():
    errs = [_f("events", Severity.CRITICAL, "Error: FAILSAFE_RADIO (ECode 1)", t) for t in (10, 14, 18, 22)]
    out = merge_duplicates(errs)
    assert out == [errs[0], errs[2]]


def test_verdict_topics_name_real_firmware_verdicts():
    assert set(VERDICT_TOPICS) <= {v.title for v in VERDICTS}


def test_an_echo_that_came_first_moves_the_start_earlier():
    gps = _f("gps", Severity.CRITICAL, "GPS lost 3D fix during the flight", 50.5, 60.0)
    fs = _f("events", Severity.CRITICAL, "Error: FAILSAFE_GPS (ECode 1)", 46.0)
    merge_duplicates([gps, fs])
    assert gps.evidence.time_start_s == 46.0 and gps.evidence.time_end_s == 60.0


def test_lane_switch_records_join_the_ekf_lane_switch_finding():
    lane = _f("ekf", Severity.INFO, "EKF primary core changed (lane switch)", 100.0, 120.0)
    err = make_finding(
        "events",
        Severity.INFO,
        "EKF primary core changed to 1",
        "x",
        time_start_s=101.0,
        samples={"Subsys": 24, "core": 1},
    )
    assert merge_duplicates([lane, err]) == [lane] and lane.related


def test_in_flight_status_line_joins_the_event():
    rc = _f("rcin", Severity.WARN, "RC signal loss", 10.0, 14.0)
    note = _f("prearm", Severity.INFO, "Startup note: Radio Failsafe", 10.2)
    assert merge_duplicates([rc, note]) == [rc]


def test_reworded_title_does_not_change_the_topic():
    err = make_finding(
        "events", Severity.CRITICAL, "Radio failsafe ERR", "x", time_start_s=1.0, samples={"Subsys": 5}
    )
    assert topic(err) == "radio"
