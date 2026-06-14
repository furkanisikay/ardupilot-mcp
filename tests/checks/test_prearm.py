"""Tests for the startup & pre-arm messages check."""

from __future__ import annotations

from ardupilot_mcp.checks.prearm import StartupMessagesCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import make_flight_log


def _msg_log(messages):
    """Build a log with a MSG block from a list of (timeUS, text) pairs."""
    recs = [{"TimeUS": t, "Message": m} for t, m in messages]
    return make_flight_log({"MSG": recs})


def _statustext_log(messages):
    recs = [{"TimeUS": t, "Text": m} for t, m in messages]
    return make_flight_log({"STATUSTEXT": recs})


def test_always_runs_even_without_msg():
    # requires=set() -> never SKIPPED; with no MSG/STATUSTEXT it RAN with no findings.
    result = StartupMessagesCheck().execute(make_flight_log({}))
    assert result.status == CheckStatus.RAN
    assert result.findings == []


def test_empty_message_block_no_findings():
    log = make_flight_log({"MSG": []})
    assert StartupMessagesCheck().run(log) == []


def test_routine_only_log_produces_no_findings():
    # A typical clean boot: banner, board id, frame, gps, init, alignment.
    msgs = [
        (0, "ArduCopter V4.5.7 (2a3dc4b7)"),
        (1, "ChibiOS: 6a85082c"),
        (2, "fmuv5 00380021 32325111 37383239"),
        (3, "Param space used: 1091/5376"),
        (4, "RCOut: PWM:1-16"),
        (5, "Frame: QUAD/X"),
        (6, "GPS 1: detected as u-blox at 230400 baud"),
        (7, "EKF2 IMU0 initial yaw alignment complete"),
        (8, "EKF2 IMU0 initialised"),
    ]
    assert StartupMessagesCheck().run(_msg_log(msgs)) == []


def test_prearm_not_calibrated_is_warn():
    # Leading routine line at t=0 anchors the rebased clock so the notable
    # line keeps its 1.0s offset.
    log = _msg_log([(0, "Frame: QUAD/X"), (1_000_000, "PreArm: Compass not calibrated")])
    findings = StartupMessagesCheck().run(log)
    assert len(findings) == 1
    f = findings[0]
    assert f.severity == Severity.WARN
    assert "not calibrated" in f.explanation.lower()
    assert f.evidence.time_start_s == 1.0
    assert f.evidence.message_types == ["MSG"]


def test_initialised_routine_message_ignored():
    log = _msg_log([(0, "EKF2 IMU0 initialised")])
    assert StartupMessagesCheck().run(log) == []


def test_ground_mag_anomaly_is_info():
    log = _msg_log([(0, "Frame: QUAD"), (5_000_000, "EKF2 IMU0 ground mag anomaly, yaw re-aligned")])
    findings = StartupMessagesCheck().run(log)
    assert len(findings) == 1
    f = findings[0]
    assert f.severity == Severity.INFO
    assert "ground mag anomaly" in f.explanation.lower()
    assert f.evidence.time_start_s == 5.0


def test_glitch_and_reset_are_info():
    log = _msg_log([(0, "GPS Glitch"), (1_000_000, "EKF3 IMU0 forced reset")])
    findings = StartupMessagesCheck().run(log)
    assert len(findings) == 2
    assert all(f.severity == Severity.INFO for f in findings)


def test_error_and_failed_are_warn():
    log = _msg_log(
        [
            (0, "GPS Glitch or Compass error"),
            (1_000_000, "PreArm: Logging failed"),
        ]
    )
    findings = StartupMessagesCheck().run(log)
    sev = {f.severity for f in findings}
    assert sev == {Severity.WARN}
    assert len(findings) == 2


def test_dedupe_identical_lines_keeps_earliest_timestamp():
    log = _msg_log(
        [
            (0, "Frame: QUAD"),  # routine anchor at t=0
            (3_000_000, "PreArm: Need 3D Fix"),
            (1_000_000, "PreArm: Need 3D Fix"),
            (5_000_000, "PreArm: Need 3D Fix"),
        ]
    )
    findings = StartupMessagesCheck().run(log)
    assert len(findings) == 1
    assert findings[0].evidence.time_start_s == 1.0


def test_distinct_imu_lines_are_separate_findings():
    # Different IMU numbers -> distinct text -> two findings.
    log = _msg_log(
        [
            (0, "EKF2 IMU0 ground mag anomaly, yaw re-aligned"),
            (1_000_000, "EKF2 IMU1 ground mag anomaly, yaw re-aligned"),
        ]
    )
    findings = StartupMessagesCheck().run(log)
    assert len(findings) == 2


def test_statustext_source_is_read():
    log = _statustext_log(
        [
            (0, "EKF2 IMU0 initialised"),  # routine anchor at t=0
            (2_000_000, "PreArm: Battery 1 below minimum arming voltage"),
        ]
    )
    findings = StartupMessagesCheck().run(log)
    assert len(findings) == 1
    assert findings[0].severity == Severity.WARN
    assert findings[0].evidence.time_start_s == 2.0


def test_board_id_line_is_ignored():
    # Two+ 8-hex tokens -> board-id line -> routine.
    log = _msg_log([(0, "PX4v2 00480026 3435510B 38393730 error")])
    # Even with an 'error' token, the hex board-id rule filters it.
    assert StartupMessagesCheck().run(log) == []


def test_cap_at_six_findings_with_overflow_note():
    msgs = [(i * 1_000_000, f"PreArm: problem number {i}") for i in range(8)]
    findings = StartupMessagesCheck().run(_msg_log(msgs))
    assert len(findings) == 6
    last = findings[-1]
    assert last.evidence.samples["notable_messages"] == 8
    assert last.evidence.samples.get("shown") == 6
    assert "showing the first" in last.explanation


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("prearm") is not None
