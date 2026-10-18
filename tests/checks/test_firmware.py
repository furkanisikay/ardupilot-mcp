"""Tests for the firmware-verdicts check."""

from __future__ import annotations

from ardupilot_mcp.checks.firmware import FirmwareVerdictsCheck
from ardupilot_mcp.model import Severity
from tests.helpers import make_flight_log


def _msgs(*lines: tuple[float, str]) -> list[dict]:
    return [{"TimeUS": int(t * 1e6), "Message": m} for t, m in lines]


def test_no_messages_no_findings():
    assert FirmwareVerdictsCheck().run(make_flight_log({})) == []


def test_routine_messages_are_ignored():
    log = make_flight_log({"MSG": _msgs((0, "ArduCopter V4.6.3"), (1, "EKF3 IMU0 is using GPS"))})
    assert FirmwareVerdictsCheck().run(log) == []


def test_thrust_loss_is_left_to_the_thrust_loss_check():
    log = make_flight_log({"MSG": _msgs((0, "ArduCopter V4.6.3"), (12.5, "Potential Thrust Loss (4)"))})
    assert FirmwareVerdictsCheck().run(log) == []


def test_yaw_imbalance_is_warn_with_percentage():
    log = make_flight_log({"MSG": _msgs((0, "boot"), (30, "Yaw Imbalance 84%"))})
    [f] = FirmwareVerdictsCheck().run(log)
    assert f.severity == Severity.WARN
    assert f.evidence.samples["value_pct"] == 84


def test_crash_disarm_is_critical():
    log = make_flight_log({"MSG": _msgs((0, "boot"), (50, "Crash: Disarming: AngErr=60>30, Accel=0.1<3.0"))})
    [f] = FirmwareVerdictsCheck().run(log)
    assert f.severity == Severity.CRITICAL


def test_failsafe_mode_reasons_are_grouped():
    modes = [
        {"TimeUS": 0, "Mode": 5, "ModeNum": 5, "Rsn": 1},
        {"TimeUS": 90_000_000, "Mode": 6, "ModeNum": 6, "Rsn": 3},
        {"TimeUS": 300_000_000, "Mode": 6, "ModeNum": 6, "Rsn": 3},
        {"TimeUS": 400_000_000, "Mode": 9, "ModeNum": 9, "Rsn": 4},
    ]
    findings = FirmwareVerdictsCheck().run(make_flight_log({"MODE": modes}, vehicle_kind="copter"))
    titles = {f.title: f for f in findings}
    radio = titles["Mode changed by radio failsafe"]
    assert radio.severity == Severity.WARN
    assert radio.evidence.samples["occurrences"] == 2
    assert "RTL" in radio.explanation
    assert "Mode changed by battery failsafe" in titles


def test_pilot_mode_changes_are_not_findings():
    modes = [
        {"TimeUS": 0, "Mode": 5, "ModeNum": 5, "Rsn": 1},
        {"TimeUS": 1_000_000, "Mode": 2, "ModeNum": 2, "Rsn": 2},
    ]
    assert FirmwareVerdictsCheck().run(make_flight_log({"MODE": modes})) == []
