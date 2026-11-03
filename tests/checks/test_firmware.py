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


def test_announced_failsafes_are_verdicts_and_cleared_lines_are_not():
    log = make_flight_log(
        {
            "MSG": _msgs(
                (0, "boot"),
                (40, "Radio Failsafe - Continuing Landing"),
                (45, "Radio Failsafe Cleared"),
                (60, "Battery Failsafe"),
                (70, "GCS Failsafe Cleared"),
            )
        }
    )
    found = {f.title: f for f in FirmwareVerdictsCheck().run(log)}
    assert set(found) == {"Firmware triggered a radio failsafe", "Firmware triggered a battery failsafe"}
    radio = found["Firmware triggered a radio failsafe"]
    assert radio.severity == Severity.CRITICAL and radio.evidence.samples["occurrences"] == 1


def test_quadplane_transition_failure_and_assists():
    # Messages as ArduPlane writes them (quadplane.cpp, VTOL_Assist.cpp); the texts are as
    # forum logs 140907_0, 142096_0 and 143882_0 carry them.
    log = make_flight_log(
        {
            "MSG": _msgs(
                (0, "ArduPlane V4.6.3"),
                (40, "Transition failed, exceeded time limit"),
                (55, "Alt assist 4.2m"),
                (60, "Angle assist r=45 p=-10"),
            ),
            "MODE": [{"TimeUS": 40_100_000, "Mode": 20, "ModeNum": 20, "Rsn": 23}],
        },
        vehicle_type="ArduPlane",
        vehicle_kind="plane",
    )
    titles = {f.title: f.severity for f in FirmwareVerdictsCheck().run(log)}
    assert titles["QuadPlane transition to forward flight failed"] == Severity.CRITICAL
    assert titles["VTOL motors assisted: altitude too low"] == Severity.WARN
    assert titles["VTOL motors assisted: attitude error"] == Severity.WARN
    assert "Mode changed by failed VTOL transition" in titles


def test_vtol_thrust_loss_is_reported_by_the_firmware_check():
    # Plane 4.7 (quadplane.cpp thrust_loss_check), as forum logs 144992_0 and 145010_1 carry it.
    log = make_flight_log(
        {"MSG": _msgs((0, "ArduPlane V4.7.0"), (173, "Potential VTOL Thrust Loss (1)"))},
        vehicle_type="ArduPlane",
        vehicle_kind="plane",
    )
    f = [x for x in FirmwareVerdictsCheck().run(log) if "thrust loss" in x.title]
    assert [(x.title, x.severity) for x in f] == [("Firmware detected VTOL thrust loss", Severity.CRITICAL)]
    assert f[0].evidence.samples["motor"] == 1  # the suspect motor (index + 1)
