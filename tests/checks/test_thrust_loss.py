"""Tests for the thrust-loss check.

The output pattern is modelled on ArduCopter 4.6.3 SITL with SIM_ENGINE_FAIL=1,
SIM_ENGINE_MUL=0.55 on a quad plus frame: the firmware logged "Potential Thrust
Loss (2)" while C2 sat at 1950 us, C1 (the motor opposite it) at ~1521 us and C3/C4
at ~1557 us, sinking at full throttle output, about 10 degrees of requested lean.
"""

from __future__ import annotations

from ardupilot_mcp.checks.firmware import FirmwareVerdictsCheck
from ardupilot_mcp.checks.thrust_loss import ThrustLossCheck, frame_geometry, opposite_side
from ardupilot_mcp.model import Severity
from ardupilot_mcp.motor_geometry import FRAME_GEOMETRY
from tests.helpers import build_series, make_flight_log

QUAD_PLUS = {"FRAME_CLASS": 1, "FRAME_TYPE": 0}
HOVER = (1591.0, 1591.0, 1591.0, 1591.0)
LOST_2 = (1521.0, 1950.0, 1557.0, 1557.0)


def _flight(
    outputs: tuple[float, float, float, float] = LOST_2,
    *,
    seconds: float = 3.0,
    thr_in: float = 1.5,
    thr_out: float = 1.0,
    climb: float = -200.0,
    des_roll: float = 9.7,
    roll: float = -5.0,
    msgs: list[tuple[float, str]] | None = None,
    extra: dict | None = None,
    params: dict | None = None,
) -> object:
    rate = 20.0
    n_hover, n_fault = 200, int(seconds * rate)
    cols = {f"C{i + 1}": [HOVER[i]] * n_hover + [outputs[i]] * n_fault for i in range(4)}
    rcou = build_series(0.0, rate, **cols)
    ctun = build_series(
        0.0,
        rate,
        ThI=[0.36] * n_hover + [thr_in] * n_fault,
        ThO=[0.36] * n_hover + [thr_out] * n_fault,
        CRt=[0.0] * n_hover + [climb] * n_fault,
    )
    att = build_series(
        0.0,
        rate,
        DesRoll=[0.2] * n_hover + [des_roll] * n_fault,
        Roll=[0.2] * n_hover + [roll] * n_fault,
        DesPitch=[0.1] * n_hover + [-2.5] * n_fault,
        Pitch=[0.1] * n_hover + [4.0] * n_fault,
    )
    ev = [{"TimeUS": 0, "Id": 10}, {"TimeUS": 500_000, "Id": 28}]
    messages = {"RCOU": rcou, "CTUN": ctun, "ATT": att, "EV": ev}
    if msgs:
        messages["MSG"] = [{"TimeUS": int(t * 1e6), "Message": m} for t, m in msgs]
    messages.update(extra or {})
    return make_flight_log(messages, params={**QUAD_PLUS, **(params or {})}, vehicle_kind="copter")


def test_geometry_table_covers_the_common_frames():
    for key, count in {(1, 0): 4, (1, 1): 4, (2, 1): 6, (3, 1): 8, (14, 1): 10}.items():
        assert len(FRAME_GEOMETRY[key]) == count


def test_opposite_motor_follows_the_frame():
    plus = FRAME_GEOMETRY[(1, 0)]
    assert opposite_side(plus, 1) == [0]  # motor 2 (left) faces motor 1 (right)
    quad_x = FRAME_GEOMETRY[(1, 1)]
    assert opposite_side(quad_x, 0) == [1]  # front-right faces rear-left
    octaquad_x = FRAME_GEOMETRY[(4, 1)]
    assert len(opposite_side(octaquad_x, 0)) == 2  # a coaxial pair sits opposite


def test_unknown_frame_has_no_geometry():
    log = make_flight_log({}, params={"FRAME_CLASS": 6})  # heli
    assert frame_geometry(log) is None


def test_hover_without_fault_reports_nothing():
    assert ThrustLossCheck().run(_flight(HOVER)) == []


def test_signature_alone_is_warn_and_names_the_motor():
    [f] = ThrustLossCheck().run(_flight())
    assert f.severity == Severity.WARN
    assert f.evidence.samples["motor"] == 2
    assert "opposite motor (1)" in f.explanation
    assert f.evidence.time_start_s is not None and abs(f.evidence.time_start_s - 10.0) < 0.1


def test_firmware_message_is_critical_and_corroborated():
    log = _flight(msgs=[(11.0, "Potential Thrust Loss (2)")])
    [f] = ThrustLossCheck().run(log)
    assert f.severity == Severity.CRITICAL
    assert f.title == "Thrust loss on motor 2"
    assert f.evidence.samples["signature_motor"] == 2
    # The firmware check leaves this verdict to the thrust-loss check: one finding, not two.
    assert FirmwareVerdictsCheck().run(log) == []


def test_motb_thrust_boost_alone_is_critical():
    motb = build_series(0.0, 10.0, FailFlags=[2] * 100 + [3] * 20)
    [f] = ThrustLossCheck().run(_flight(HOVER, extra={"MOTB": motb}))
    assert f.severity == Severity.CRITICAL
    assert f.evidence.samples["thrust_boost_s"] > 1.0


def test_err_thrust_loss_alone_is_critical():
    err = [{"TimeUS": 11_000_000, "Subsys": 25, "ECode": 1}]
    [f] = ThrustLossCheck().run(_flight(HOVER, extra={"ERR": err}))
    assert f.severity == Severity.CRITICAL


def test_shorter_than_a_second_is_ignored():
    assert ThrustLossCheck().run(_flight(seconds=0.6)) == []


def test_climbing_is_not_thrust_loss():
    assert ThrustLossCheck().run(_flight(climb=150.0)) == []


def test_low_throttle_is_not_thrust_loss():
    assert ThrustLossCheck().run(_flight(thr_in=0.5, thr_out=0.6)) == []


def test_saturated_throttle_output_counts_like_the_firmware_upper_limit():
    # Firmware: throttle_in >= 0.9 OR motors at their upper throttle limit.
    [f] = ThrustLossCheck().run(_flight(thr_in=0.5, thr_out=1.0))
    assert f.severity == Severity.WARN


def test_large_requested_lean_is_excluded():
    assert ThrustLossCheck().run(_flight(des_roll=25.0)) == []


def test_lost_attitude_control_is_excluded():
    assert ThrustLossCheck().run(_flight(roll=-35.0)) == []


def test_opposite_not_lowest_is_not_the_signature():
    # C2 pinned, but the motor opposite it is the highest of the rest: not a lost motor.
    assert ThrustLossCheck().run(_flight((1700.0, 1950.0, 1557.0, 1557.0))) == []


def test_two_motors_pinned_is_overload_not_one_motor():
    assert ThrustLossCheck().run(_flight((1521.0, 1950.0, 1950.0, 1557.0))) == []


def test_motor_order_from_servo_functions():
    # Motor 2 wired to output 5: its PWM is read from C5, not C2.
    log = _flight(
        HOVER,
        params={"SERVO1_FUNCTION": 33, "SERVO5_FUNCTION": 34, "SERVO3_FUNCTION": 35, "SERVO4_FUNCTION": 36},
        extra={"RCOU": build_series(0.0, 20.0, **_moved_outputs())},
    )
    [f] = ThrustLossCheck().run(log)
    assert f.evidence.samples["motor"] == 2 and "(C5)" in f.explanation


def _moved_outputs() -> dict[str, list[float]]:
    n_hover, n_fault = 200, 60
    return {
        "C1": [HOVER[0]] * n_hover + [LOST_2[0]] * n_fault,
        "C2": [1000.0] * (n_hover + n_fault),
        "C3": [HOVER[2]] * n_hover + [LOST_2[2]] * n_fault,
        "C4": [HOVER[3]] * n_hover + [LOST_2[3]] * n_fault,
        "C5": [HOVER[1]] * n_hover + [LOST_2[1]] * n_fault,
    }


def test_signature_far_from_the_detection_is_not_used_as_corroboration():
    # The firmware detected at t=100 s; the pattern in the outputs is at t=10..13 s.
    [f] = ThrustLossCheck().run(_flight(msgs=[(100.0, "Potential Thrust Loss (2)")]))
    assert f.severity == Severity.CRITICAL
    assert "signature_motor" not in f.evidence.samples


def test_firmware_finding_names_the_output_channel():
    log = _flight(
        HOVER,
        msgs=[(11.0, "Potential Thrust Loss (2)")],
        params={"SERVO11_FUNCTION": 34},
    )
    [f] = ThrustLossCheck().run(log)
    assert "motor 2 = output C11" in f.explanation


def test_onset_is_before_the_detection():
    # The firmware fires after 1 s of the pattern: the loss began at least 1 s earlier.
    err = [{"TimeUS": 30_000_000, "Subsys": 25, "ECode": 1}]
    [f] = ThrustLossCheck().run(_flight(HOVER, extra={"ERR": err}))
    assert f.evidence.time_start_s == 29.0
    # With a corroborating output pattern, the pattern's own start is the onset.
    [g] = ThrustLossCheck().run(_flight(msgs=[(11.5, "Potential Thrust Loss (2)")]))
    assert abs(g.evidence.time_start_s - 10.0) < 0.1
