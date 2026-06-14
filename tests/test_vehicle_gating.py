"""Vehicle-kind gating: checks only run on the vehicle types they make sense for."""

from __future__ import annotations

import pytest

from ardupilot_mcp.ardupilot_meta import vehicle_kind
from ardupilot_mcp.checks.attitude import AttitudeTrackingCheck
from ardupilot_mcp.checks.motors import MotorsCheck
from ardupilot_mcp.model import CheckStatus
from tests.helpers import build_series, make_flight_log


def test_vehicle_kind_detection():
    assert vehicle_kind("ArduCopter") == "copter"
    assert vehicle_kind("ArduCopter", frame_class=6) == "heli"  # heli FRAME_CLASS
    assert vehicle_kind("ArduPlane") == "plane"
    assert vehicle_kind("Rover") == "rover"
    assert vehicle_kind("ArduSub") == "sub"
    assert vehicle_kind(None) == "unknown"


def _rcou_log(kind):
    recs = build_series(0.0, 10.0, C1=[1500.0] * 50, C2=[1500.0] * 50, C3=[1500.0] * 50, C4=[1500.0] * 50)
    return make_flight_log({"RCOU": recs}, vehicle_kind=kind)


@pytest.mark.parametrize("kind", ["heli", "plane", "rover", "sub"])
def test_motors_skipped_on_non_multirotor(kind):
    result = MotorsCheck().execute(_rcou_log(kind))
    assert result.status == CheckStatus.SKIPPED
    assert kind in (result.skipped_reason or "")


def test_motors_runs_on_copter_and_unknown():
    assert MotorsCheck().execute(_rcou_log("copter")).status == CheckStatus.RAN
    # Unknown vehicle kind must NOT be gated out (avoid false negatives).
    assert MotorsCheck().execute(_rcou_log(None)).status == CheckStatus.RAN


def _att_log(kind):
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * 50,
        Roll=[0.0] * 50,
        DesPitch=[0.0] * 50,
        Pitch=[0.0] * 50,
        DesYaw=[0.0] * 50,
        Yaw=[0.0] * 50,
    )
    return make_flight_log({"ATT": recs}, vehicle_kind=kind)


def test_attitude_skipped_on_rover():
    result = AttitudeTrackingCheck().execute(_att_log("rover"))
    assert result.status == CheckStatus.SKIPPED


@pytest.mark.parametrize("kind", ["copter", "heli", "plane"])
def test_attitude_runs_on_attitude_controlled_vehicles(kind):
    assert AttitudeTrackingCheck().execute(_att_log(kind)).status == CheckStatus.RAN
