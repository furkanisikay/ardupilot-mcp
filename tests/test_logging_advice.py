"""Tests for the logging advisor."""

from __future__ import annotations

from ardupilot_mcp.logging_advice import logging_advice
from tests.helpers import build_series, make_flight_log


def test_missing_message_with_bit_off_is_advised():
    log = make_flight_log(
        {"ATT": build_series(0.0, 10.0, Roll=[0.0] * 10)}, params={"LOG_BITMASK": 0}, vehicle_kind="copter"
    )
    missing = {a.missing: a for a in logging_advice(log)}
    assert "VIBE" in missing and missing["VIBE"].bitmask_bit == 7
    assert "ATT" not in missing  # it is logged


def test_missing_message_with_bit_on_is_not_a_logging_problem():
    log = make_flight_log({}, params={"LOG_BITMASK": 1 << 2}, vehicle_kind="copter")
    assert "GPS" not in {a.missing for a in logging_advice(log)}


def test_slow_imu_suggests_fast_imu_or_batch_sampler():
    imu = build_series(0.0, 25.0, GyrX=[0.0] * 100)
    log = make_flight_log({"IMU": imu}, params={"LOG_BITMASK": 1 << 7}, vehicle_kind="copter")
    assert any(a.bitmask_bit == 18 for a in logging_advice(log))
    log = make_flight_log(
        {"IMU": imu}, params={"LOG_BITMASK": 1 << 7, "INS_LOG_BAT_MASK": 1}, vehicle_kind="copter"
    )
    assert not any(a.bitmask_bit == 18 for a in logging_advice(log))


def test_plane_gets_no_copter_bit_numbers():
    assert logging_advice(make_flight_log({}, params={"LOG_BITMASK": 0}, vehicle_kind="plane")) == []
