"""Tests for the FlightLog domain model."""

from __future__ import annotations

import numpy as np
import pytest

from ardupilot_mcp.flight_log import FlightLog, LogMeta
from ardupilot_mcp.model import LogIntegrity
from tests.helpers import build_series, make_flight_log


def test_has_get_count():
    log = make_flight_log({"ATT": [{"TimeUS": 0, "Roll": 1.0}, {"TimeUS": 1000, "Roll": 2.0}]})
    assert log.has("ATT")
    assert not log.has("VIBE")
    assert log.count("ATT") == 2
    assert log.get("VIBE") == []


def test_timestamp_normalised_from_timeus_and_rebased():
    # TimeUS is microseconds since boot; the log is rebased to flight-relative,
    # so a single record at 2 s boot-time becomes t=0, with the boot origin kept
    # in meta.start_time_s.
    log = make_flight_log({"ATT": [{"TimeUS": 2_000_000, "Roll": 1.0}]})
    rec = log.get("ATT")[0]
    assert rec["timestamp"] == 0.0
    assert log.meta.start_time_s == 2.0


def test_times_are_flight_relative():
    # Records starting at 100 s boot-time are rebased to start at 0.
    log = make_flight_log(
        {
            "ATT": [
                {"TimeUS": 100_000_000, "Roll": 1.0},
                {"TimeUS": 101_000_000, "Roll": 2.0},
            ]
        }
    )
    t, _ = log.series("ATT", "Roll")
    assert list(t) == [0.0, 1.0]
    assert log.meta.start_time_s == 100.0
    assert abs(log.meta.duration_s - 1.0) < 1e-6


def test_series_aligned_and_skips_missing():
    recs = [
        {"TimeUS": 0, "Roll": 1.0},
        {"TimeUS": 1_000_000},  # missing Roll -> skipped in series
        {"TimeUS": 2_000_000, "Roll": 3.0},
    ]
    log = make_flight_log({"ATT": recs})
    t, v = log.series("ATT", "Roll")
    assert list(v) == [1.0, 3.0]
    assert list(t) == [0.0, 2.0]


def test_field_skips_non_numeric():
    log = make_flight_log({"MSG": [{"TimeUS": 0, "Message": "hello"}]})
    assert log.field("MSG", "Message").size == 0


def test_meta_times_filled():
    recs = build_series(10.0, 10.0, Roll=[1.0] * 11)  # 11 samples over 1s starting at 10s
    log = make_flight_log({"ATT": recs})
    assert log.meta.start_time_s == 10.0
    assert abs(log.meta.duration_s - 1.0) < 1e-6


def test_summary_reports_messages_and_meta():
    log = make_flight_log(
        {"ATT": [{"TimeUS": 0, "Roll": 1.0}], "VIBE": [{"TimeUS": 0, "VibeX": 5.0}]},
        flight_modes=["STABILIZE", "LOITER"],
        max_altitude_m=42.0,
    )
    s = log.summary()
    assert set(s.available_messages) == {"ATT", "VIBE"}
    assert s.message_counts == {"ATT": 1, "VIBE": 1}
    assert s.flight_modes == ["STABILIZE", "LOITER"]
    assert s.max_altitude_m == 42.0
    assert s.vehicle_type == "ArduCopter"


def test_duration_anchored_to_reference_clock():
    # ATT (a reference FC stream) spans ~10 s; a GPS family logged on a different
    # clock (~10000 s, as some pre-2015 logs do) must NOT inflate the duration.
    att = build_series(0.0, 50.0, Roll=[1.0] * 500)  # 0..~10 s
    gps = [{"TimeUS": int((10_000.0 + i * 0.2) * 1_000_000), "Status": 3} for i in range(100)]
    log = make_flight_log({"ATT": att, "GPS": gps})
    assert log.meta.duration_s is not None and log.meta.duration_s < 20.0


def test_corrupt_outlier_timestamp_does_not_inflate_duration():
    # A single garbage ATT timestamp (a multi-day value, as seen in some old logs)
    # must not define the flight duration; the IQR fence rejects it.
    rolls = [1.0] * 200
    recs = build_series(0.0, 50.0, Roll=rolls)  # ~4 s of clean ATT
    recs[100]["TimeUS"] = 3_000_000_000_000  # one corrupt sample (~34 days)
    log = make_flight_log({"ATT": recs})
    assert log.meta.duration_s is not None and log.meta.duration_s < 60.0


def test_empty_log_is_safe():
    log = FlightLog(messages={}, meta=LogMeta())
    assert log.available_messages == set()
    assert isinstance(log.times("ATT"), np.ndarray)
    assert log.summary().integrity == LogIntegrity.OK


def test_armed_intervals_pairs_arm_and_disarm():
    att = build_series(0.0, 10.0, Roll=[0.0] * 300)  # 0..29.9 s
    ev = [{"TimeUS": 2_000_000, "Id": 10}, {"TimeUS": 9_000_000, "Id": 11}, {"TimeUS": 15_000_000, "Id": 10}]
    log = make_flight_log({"ATT": att, "EV": ev})
    assert log.armed_intervals() == [(2.0, 9.0), (15.0, pytest.approx(29.9))]


def test_log_starting_armed_is_armed_from_its_start():
    # LOG_DISARMED=0: the file opens at arming, so the ARM record is often missing
    # and the first edge seen is the disarm.
    att = build_series(0.0, 10.0, Roll=[0.0] * 300)
    log = make_flight_log({"ATT": att, "ARM": [{"TimeUS": 20_000_000, "ArmState": 0}]})
    assert log.armed_intervals() == [(0.0, 20.0)]


def test_no_arm_edges_with_log_disarmed_off_means_whole_log_armed():
    att = build_series(0.0, 10.0, Roll=[0.0] * 300)
    log = make_flight_log({"ATT": att}, params={"LOG_DISARMED": 0})
    assert log.armed_intervals() == [(0.0, pytest.approx(29.9))]
    assert make_flight_log({"ATT": att}, params={"LOG_DISARMED": 1}).armed_intervals() == []


def test_flying_mask_uses_plane_is_flying():
    att = build_series(0.0, 10.0, Roll=[0.0] * 100)
    stat = [
        {"TimeUS": 0, "isFlying": 0},
        {"TimeUS": 4_000_000, "isFlying": 1},
        {"TimeUS": 8_000_000, "isFlying": 0},
    ]
    log = make_flight_log({"ATT": att, "STAT": stat, "EV": [{"TimeUS": 0, "Id": 10}]})
    t = np.array([1.0, 5.0, 9.0])
    assert list(log.flying_mask(t)) == [False, True, False]


def test_records_with_an_int_field_missing_from_some_records():
    log = make_flight_log(
        {"EV": [{"TimeUS": 1_000_000, "Id": 10}, {"TimeUS": 2_000_000, "Id": 11, "Extra": 3}]}
    )
    recs = log.get("EV")
    assert [r["Id"] for r in recs] == [10, 11]
    assert "Extra" not in recs[0] and recs[1]["Extra"] == 3
    assert isinstance(recs[1]["Extra"], int)


def test_series_many_keeps_fields_of_one_record_together():
    recs = build_series(0.0, 10.0, A=[1.0, 2.0, 3.0, 4.0], B=[10.0, float("nan"), 30.0, 40.0])
    log = make_flight_log({"X": recs})
    t, (a, b) = log.series_many("X", ["A", "B"])
    assert a.tolist() == [1.0, 3.0, 4.0]
    assert b.tolist() == [10.0, 30.0, 40.0]
    assert t.tolist() == pytest.approx([0.0, 0.2, 0.3])


def test_an_event_before_the_first_imu_sample_sets_the_time_origin():
    # Logging that starts at arming writes EV ARM just before the first IMU sample.
    log = make_flight_log(
        {"IMU": build_series(100.0, 50.0, AccX=[0.0] * 500), "EV": [{"TimeUS": 99_700_000, "Id": 10}]}
    )
    assert log.times("EV").tolist() == [0.0]
    assert log.times("IMU")[0] == pytest.approx(0.3)
    assert log.armed_window()[0] == 0.0


def test_a_corrupt_zero_timestamp_does_not_move_the_time_origin():
    log = make_flight_log(
        {
            "IMU": build_series(200.0, 50.0, AccX=[0.0] * 500),
            "MSG": [{"TimeUS": 0, "Message": "corrupt"}, {"TimeUS": 201_000_000, "Message": "ok"}],
        }
    )
    assert log.times("IMU")[0] == 0.0
    assert log.meta.duration_s == pytest.approx(9.98)


def test_batches_are_decoded_once_when_threads_read_them_together():
    import threading
    import time

    calls = []

    def loader(_log):
        calls.append(1)
        time.sleep(0.05)
        return ["batch"]

    log = make_flight_log({})
    log._batch_loader = loader
    seen = []
    threads = [threading.Thread(target=lambda: seen.append(list(log.batches))) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert calls == [1]
    assert seen == [["batch"]] * 4
