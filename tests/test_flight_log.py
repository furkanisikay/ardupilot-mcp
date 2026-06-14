"""Tests for the FlightLog domain model."""

from __future__ import annotations

import numpy as np

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
