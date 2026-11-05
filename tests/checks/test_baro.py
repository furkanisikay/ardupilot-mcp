"""Tests for the barometer-follows-throttle check."""

from __future__ import annotations

import numpy as np

from ardupilot_mcp.checks.baro import BaroThrustCheck
from ardupilot_mcp.model import Severity
from tests.helpers import build_series, make_flight_log

N = 1200  # 120 s at 10 Hz


def _log(baro_alt, gps_alt, throttle):
    msgs = {
        "GPS": build_series(0.0, 10.0, Alt=list(gps_alt), Status=[3] * N),
        "BARO": build_series(0.0, 10.0, Alt=list(baro_alt)),
        "CTUN": build_series(0.0, 10.0, ThO=list(throttle)),
        "EV": [{"TimeUS": 0, "Id": 10}],
    }
    return make_flight_log(msgs, vehicle_kind="copter")


def _flight(seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(N) / 10.0
    height = 20 + 3 * np.sin(t / 7.0)  # real height, the same for both sensors
    throttle = 0.4 + 0.15 * np.sign(np.sin(t / 3.0))  # punches up and down
    return t, height, throttle, rng


def test_barometer_that_follows_the_throttle_warns():
    # A cinewhoop's duct airflow: the baro reads ~15 m higher per full throttle (forum 145480).
    t, h, thr, rng = _flight()
    baro = h + 15.0 * thr + rng.normal(0, 0.3, N)
    gps = h + rng.normal(0, 0.8, N)
    f = BaroThrustCheck().run(_log(baro, gps, thr))
    assert [x.severity for x in f] == [Severity.WARN]
    assert f[0].evidence.samples["correlation"] > 0.6


def test_barometer_drifting_slowly_from_gps_is_fine():
    # Weather or temperature drift between the sensors is no thrust effect.
    t, h, thr, rng = _flight()
    baro = h + 0.1 * t + rng.normal(0, 0.3, N)
    gps = h + rng.normal(0, 0.8, N)
    assert BaroThrustCheck().run(_log(baro, gps, thr)) == []


def test_without_a_gps_fix_nothing_is_judged():
    # No 3D fix: no independent height to compare the barometer with.
    t, h, thr, rng = _flight()
    msgs = {
        "GPS": build_series(0.0, 10.0, Alt=list(h), Status=[1] * N),
        "BARO": build_series(0.0, 10.0, Alt=list(h + 15.0 * thr)),
        "CTUN": build_series(0.0, 10.0, ThO=list(thr)),
        "EV": [{"TimeUS": 0, "Id": 10}],
    }
    assert BaroThrustCheck().run(make_flight_log(msgs, vehicle_kind="copter")) == []


def test_gps_lag_in_hard_climbs_is_not_a_thrust_effect():
    # Throttle punches drive climbs; a GPS that lags 1 s makes baro - GPS follow the climb rate,
    # and so the throttle, with no airflow on the barometer at all.
    t = np.arange(N) / 10.0
    thr = 0.4 + 0.2 * np.sign(np.sin(t / 4.0))
    climb = 4.0 * (thr - 0.4)
    h = 20 + np.cumsum(climb) / 10.0
    gps = np.interp(t - 1.0, t, h)  # 1 s late
    assert BaroThrustCheck().run(_log(h, gps, thr)) == []
