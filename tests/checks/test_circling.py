"""Tests for the toilet-bowl (circling in position hold) check."""

from __future__ import annotations

import numpy as np

from ardupilot_mcp.checks.circling import CirclingCheck
from ardupilot_mcp.model import Severity
from tests.helpers import build_series, make_flight_log


def _log(err_n, err_e, mode=5, kind="copter", tgt_n=None, tgt_e=None, tv=0.0):
    n = len(err_n)
    tn = list(tgt_n) if tgt_n is not None else [0.0] * n
    te = list(tgt_e) if tgt_e is not None else [0.0] * n
    pscn = build_series(0.0, 10.0, TPN=tn, PN=[a + b for a, b in zip(tn, err_n, strict=True)], TVN=[tv] * n)
    psce = build_series(0.0, 10.0, TPE=te, PE=[a + b for a, b in zip(te, err_e, strict=True)], TVE=[0.0] * n)
    msgs = {
        "PSCN": pscn,
        "PSCE": psce,
        "MODE": [{"TimeUS": 0, "Mode": mode, "ModeNum": mode, "Rsn": 1}],
        "EV": [{"TimeUS": 0, "Id": 10}],
    }
    return make_flight_log(msgs, vehicle_kind=kind)


def _orbit(turns: float, radius: float, n: int = 600):
    a = np.linspace(0.0, turns * 2 * np.pi, n)
    return radius * np.cos(a), radius * np.sin(a)


def test_orbiting_the_target_in_loiter_is_toilet_bowling():
    n, e = _orbit(3.0, 1.5)
    f = CirclingCheck().run(_log(n, e))
    assert [x.severity for x in f] == [Severity.WARN]
    assert f[0].evidence.samples["turns"] >= 2.5
    assert "clockwise" in f[0].explanation


def test_position_noise_around_the_target_is_not_circling():
    rng = np.random.default_rng(1)
    n, e = rng.normal(0, 0.3, 600), rng.normal(0, 0.3, 600)
    assert CirclingCheck().run(_log(n, e)) == []


def test_large_wandering_without_a_direction_is_not_circling():
    # Drifting out and back (a gust, a slow GPS) without orbiting: no net rotation.
    t = np.linspace(0, 60, 600)
    n, e = 2.0 * np.sin(t / 10.0), 0.5 * np.ones(600)
    assert CirclingCheck().run(_log(n, e)) == []


def test_circling_outside_a_hold_mode_is_not_judged():
    n, e = _orbit(3.0, 1.5)
    assert CirclingCheck().run(_log(n, e, mode=0)) == []  # STABILIZE


def test_quadplane_qloiter_is_judged():
    n, e = _orbit(3.0, 1.0)
    f = CirclingCheck().run(_log(n, e, mode=19, kind="plane"))
    assert len(f) == 1


def test_pilot_flying_circles_in_loiter_is_not_a_toilet_bowl():
    # The pilot moves the target around a 30 m circle at 5 m/s; the vehicle trails it by 0.8 m,
    # so the error turns with the path. The target was not still: nothing to report.
    a = np.linspace(0.0, 2.4 * 2 * np.pi, 600)
    tgt_n, tgt_e = 30 * np.cos(a), 30 * np.sin(a)
    err_n, err_e = -0.8 * np.cos(a + np.pi / 2), -0.8 * np.sin(a + np.pi / 2)
    assert CirclingCheck().run(_log(err_n, err_e, tgt_n=tgt_n, tgt_e=tgt_e, tv=5.0)) == []


def test_a_dropped_psce_record_does_not_shift_the_pairing():
    n, e = _orbit(3.0, 1.5)
    size = len(n)
    pscn = build_series(0.0, 10.0, TPN=[0.0] * size, PN=list(n), TVN=[0.0] * size)
    psce = build_series(0.0, 10.0, TPE=[0.0] * size, PE=list(e), TVE=[0.0] * size)
    del psce[100]  # one record lost: index pairing would be off by one from here on
    msgs = {
        "PSCN": pscn,
        "PSCE": psce,
        "MODE": [{"TimeUS": 0, "Mode": 5, "ModeNum": 5, "Rsn": 1}],
        "EV": [{"TimeUS": 0, "Id": 10}],
    }
    f = CirclingCheck().run(make_flight_log(msgs, vehicle_kind="copter"))
    assert len(f) == 1 and f[0].evidence.samples["turns"] >= 2.5
