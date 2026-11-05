"""Tests for the Plane landing-flare check."""

from __future__ import annotations

from ardupilot_mcp.checks.landing import FlareCheck
from ardupilot_mcp.model import CheckStatus, Severity
from ardupilot_mcp.orchestrator import diagnose
from tests.helpers import build_series, make_flight_log


def _landing(sink_after_flare: float, *, flare_text: str = "Flare 2.0m sink=2.00 speed=15.0 dist=20.0"):
    # TECS at 10 Hz for 20 s; the flare is announced at 10 s and the descent stops at 11 s.
    dh = [-2.0] * 100 + [-sink_after_flare] * 10 + [0.5] + [0.0] * 89
    tecs = build_series(0.0, 10.0, dh=dh)
    imu = build_series(0.0, 10.0, AccZ=[-9.8] * 109 + [-60.0] * 3 + [-9.8] * 88)
    msg = [{"TimeUS": 10_000_000, "Message": flare_text}]
    return make_flight_log(
        {"TECS": tecs, "MSG": msg, "IMU": imu},
        vehicle_type="ArduPlane",
        vehicle_kind="plane",
        params={"LAND_FLARE_ALT": 2.0, "LAND_FLARE_SEC": 1.0, "TECS_LAND_SINK": 0.25},
    )


def test_skipped_without_tecs():
    log = make_flight_log({"MSG": []}, vehicle_type="ArduPlane", vehicle_kind="plane")
    assert FlareCheck().execute(log).status == CheckStatus.SKIPPED


def test_a_flare_that_arrested_the_sink_is_clean():
    assert FlareCheck().run(_landing(0.2)) == []


def test_a_flare_that_left_the_sink_in_place_is_a_warning():
    f = FlareCheck().run(_landing(1.9))
    assert [x.severity for x in f] == [Severity.WARN]
    s = f[0].evidence.samples
    assert s["touchdown_sink_ms"] == 1.9 and s["sink_kept_ratio"] == 0.95
    assert s["touchdown_s"] == 11.0 and s["final_touchdown_s"] == 11.0
    assert s["touchdown_jolt_ms2"] > 40
    assert "TECS_LAND_SINK 0.25" in f[0].explanation and "LAND_FLARE_SEC 1" in f[0].explanation


def test_a_flare_without_a_descent_is_not_judged():
    assert FlareCheck().run(_landing(1.9, flare_text="Flare 24.0m sink=-0.38 speed=15.0 dist=20.0")) == []


def test_copter_is_not_judged():
    log = _landing(1.9)
    log.meta.vehicle_kind = "copter"
    assert FlareCheck().execute(log).status == CheckStatus.SKIPPED


def test_the_final_hard_touchdown_is_the_impact():
    rep = diagnose(_landing(1.9))
    assert rep.incident is not None and rep.incident.impact_s == 11.0
    assert rep.incident.impact_basis == "hard touchdown after the flare"
