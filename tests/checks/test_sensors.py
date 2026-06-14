"""Tests for the sensor presence & health check."""

from __future__ import annotations

from ardupilot_mcp.checks.sensors import SensorsCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _run(messages=None, *, params=None):
    log = make_flight_log(messages or {}, params=params)
    return SensorsCheck().run(log)


# -- always runs (requires is empty) -----------------------------------------


def test_never_skipped_even_on_empty_log():
    # requires=set() => the check always runs, never SKIPPED.
    log = make_flight_log({})
    result = SensorsCheck().execute(log)
    assert result.status == CheckStatus.RAN


def test_empty_log_no_findings():
    assert _run({}) == []


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("sensors") is not None


# -- rangefinder -------------------------------------------------------------


def test_rangefinder_configured_no_data_warns():
    findings = _run({}, params={"RNGFND_TYPE": 10})
    rng = [f for f in findings if "Rangefinder" in f.title]
    assert len(rng) == 1
    assert rng[0].severity == Severity.WARN
    assert rng[0].evidence.samples["rngfnd_type"] == 10


def test_rangefinder_numbered_param_configured_no_data_warns():
    # Modern firmware uses RNGFND1_TYPE (as in ov09/ov10 with type 30).
    findings = _run({}, params={"RNGFND1_TYPE": 30})
    rng = [f for f in findings if "Rangefinder" in f.title]
    assert len(rng) == 1
    assert rng[0].severity == Severity.WARN
    assert rng[0].evidence.samples["rngfnd_type"] == 30


def test_rangefinder_configured_with_rfnd_no_finding():
    rfnd = build_series(0.0, 5.0, Dist=[1.2] * 10)
    findings = _run({"RFND": rfnd}, params={"RNGFND_TYPE": 10})
    rng = [f for f in findings if "Rangefinder" in f.title]
    assert rng == []


def test_rangefinder_configured_with_legacy_rngfnd_no_finding():
    rngfnd = build_series(0.0, 5.0, Dist1=[1.2] * 10)
    findings = _run({"RNGFND": rngfnd}, params={"RNGFND1_TYPE": 30})
    rng = [f for f in findings if "Rangefinder" in f.title]
    assert rng == []


def test_rangefinder_not_configured_no_finding():
    # TYPE = 0 (present but disabled) -> not configured.
    findings = _run({}, params={"RNGFND_TYPE": 0, "RNGFND1_TYPE": 0})
    assert [f for f in findings if "Rangefinder" in f.title] == []


def test_rangefinder_absent_param_no_finding():
    assert _run({}, params={}) == []


# -- GPS ---------------------------------------------------------------------


def test_gps_configured_no_data_warns():
    findings = _run({}, params={"GPS_TYPE": 1})
    gps = [f for f in findings if "GPS" in f.title]
    assert len(gps) == 1
    assert gps[0].severity == Severity.WARN
    assert gps[0].evidence.samples["gps_type"] == 1


def test_gps_configured_with_data_no_finding():
    gps_recs = build_series(0.0, 5.0, Status=[3] * 10, NSats=[12] * 10)
    findings = _run({"GPS": gps_recs}, params={"GPS_TYPE": 1})
    assert [f for f in findings if "GPS" in f.title] == []


def test_gps_not_configured_no_finding():
    findings = _run({}, params={"GPS_TYPE": 0})
    assert [f for f in findings if "GPS" in f.title] == []


# -- compass health ----------------------------------------------------------


def test_compass_all_healthy_no_finding():
    mag = build_series(0.0, 10.0, Health=[1] * 50, MagX=[100.0] * 50)
    findings = _run({"MAG": mag})
    assert [f for f in findings if "Compass" in f.title] == []


def test_compass_half_unhealthy_warns():
    # 50% Health=0 -> well above the 20% threshold.
    health = [1] * 25 + [0] * 25
    mag = build_series(0.0, 10.0, Health=health, MagX=[100.0] * 50)
    findings = _run({"MAG": mag})
    comp = [f for f in findings if "Compass" in f.title]
    assert len(comp) == 1
    assert comp[0].severity == Severity.WARN
    assert comp[0].evidence.samples["unhealthy_fraction"] >= 0.20
    assert comp[0].evidence.samples["mag_samples"] == 50


def test_compass_just_below_threshold_no_finding():
    # 10% unhealthy (<= 20%) -> no finding (conservative).
    health = [0] * 5 + [1] * 45
    mag = build_series(0.0, 10.0, Health=health, MagX=[100.0] * 50)
    findings = _run({"MAG": mag})
    assert [f for f in findings if "Compass" in f.title] == []


def test_compass_no_mag_no_finding():
    assert [f for f in _run({}) if "Compass" in f.title] == []


# -- combined ----------------------------------------------------------------


def test_multiple_faults_each_reported_once():
    health = [0] * 30 + [1] * 20  # 60% unhealthy
    mag = build_series(0.0, 10.0, Health=health, MagX=[100.0] * 50)
    findings = _run({"MAG": mag}, params={"RNGFND1_TYPE": 30, "GPS_TYPE": 1})
    titles = [f.title for f in findings]
    assert any("Rangefinder" in t for t in titles)
    assert any("GPS" in t for t in titles)
    assert any("Compass" in t for t in titles)
    assert len(findings) == 3
    assert all(f.severity == Severity.WARN for f in findings)
