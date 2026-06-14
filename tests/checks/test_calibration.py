"""Tests for the sensor-calibration check."""

from __future__ import annotations

from ardupilot_mcp.checks.calibration import LARGE_OFFSET_MGAUSS, CalibrationCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import make_flight_log


def _run(params):
    return CalibrationCheck().run(make_flight_log({}, params=params))


def _titles(findings):
    return [f.title for f in findings]


# -- always runs / empty -------------------------------------------------------


def test_runs_even_with_no_params():
    # requires is empty -> never skipped, even with an otherwise empty log.
    result = CalibrationCheck().execute(make_flight_log({}))
    assert result.status == CheckStatus.RAN
    assert result.findings == []


def test_no_compass_params_no_findings():
    assert _run({}) == []


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("calibration") is not None


# -- compass: large offsets (WARN) ---------------------------------------------


def test_large_primary_offset_warns():
    # magnitude ~800 (480^2+480^2+480^2 = 691200 -> 831).
    findings = _run(
        {
            "COMPASS_OFS_X": 480.0,
            "COMPASS_OFS_Y": 480.0,
            "COMPASS_OFS_Z": 480.0,
            "COMPASS_USE": 1.0,
            "COMPASS_DEV_ID": 12345.0,
        }
    )
    warns = [f for f in findings if f.severity == Severity.WARN]
    assert warns, "expected a WARN for large offsets"
    f = warns[0]
    assert "Compass 1 offsets are large" in f.title
    assert f.evidence.samples["compass1_offset_mag"] > LARGE_OFFSET_MGAUSS


def test_very_large_offset_stays_warn():
    # magnitude > 900 -> still WARN (no separate critical band).
    findings = _run(
        {
            "COMPASS_OFS_X": 600.0,
            "COMPASS_OFS_Y": 600.0,
            "COMPASS_OFS_Z": 600.0,
            "COMPASS_USE": 1.0,
        }
    )
    sev = {f.title: f.severity for f in findings}
    assert any(s == Severity.WARN for s in sev.values())
    assert all(s != Severity.CRITICAL for s in sev.values())


def test_large_secondary_offset_warns():
    findings = _run(
        {
            "COMPASS_OFS2_X": 700.0,
            "COMPASS_OFS2_Y": 0.0,
            "COMPASS_OFS2_Z": 0.0,
            "COMPASS_USE2": 1.0,
            "COMPASS_DEV_ID2": 99.0,
        }
    )
    titles = _titles(findings)
    assert any("Compass 2 offsets are large" in t for t in titles)


# -- compass: small offsets -> nothing -----------------------------------------


def test_small_offset_no_finding():
    # magnitude ~150 -> no finding.
    findings = _run(
        {
            "COMPASS_OFS_X": 86.6,
            "COMPASS_OFS_Y": 86.6,
            "COMPASS_OFS_Z": 86.6,
            "COMPASS_USE": 1.0,
            "COMPASS_DEV_ID": 12345.0,
        }
    )
    assert findings == []


# -- compass: enabled but never calibrated (zero offsets) -> WARN --------------


def test_primary_zero_offsets_enabled_warns():
    findings = _run(
        {
            "COMPASS_OFS_X": 0.0,
            "COMPASS_OFS_Y": 0.0,
            "COMPASS_OFS_Z": 0.0,
            "COMPASS_USE": 1.0,
            "COMPASS_DEV_ID": 12345.0,
        }
    )
    warns = [f for f in findings if f.severity == Severity.WARN]
    assert warns, "expected a WARN for an enabled-but-uncalibrated compass"
    assert "never calibrated" in warns[0].title


def test_primary_zero_offsets_without_devid_still_warns():
    # No DEV_ID logged -> we cannot tell the device is absent, so still flag.
    findings = _run(
        {
            "COMPASS_OFS_X": 0.0,
            "COMPASS_OFS_Y": 0.0,
            "COMPASS_OFS_Z": 0.0,
            "COMPASS_USE": 1.0,
        }
    )
    assert any("never calibrated" in t for t in _titles(findings))


def test_zero_offsets_but_use_zero_no_finding():
    # COMPASS_USE == 0 -> not enabled -> no "uncalibrated" finding.
    findings = _run(
        {
            "COMPASS_OFS_X": 0.0,
            "COMPASS_OFS_Y": 0.0,
            "COMPASS_OFS_Z": 0.0,
            "COMPASS_USE": 0.0,
            "COMPASS_DEV_ID": 12345.0,
        }
    )
    assert findings == []


def test_zero_offsets_but_compass_disabled_no_finding():
    # COMPASS_ENABLE == 0 overrides USE -> not enabled.
    findings = _run(
        {
            "COMPASS_OFS_X": 0.0,
            "COMPASS_OFS_Y": 0.0,
            "COMPASS_OFS_Z": 0.0,
            "COMPASS_USE": 1.0,
            "COMPASS_ENABLE": 0.0,
            "COMPASS_DEV_ID": 12345.0,
        }
    )
    assert findings == []


def test_zero_offsets_absent_primary_device_no_finding():
    # DEV_ID == 0 -> no physical device -> defaulted offsets are not a problem.
    findings = _run(
        {
            "COMPASS_OFS_X": 0.0,
            "COMPASS_OFS_Y": 0.0,
            "COMPASS_OFS_Z": 0.0,
            "COMPASS_USE": 1.0,
            "COMPASS_DEV_ID": 0.0,
        }
    )
    assert findings == []


def test_secondary_zero_offsets_not_flagged():
    # A present-but-uncalibrated SECONDARY compass (very common) must NOT flag,
    # to avoid false positives like real-log hl05 compass 2.
    findings = _run(
        {
            "COMPASS_OFS_X": 100.0,
            "COMPASS_OFS_Y": 0.0,
            "COMPASS_OFS_Z": 0.0,
            "COMPASS_USE": 1.0,
            "COMPASS_DEV_ID": 12345.0,
            "COMPASS_OFS2_X": 0.0,
            "COMPASS_OFS2_Y": 0.0,
            "COMPASS_OFS2_Z": 0.0,
            "COMPASS_USE2": 1.0,
            "COMPASS_DEV_ID2": 67890.0,
        }
    )
    assert findings == []


# -- accelerometer (INFO) ------------------------------------------------------


def test_accel_factory_defaults_info():
    findings = _run(
        {
            "INS_ACCSCAL_X": 1.0,
            "INS_ACCSCAL_Y": 1.0,
            "INS_ACCSCAL_Z": 1.0,
            "INS_ACCOFFS_X": 0.0,
            "INS_ACCOFFS_Y": 0.0,
            "INS_ACCOFFS_Z": 0.0,
        }
    )
    infos = [f for f in findings if f.severity == Severity.INFO]
    assert infos, "expected an INFO for an uncalibrated accelerometer"
    assert "Accelerometer appears uncalibrated" in infos[0].title


def test_accel_calibrated_no_finding():
    findings = _run(
        {
            "INS_ACCSCAL_X": 1.006,
            "INS_ACCSCAL_Y": 0.996,
            "INS_ACCSCAL_Z": 0.994,
            "INS_ACCOFFS_X": -0.02,
            "INS_ACCOFFS_Y": -0.21,
            "INS_ACCOFFS_Z": 0.25,
        }
    )
    assert findings == []


def test_accel_partial_params_no_finding():
    # Missing one offset param -> cannot evaluate -> nothing.
    findings = _run(
        {
            "INS_ACCSCAL_X": 1.0,
            "INS_ACCSCAL_Y": 1.0,
            "INS_ACCSCAL_Z": 1.0,
            "INS_ACCOFFS_X": 0.0,
            "INS_ACCOFFS_Y": 0.0,
        }
    )
    assert findings == []
