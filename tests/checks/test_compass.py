"""Tests for the compass check."""

from __future__ import annotations

from ardupilot_mcp.checks.compass import CompassCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _log(mag_records, *, params=None):
    return make_flight_log({"MAG": mag_records}, params=params)


def test_skipped_when_no_mag():
    log = make_flight_log({})
    result = CompassCheck().execute(log)
    assert result.status == CheckStatus.SKIPPED
    assert "MAG" in (result.skipped_reason or "")


def test_too_few_samples_no_findings():
    # Below MIN_SAMPLES: statistics are not trustworthy, so emit nothing even
    # though the few points vary wildly.
    recs = build_series(
        0.0,
        5.0,
        MagX=[100.0, 900.0, 100.0],
        MagY=[0.0, 0.0, 0.0],
        MagZ=[0.0, 0.0, 0.0],
    )
    assert CompassCheck().run(_log(recs)) == []


def test_clean_steady_field_no_findings():
    # Steady ~500 mGauss magnitude (300/400/0 -> 500) with tiny jitter.
    n = 50
    mx = [300.0] * n
    my = [400.0] * n
    mz = [0.0] * n
    recs = build_series(0.0, 10.0, MagX=mx, MagY=my, MagZ=mz)
    assert CompassCheck().run(_log(recs)) == []


def test_warn_moderate_variation():
    # Magnitude alternates 450 <-> 650 along X only.
    # mean ~550, cv ~0.18 (< WARN_CV) but range_ratio = 200/550 ~ 0.36...
    # push it just over the WARN_RANGE_RATIO via a wider but bounded swing.
    # Use 400 <-> 700: mean 550, pp 300, range_ratio ~0.55 -> tune to clear 0.60.
    n = 50
    # Alternate to keep std moderate (cv stays between WARN_CV and CRIT_CV is
    # hard with a pure two-level signal, so design for cv in [0.30, 0.60)).
    mx = []
    for i in range(n):
        mx.append(400.0 if i % 2 == 0 else 800.0)
    recs = build_series(0.0, 10.0, MagX=mx, MagY=[0.0] * n, MagZ=[0.0] * n)
    findings = CompassCheck().run(_log(recs))
    assert findings, "expected an instability finding"
    f = findings[0]
    assert f.severity == Severity.WARN
    assert f.title == "Unstable compass field strength"
    # mean of {400,800} = 600; cv = 200/600 = 0.333 -> WARN band.
    assert 0.30 < f.evidence.samples["cv"] <= 0.60
    assert f.evidence.samples["min_mag"] == 400.0
    assert f.evidence.samples["max_mag"] == 800.0
    assert f.evidence.time_start_s == 0.0


def test_critical_large_variation():
    # Magnitude swings 200 <-> 1000 -> very high CV (>0.60).
    n = 50
    mx = [200.0 if i % 2 == 0 else 1000.0 for i in range(n)]
    recs = build_series(0.0, 10.0, MagX=mx, MagY=[0.0] * n, MagZ=[0.0] * n)
    findings = CompassCheck().run(_log(recs))
    crit = [f for f in findings if f.severity == Severity.CRITICAL]
    assert crit, "expected a critical instability finding"
    f = crit[0]
    # mean 600, std 400 -> cv ~0.667 > CRIT_CV.
    assert f.evidence.samples["cv"] > 0.60
    assert f.evidence.samples["max_mag"] == 1000.0
    assert f.evidence.message_types == ["MAG"]


def test_swing_400_to_900_flags():
    # Assignment's explicit scenario: magnitude swinging from 400 to 900.
    n = 60
    mx = [400.0 if i % 2 == 0 else 900.0 for i in range(n)]
    recs = build_series(0.0, 10.0, MagX=mx, MagY=[0.0] * n, MagZ=[0.0] * n)
    findings = CompassCheck().run(_log(recs))
    assert findings, "expected a finding for a 400->900 swing"
    assert findings[0].severity in (Severity.WARN, Severity.CRITICAL)
    assert findings[0].evidence.samples["min_mag"] == 400.0
    assert findings[0].evidence.samples["max_mag"] == 900.0


# Compass hard-iron offsets are now owned by the 'calibration' check (see
# tests/checks/test_calibration.py); the compass check focuses on field stability.


def test_small_offset_no_info_finding():
    # Steady field + small offsets -> no offset finding at all.
    n = 30
    recs = build_series(0.0, 10.0, MagX=[300.0] * n, MagY=[400.0] * n, MagZ=[0.0] * n)
    params = {"COMPASS_OFS_X": 50.0, "COMPASS_OFS_Y": 40.0, "COMPASS_OFS_Z": 10.0}
    assert CompassCheck().run(_log(recs, params=params)) == []


def test_missing_offset_params_skips_offset_check():
    # Only one of the three offset params present -> offset check stays silent.
    n = 30
    recs = build_series(0.0, 10.0, MagX=[300.0] * n, MagY=[400.0] * n, MagZ=[0.0] * n)
    params = {"COMPASS_OFS_X": 5000.0}
    assert CompassCheck().run(_log(recs, params=params)) == []


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("compass") is not None
