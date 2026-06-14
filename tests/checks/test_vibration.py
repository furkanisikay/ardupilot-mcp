"""Tests for the vibration check."""

from __future__ import annotations

from ardupilot_mcp.checks.vibration import VibrationCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _log(vibe_records):
    return make_flight_log({"VIBE": vibe_records})


def test_skipped_when_no_vibe():
    log = make_flight_log({})
    result = VibrationCheck().execute(log)
    assert result.status == CheckStatus.SKIPPED
    assert "VIBE" in (result.skipped_reason or "")


def test_clean_flight_no_findings():
    recs = build_series(
        0.0,
        10.0,
        VibeX=[8.0] * 50,
        VibeY=[7.0] * 50,
        VibeZ=[12.0] * 50,
        Clip0=[0] * 50,
        Clip1=[0] * 50,
        Clip2=[0] * 50,
    )
    findings = VibrationCheck().run(_log(recs))
    assert findings == []


def test_critical_high_vibration():
    recs = build_series(
        0.0,
        10.0,
        VibeX=[10.0] * 50,
        VibeY=[10.0] * 50,
        VibeZ=[70.0] * 50,
        Clip0=[0] * 50,
        Clip1=[0] * 50,
        Clip2=[0] * 50,
    )
    findings = VibrationCheck().run(_log(recs))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "VibeZ" in f.title]
    assert crit, "expected a critical VibeZ finding"
    assert crit[0].evidence.samples["max_VibeZ"] >= 60


def test_warn_sustained_moderate_vibration():
    # Half the flight above 30 but below 60 -> warning, not critical.
    vz = [40.0] * 30 + [10.0] * 20
    recs = build_series(
        0.0,
        10.0,
        VibeX=[5.0] * 50,
        VibeY=[5.0] * 50,
        VibeZ=vz,
        Clip0=[0] * 50,
        Clip1=[0] * 50,
        Clip2=[0] * 50,
    )
    findings = VibrationCheck().run(_log(recs))
    sev = {f.title: f.severity for f in findings}
    assert any(s == Severity.WARN for s in sev.values())
    assert not any(s == Severity.CRITICAL for s in sev.values())


def test_clipping_detected():
    # Clip0 ramps up -> clipping finding.
    clip0 = list(range(0, 150, 3))  # 50 samples, ends at 147
    recs = build_series(
        0.0,
        10.0,
        VibeX=[10.0] * 50,
        VibeY=[10.0] * 50,
        VibeZ=[10.0] * 50,
        Clip0=clip0,
        Clip1=[0] * 50,
        Clip2=[0] * 50,
    )
    findings = VibrationCheck().run(_log(recs))
    clip = [f for f in findings if "clipping" in f.title.lower()]
    assert clip, "expected a clipping finding"
    assert clip[0].severity == Severity.CRITICAL
    assert clip[0].evidence.samples["total_Clip0"] > 100


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("vibration") is not None
