"""Tests for the vibration check."""

from __future__ import annotations

from ardupilot_mcp.checks.vibration import VibrationCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, interleave_instances, make_flight_log


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
    assert clip[0].evidence.samples["total_clips"] > 100


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("vibration") is not None


# ---- modern (4.x) layout: one VIBE stream per IMU with a single Clip counter ----


def _modern(imu0: dict, imu1: dict, n: int = 50) -> list[dict]:
    base = {"VibeX": [5.0] * n, "VibeY": [5.0] * n, "VibeZ": [8.0] * n, "Clip": [0] * n}
    return interleave_instances(
        "IMU",
        {
            0: build_series(0.0, 10.0, **{**base, **imu0}),
            1: build_series(0.0, 10.0, **{**base, **imu1}),
        },
    )


def test_modern_clip_counter_detected_per_imu():
    recs = _modern({"Clip": list(range(0, 250, 5))}, {})
    findings = VibrationCheck().run(_log(recs))
    clip = [f for f in findings if "clipping" in f.title.lower()]
    assert len(clip) == 1
    assert clip[0].title == "Accelerometer clipping (IMU0)"
    assert clip[0].severity == Severity.CRITICAL
    assert clip[0].evidence.samples["total_clips"] == 245


def test_modern_clip_counter_reset_is_not_negative():
    # A reboot resets the counter: 0..40 then 0..10 -> 50 clips, not -30.
    counts = list(range(0, 41, 2)) + list(range(0, 11)) + [10] * 18
    recs = _modern({"Clip": counts}, {})
    clip = [f for f in VibrationCheck().run(_log(recs)) if "clipping" in f.title.lower()]
    assert clip[0].evidence.samples["total_clips"] == 50
    assert clip[0].severity == Severity.WARN


def test_modern_vibration_secondary_imu_is_demoted():
    # A noisy *secondary* IMU (e.g. the Cube's deliberately unisolated IMU3) alone
    # must not produce a CRITICAL; it is reported one level lower and labelled.
    recs = _modern({}, {"VibeZ": [70.0] * 50})
    findings = [f for f in VibrationCheck().run(_log(recs)) if f.title.startswith("Elevated VibeZ")]
    assert len(findings) == 1
    assert findings[0].title == "Elevated VibeZ vibration (IMU1)"
    assert findings[0].severity == Severity.WARN
    assert findings[0].evidence.samples["imu"] == 1


def test_modern_vibration_primary_imu_decides_severity():
    recs = _modern({"VibeZ": [70.0] * 50}, {})
    findings = [f for f in VibrationCheck().run(_log(recs)) if f.title.startswith("Elevated VibeZ")]
    assert [f.severity for f in findings] == [Severity.CRITICAL]
    assert findings[0].title == "Elevated VibeZ vibration (IMU0)"


def test_secondary_imu_clipping_is_demoted():
    recs = _modern({}, {"Clip": list(range(0, 250, 5))})
    clip = [f for f in VibrationCheck().run(_log(recs)) if "clipping" in f.title.lower()]
    assert clip and clip[0].title == "Accelerometer clipping (IMU1)"
    assert clip[0].severity == Severity.WARN
