"""Tests for the attitude tracking check."""

from __future__ import annotations

from ardupilot_mcp.checks.attitude import AttitudeTrackingCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _log(att_records, *, rcou_records=None):
    messages = {"ATT": att_records}
    if rcou_records is not None:
        messages["RCOU"] = rcou_records
    return make_flight_log(messages)


def test_skipped_when_no_att():
    log = make_flight_log({})
    result = AttitudeTrackingCheck().execute(log)
    assert result.status == CheckStatus.SKIPPED
    assert "ATT" in (result.skipped_reason or "")


def test_clean_flight_no_findings():
    # Perfect tracking: actual == desired on every axis -> no findings.
    n = 100
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[5.0] * n,
        Roll=[5.0] * n,
        DesPitch=[-3.0] * n,
        Pitch=[-3.0] * n,
        DesYaw=[90.0] * n,
        Yaw=[90.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    assert findings == []


def test_small_tracking_error_no_findings():
    # A few degrees of lag everywhere is normal -> nothing flagged.
    n = 100
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[10.0] * n,
        Roll=[12.0] * n,  # 2 deg
        DesPitch=[0.0] * n,
        Pitch=[5.0] * n,  # 5 deg
        DesYaw=[0.0] * n,
        Yaw=[8.0] * n,  # 8 deg
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    assert findings == []


def test_critical_sustained_roll_divergence():
    # Roll diverges from DesRoll by ~35 deg for 2 s (20 samples @ 10 Hz),
    # then recovers -> CRITICAL roll finding.
    n = 50
    des = [0.0] * n
    roll = [0.0] * 10 + [35.0] * 20 + [0.0] * 20  # 2 s at 35 deg error
    recs = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "roll" in f.title.lower()]
    assert crit, "expected a critical roll finding"
    f = crit[0]
    assert f.evidence.samples["peak_err_deg_roll"] >= 25
    assert f.evidence.samples["worst_duration_s_roll"] >= 1.0
    # Worst window should sit inside the divergence (1.0 s .. 2.9 s region).
    assert f.evidence.time_start_s is not None
    assert f.evidence.time_start_s >= 1.0


def test_warn_brief_spike():
    # 35 deg error but for only 0.3 s (3 samples @ 10 Hz) -> WARN, not CRITICAL.
    n = 50
    des = [0.0] * n
    roll = [0.0] * 20 + [35.0] * 3 + [0.0] * 27
    recs = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    roll_findings = [f for f in findings if "roll" in f.title.lower()]
    assert roll_findings, "expected a roll finding"
    assert roll_findings[0].severity == Severity.WARN
    assert not any(f.severity == Severity.CRITICAL for f in findings)
    assert roll_findings[0].evidence.samples["peak_err_deg_roll"] >= 30


def test_warn_sustained_moderate_band():
    # ~20 deg sustained error for 2 s -> WARN (15-25 band), not CRITICAL.
    n = 50
    des = [0.0] * n
    pitch = [0.0] * 10 + [20.0] * 20 + [0.0] * 20
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=[0.0] * n,
        DesPitch=des,
        Pitch=pitch,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    pitch_findings = [f for f in findings if "pitch" in f.title.lower()]
    assert pitch_findings, "expected a pitch finding"
    assert pitch_findings[0].severity == Severity.WARN
    assert not any(f.severity == Severity.CRITICAL for f in findings)
    assert 15 <= pitch_findings[0].evidence.samples["peak_err_deg_pitch"] <= 25


def test_yaw_error_is_wrapped():
    # Desired yaw 350, actual yaw 5 -> naive diff is -345 but wrapped is +15,
    # which is small -> NO finding. Confirms circular wrapping works.
    n = 100
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=[0.0] * n,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[350.0] * n,
        Yaw=[5.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    assert findings == [], "wrapped yaw error of 15 deg must not trigger a finding"


def test_yaw_wrapped_large_error_critical():
    # Desired 0, actual 200 -> wrapped error is -160 (abs 160), sustained 2 s.
    n = 50
    des = [0.0] * n
    yaw = [0.0] * 10 + [200.0] * 20 + [0.0] * 20
    recs = build_series(
        0.0,
        10.0,
        DesRoll=[0.0] * n,
        Roll=[0.0] * n,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=des,
        Yaw=yaw,
    )
    findings = AttitudeTrackingCheck().run(_log(recs))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "yaw" in f.title.lower()]
    assert crit, "expected a critical yaw finding"
    # 200 wrapped -> 160, not 200.
    assert crit[0].evidence.samples["peak_err_deg_yaw"] == 160.0


def test_motor_saturation_enrichment():
    # Sustained critical roll divergence WITH saturated motors -> the explanation
    # mentions underpowered, and the sample flag is set.
    n = 50
    des = [0.0] * n
    roll = [0.0] * 10 + [35.0] * 20 + [0.0] * 20
    att = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    rcou = build_series(
        0.0,
        10.0,
        C1=[1980.0] * n,
        C2=[1975.0] * n,
        C3=[1990.0] * n,
        C4=[1985.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(att, rcou_records=rcou))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "roll" in f.title.lower()]
    assert crit, "expected a critical roll finding"
    f = crit[0]
    assert f.evidence.samples.get("motors_saturated") == 1.0
    # The enrichment adds a distinct RCOU sentence on top of the base message.
    assert "rcou" in f.explanation.lower()
    assert "saturated" in f.explanation.lower()
    assert "RCOU" in f.evidence.message_types


def test_no_motor_saturation_no_enrichment():
    # Same divergence but motors well below saturation -> no enrichment.
    n = 50
    des = [0.0] * n
    roll = [0.0] * 10 + [35.0] * 20 + [0.0] * 20
    att = build_series(
        0.0,
        10.0,
        DesRoll=des,
        Roll=roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )
    rcou = build_series(
        0.0,
        10.0,
        C1=[1500.0] * n,
        C2=[1500.0] * n,
        C3=[1500.0] * n,
        C4=[1500.0] * n,
    )
    findings = AttitudeTrackingCheck().run(_log(att, rcou_records=rcou))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "roll" in f.title.lower()]
    assert crit
    assert "motors_saturated" not in crit[0].evidence.samples
    # No RCOU enrichment sentence when motors are not saturated.
    assert "rcou" not in crit[0].explanation.lower()
    assert "RCOU" not in crit[0].evidence.message_types


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("attitude") is not None
