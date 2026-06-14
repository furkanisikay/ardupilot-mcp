"""Tests for the RC input check."""

from __future__ import annotations

from ardupilot_mcp.checks.rcin import RCInputCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _log(records):
    return make_flight_log({"RCIN": records})


def test_skipped_without_rcin():
    result = RCInputCheck().execute(make_flight_log({}))
    assert result.status == CheckStatus.SKIPPED


def test_normal_flight_no_finding():
    # Throttle low-ish, but roll/pitch/yaw centred — never all low together.
    n = 200
    recs = build_series(
        0.0,
        10.0,
        C1=[1500.0] * n,
        C2=[1500.0] * n,
        C3=[1100.0] * n,
        C4=[1500.0] * n,
    )
    assert RCInputCheck().run(_log(recs)) == []


def test_rc_loss_to_end_is_critical():
    # All four channels healthy, then all drop to ~990 us for the last 3 s.
    good, bad = 170, 30  # 30 samples @10Hz = 3 s

    def ch(hi):
        return [hi] * good + [990.0] * bad

    recs = build_series(
        0.0,
        10.0,
        C1=ch(1500.0),
        C2=ch(1500.0),
        C3=ch(1100.0),
        C4=ch(1500.0),
    )
    findings = RCInputCheck().run(_log(recs))
    assert findings, "expected an RC signal-loss finding"
    f = findings[0]
    assert f.severity == Severity.CRITICAL
    assert f.evidence.samples["lost_to_end"] == 1.0
    assert f.evidence.samples["min_pwm_us"] <= 1000


def test_brief_dropout_is_warning_not_critical():
    # A short (~0.7 s) all-low dropout mid-flight that recovers -> WARN.
    pattern_lo = 7  # ~0.7 s at 10 Hz
    n = 200
    c = [1500.0] * 80 + [990.0] * pattern_lo + [1500.0] * (n - 80 - pattern_lo)
    c3 = [1100.0] * 80 + [990.0] * pattern_lo + [1100.0] * (n - 80 - pattern_lo)
    recs = build_series(0.0, 10.0, C1=c, C2=c, C3=c3, C4=c)
    findings = RCInputCheck().run(_log(recs))
    assert findings
    assert findings[0].severity == Severity.WARN
    assert findings[0].evidence.samples["lost_to_end"] == 0.0


def test_throttle_alone_low_is_not_rc_loss():
    # Only throttle low (normal idle/landing); others centred -> no finding.
    n = 150
    recs = build_series(
        0.0,
        10.0,
        C1=[1500.0] * n,
        C2=[1500.0] * n,
        C3=[980.0] * n,
        C4=[1500.0] * n,
    )
    assert RCInputCheck().run(_log(recs)) == []


def test_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("rcin") is not None
