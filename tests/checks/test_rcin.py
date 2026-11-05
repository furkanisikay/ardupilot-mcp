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


def test_the_loss_reports_the_minimum_inside_the_loss_window():
    # 800 us at boot (before arming), the loss itself sits at 990 us.
    c = [800.0] * 20 + [1500.0] * 180 + [990.0] * 100 + [1500.0] * 100
    log = make_flight_log(
        {"RCIN": build_series(0.0, 10.0, C1=c, C2=c, C3=c, C4=c), "EV": [{"TimeUS": 3_000_000, "Id": 10}]}
    )
    findings = RCInputCheck().run(log)
    assert [f.evidence.samples["min_pwm_us"] for f in findings] == [990.0]


def test_no_rc_receiver_at_all_is_not_a_link_loss():
    # ArduSub logs RCIN while a MAVLink joystick drives it: HAS_VALID_INPUT is never set.
    # No link existed, so none was lost.
    n = 100
    rcin = build_series(0.0, 10.0, C1=[0] * n, C2=[0] * n, C3=[0] * n, C4=[0] * n)
    rci2 = build_series(0.0, 10.0, Flags=[0] * n)
    ev = [{"TimeUS": 0, "Id": 10}]
    log = make_flight_log({"RCIN": rcin, "RCI2": rci2, "EV": ev}, vehicle_kind="sub")
    findings = RCInputCheck().run(log)
    assert [(f.title, f.severity) for f in findings] == [("No RC input in this log", Severity.INFO)]


def test_no_rc_receiver_without_rci2_is_not_a_link_loss():
    n = 100
    rcin = build_series(0.0, 10.0, C1=[0] * n, C2=[0] * n, C3=[0] * n, C4=[0] * n)
    log = make_flight_log({"RCIN": rcin, "EV": [{"TimeUS": 0, "Id": 10}]}, vehicle_kind="sub")
    assert [f.severity for f in RCInputCheck().run(log)] == [Severity.INFO]


def _glitch_log(params):
    # The 145881 forum logs (FrSky Fport): every channel jumps to 875 us for 1 s while
    # RCI2.Flags keeps reporting valid input; the mode channel falls with it.
    n = 200
    c = [1500.0] * 100 + [875.0] * 10 + [1500.0] * 90
    c3 = [1400.0] * 100 + [875.0] * 10 + [1400.0] * 90
    rcin = build_series(0.0, 10.0, C1=c, C2=c, C3=c3, C4=c)
    rci2 = build_series(0.0, 10.0, Flags=[1] * n)
    msgs = {
        "RCIN": rcin,
        "RCI2": rci2,
        "EV": [{"TimeUS": 0, "Id": 10}],
        "MODE": [{"TimeUS": 10_100_000, "Mode": 0}],
    }
    return make_flight_log(msgs, params=params, vehicle_kind="copter")


def test_receiver_sending_its_own_failsafe_values_is_caught_despite_valid_flags():
    findings = RCInputCheck().run(_glitch_log({"FS_THR_ENABLE": 0}))
    assert [(f.title, f.severity) for f in findings] == [("RC signal loss", Severity.CRITICAL)]
    text = findings[0].explanation
    assert "kept reporting valid input" in text and "FS_THR_ENABLE is 0" in text and "mode changed" in text


def test_glitch_with_the_failsafe_on_is_a_warning():
    findings = RCInputCheck().run(_glitch_log({"FS_THR_ENABLE": 1}))
    assert [(f.title, f.severity) for f in findings] == [("RC signal loss", Severity.WARN)]


def test_rc_failsafe_tested_on_the_ground_is_not_an_in_flight_loss():
    # Benchmark log 141676_0: a plane on the ground (isFlying 0) with the transmitter off.
    n = 100
    rcin = build_series(0.0, 10.0, C1=[1500.0] * n, C2=[1500.0] * n, C3=[1000.0] * n, C4=[1500.0] * n)
    rci2 = build_series(0.0, 10.0, Flags=[1] * 30 + [2] * 40 + [1] * 30)
    stat = [{"TimeUS": 0, "isFlying": 0}]
    log = make_flight_log(
        {"RCIN": rcin, "RCI2": rci2, "STAT": stat, "EV": [{"TimeUS": 0, "Id": 10}]}, vehicle_kind="plane"
    )
    assert RCInputCheck().run(log) == []
