"""Tests for the flight-controller health check (watchdog, MON, internal errors)."""

from __future__ import annotations

from ardupilot_mcp.checks.health import HealthCheck, health_owns
from ardupilot_mcp.checks.prearm import StartupMessagesCheck
from ardupilot_mcp.model import Severity
from tests.helpers import build_series, make_flight_log


def _wdog(ft: int, fl: int = 122, tsk: int = -2) -> list[dict]:
    return [
        {"TimeUS": int(t * 1e6), "Tsk": tsk, "IE": 0, "FL": fl, "FT": ft, "TN": "Ardu"}
        for t in (1.4, 2.4, 3.4)
    ]


def _msg(*lines: tuple[float, str]) -> list[dict]:
    return [{"TimeUS": int(t * 1e6), "Message": m} for t, m in lines]


def _titles(findings):
    return [(f.title, f.severity) for f in findings]


def test_clean_log_has_no_findings():
    pm = build_series(0.0, 1.0, InE=[0] * 10, NLon=[0] * 10, NL=[4000] * 10)
    assert HealthCheck().run(make_flight_log({"PM": pm, "MSG": _msg((1.0, "Frame: QUAD/X"))})) == []
    assert HealthCheck().run(make_flight_log({})) == []


def test_hard_fault_watchdog_is_critical():
    # The 86622 forum log (CUAV-X7+): WDOG FT=3 FL=122, boot texts restoring the attitude.
    log = make_flight_log(
        {
            "WDOG": _wdog(3),
            "MSG": _msg((0.2, "Restored watchdog attitude 1 6 8"), (0.2, "Forcing safety off for watchdog")),
            "PM": build_series(0.0, 10.0, InE=[2048] * 3),
        }
    )
    findings = HealthCheck().run(log)
    assert _titles(findings) == [("Flight controller was reset by its watchdog", Severity.CRITICAL)]
    f = findings[0]
    assert f.evidence.time_start_s == 0.2  # the boot texts, before the first WDOG record
    assert f.evidence.samples["fault_type"] == 3 and f.evidence.samples["fault_line"] == 122
    assert "Hard Fault" in f.explanation and "safety switch was off" in f.explanation


def test_hang_without_fault_on_the_bench_is_a_warning():
    # The 138782 forum log: FT=0, task 6, nothing about arming or the safety switch.
    findings = HealthCheck().run(make_flight_log({"WDOG": _wdog(0, fl=0, tsk=6)}))
    assert _titles(findings) == [("Flight controller was reset by its watchdog", Severity.WARN)]
    assert "no fault recorded" in findings[0].explanation
    assert "extremely serious" not in findings[0].explanation


def test_armed_at_reset_is_critical_even_without_a_fault():
    log = make_flight_log(
        {"WDOG": _wdog(0, fl=0, tsk=6), "MSG": _msg((0.1, "Forcing logging for watchdog reset"))}
    )
    findings = HealthCheck().run(log)
    assert findings[0].severity is Severity.CRITICAL
    assert findings[0].evidence.samples["armed_at_reset"] == 1


def test_telemetry_text_alone_reports_the_watchdog():
    text = "WDG: T54 SL0 FL127 FT3 FA0 FTP180 FLR806BBF9 FICSR4196355 MM0 MC0 IE0 IEC0 TN:Ardu"
    findings = HealthCheck().run(make_flight_log({"MSG": _msg((10.0, text))}))
    assert _titles(findings) == [("Flight controller was reset by its watchdog", Severity.CRITICAL)]
    assert findings[0].evidence.samples["fault_line"] == 127


def test_internal_errors_decoded_from_pm():
    pm = build_series(0.0, 1.0, InE=[0] * 5 + [0x4000 | 0x8000] * 5)
    findings = HealthCheck().run(make_flight_log({"PM": pm}))
    assert _titles(findings) == [("Autopilot raised internal errors", Severity.WARN)]
    assert "spi_fail, main_loop_stuck" in findings[0].explanation
    assert findings[0].evidence.time_start_s == 5.0


def test_internal_errors_from_the_prearm_text_and_prearm_defers():
    log = make_flight_log({"MSG": _msg((15.0, "PreArm: Internal errors 0x100000 l:917 flow_of_ctr"))})
    assert _titles(HealthCheck().run(log)) == [("Autopilot raised internal errors", Severity.WARN)]
    assert StartupMessagesCheck().run(log) == []


def test_watchdog_bit_is_not_reported_twice():
    log = make_flight_log({"WDOG": _wdog(3), "PM": build_series(0.0, 1.0, InE=[2048] * 3)})
    assert [f.title for f in HealthCheck().run(log)] == ["Flight controller was reset by its watchdog"]


def test_main_loop_stall_while_armed():
    mon = [{"TimeUS": 20_000_000, "Dly": 620, "Tsk": 4}, {"TimeUS": 40_000_000, "Dly": 250, "Tsk": 4}]
    ev = [{"TimeUS": 10_000_000, "Id": 10}, {"TimeUS": 30_000_000, "Id": 11}]
    findings = HealthCheck().run(make_flight_log({"MON": mon, "EV": ev}))
    assert _titles(findings) == [("Main loop stalled while armed", Severity.WARN)]
    assert findings[0].evidence.samples == {"max_delay_ms": 620, "records": 1}


def test_short_stall_is_info_and_disarmed_stall_is_ignored():
    ev = [{"TimeUS": 0, "Id": 10}]
    short = make_flight_log({"MON": [{"TimeUS": 5_000_000, "Dly": 240}], "EV": ev})
    assert _titles(HealthCheck().run(short)) == [("Main loop stalled while armed", Severity.INFO)]
    disarmed = make_flight_log(
        {"MON": [{"TimeUS": 5_000_000, "Dly": 900}], "EV": [{"TimeUS": 9_000_000, "Id": 10}]}
    )
    assert HealthCheck().run(disarmed) == []


def test_health_owns_and_bare_prearm_line():
    assert health_owns("PreArm: Internal errors (0x800)")
    assert health_owns("WDG: T6 SL0 FL0 FT0 FA0")
    assert not health_owns("PreArm: Compass not calibrated")
    log = make_flight_log({"MSG": _msg((1.0, "PreArm:"), (1.0, "PreArm: Compass not calibrated"))})
    assert [f.title for f in StartupMessagesCheck().run(log)] == [
        "Startup warning: PreArm: Compass not calibrated"
    ]
