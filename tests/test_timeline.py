"""Tests for the incident timeline (onset / impact ordering)."""

from __future__ import annotations

from ardupilot_mcp.orchestrator import diagnose
from ardupilot_mcp.timeline import build_timeline
from tests.helpers import build_series, make_flight_log

ARMED = [{"TimeUS": 5_000_000, "Id": 10}]


def _crash_log():
    n = 900  # 90 s at 10 Hz
    vibe_z = [10.0] * 400 + [75.0] * 500  # vibration from t=40 s
    roll = [0.0] * 610 + [80.0] * 290  # tumbles after the crash at t=60 s
    return make_flight_log(
        {
            "EV": ARMED,
            "VIBE": build_series(0.0, 10.0, VibeX=[5.0] * n, VibeY=[5.0] * n, VibeZ=vibe_z),
            "ATT": build_series(
                0.0,
                10.0,
                DesRoll=[0.0] * n,
                Roll=roll,
                DesPitch=[0.0] * n,
                Pitch=[0.0] * n,
                DesYaw=[0.0] * n,
                Yaw=[0.0] * n,
            ),
            "ERR": [{"TimeUS": 60_000_000, "Subsys": 12, "ECode": 1}],
        },
        vehicle_kind="copter",
    )


def test_onset_precedes_impact_and_consequences_are_flagged():
    rep = diagnose(_crash_log())
    inc = rep.incident
    assert inc is not None
    assert inc.impact_s == 60.0 and "CRASH_CHECK" in (inc.impact_basis or "")
    assert inc.onset_s == 40.0 and inc.first_anomaly_check_id == "vibration"
    att = [f for f in rep.findings if f.check_id == "attitude"]
    assert att and all(f.after_impact for f in att)
    vib = [f for f in rep.findings if f.check_id == "vibration"]
    assert vib and not any(f.after_impact for f in vib)
    # Consequences are listed after everything else.
    assert rep.findings[-1].after_impact
    assert "First anomaly while armed" in rep.summary_text


def test_timeline_phases_are_ordered():
    log = _crash_log()
    tl = build_timeline(log, diagnose(log).findings)
    phases = [e.phase for e in tl.events if e.time_s is not None]
    order = ["before_onset", "onset", "after_onset", "impact", "after_impact"]
    assert phases == sorted(phases, key=order.index)
    assert any(e.phase == "impact" for e in tl.events)


def test_clean_log_has_no_incident():
    n = 100
    log = make_flight_log(
        {
            "EV": ARMED + [{"TimeUS": 9_000_000, "Id": 11}],
            "VIBE": build_series(0.0, 10.0, VibeX=[5.0] * n, VibeY=[5.0] * n, VibeZ=[9.0] * n),
        }
    )
    assert diagnose(log).incident is None


def test_log_ending_armed_is_the_impact():
    n = 300
    log = make_flight_log({"EV": ARMED, "ATT": build_series(0.0, 10.0, Roll=[0.0] * n)})
    tl = build_timeline(log, diagnose(log).findings)
    assert tl.log_ended_armed
    assert tl.impact_basis == "log ended while armed"
    assert tl.impact_s is not None and abs(tl.impact_s - 29.9) < 1e-6


def test_flight_long_conditions_are_not_the_onset():
    from ardupilot_mcp.checks.util import make_finding
    from ardupilot_mcp.model import Finding, Severity

    log = _crash_log()
    findings = diagnose(log).findings + [
        make_finding("gps", Severity.WARN, "High GPS HDOP, peak 3.0", "x", time_start_s=5.0, time_end_s=89.0),
        make_finding("compass", Severity.WARN, "Unstable compass field strength", "x", time_start_s=0.0),
    ]
    assert all(isinstance(f, Finding) for f in findings)
    tl = build_timeline(log, findings)
    assert tl.onset_s == 40.0  # the vibration, not the flight-long HDOP or compass statistic
    assert set(tl.conditions) >= {"High GPS HDOP, peak 3.0", "Unstable compass field strength"}
