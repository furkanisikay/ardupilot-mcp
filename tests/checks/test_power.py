"""Tests for the battery & power check."""

from __future__ import annotations

from ardupilot_mcp.checks.power import PowerCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import build_series, make_flight_log


def _log(bat_records, *, params=None):
    return make_flight_log({"BAT": bat_records}, params=params)


def test_skipped_when_no_bat():
    log = make_flight_log({})
    result = PowerCheck().execute(log)
    assert result.status == CheckStatus.SKIPPED
    assert "BAT" in (result.skipped_reason or "")


def test_clean_flight_no_findings():
    # Flat, healthy voltage well above any threshold; modest steady current.
    recs = build_series(
        0.0,
        10.0,
        Volt=[16.0] * 50,
        Curr=[8.0] * 50,
    )
    params = {"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.2}
    findings = PowerCheck().run(_log(recs, params=params))
    assert findings == []


def test_critical_below_crt_volt():
    # Voltage sags below the configured critical threshold -> CRITICAL.
    volt = [16.0] * 30 + [13.0] * 20  # ends at 13.0, below BATT_CRT_VOLT=13.2
    recs = build_series(0.0, 10.0, Volt=volt, Curr=[10.0] * 50)
    params = {"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.2}
    findings = PowerCheck().run(_log(recs, params=params))
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "critical" in f.title.lower()]
    assert crit, "expected a critical voltage finding"
    assert crit[0].evidence.samples["min_volt"] <= crit[0].evidence.samples["crit_volt"]
    assert crit[0].evidence.samples["min_volt"] == 13.0


def test_warn_below_low_volt_only():
    # Below low but above critical -> WARN, not CRITICAL.
    volt = [16.0] * 30 + [13.8] * 20  # min 13.8: <= LOW (14.0), > CRT (13.2)
    recs = build_series(0.0, 10.0, Volt=volt, Curr=[10.0] * 50)
    params = {"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.2}
    findings = PowerCheck().run(_log(recs, params=params))
    low = [f for f in findings if "low-voltage" in f.title.lower()]
    assert low, "expected a low-voltage warning"
    assert low[0].severity == Severity.WARN
    assert low[0].evidence.samples["min_volt"] == 13.8
    assert not any(f.severity == Severity.CRITICAL for f in findings)


def test_derives_thresholds_from_cells_without_params():
    # No BATT_*_VOLT params: thresholds are derived from the estimated cell count.
    # A 4S pack (peak ~16.6 V) dropping to 12.0 V is below 4*3.3 = 13.2 V critical.
    volt = [16.6] * 30 + [12.0] * 20
    recs = build_series(0.0, 10.0, Volt=volt, Curr=[5.0] * 50)
    findings = PowerCheck().run(_log(recs))  # no params
    crit = [f for f in findings if f.severity == Severity.CRITICAL and "critical voltage" in f.title.lower()]
    assert crit, "expected a cell-derived critical-voltage finding"
    assert "derived" in crit[0].explanation.lower()


def test_no_finding_when_voltage_healthy_without_params():
    # Healthy 4S voltage, no params -> derived thresholds not breached -> no finding.
    recs = build_series(0.0, 10.0, Volt=[15.5] * 50, Curr=[5.0] * 50)
    findings = PowerCheck().run(_log(recs))
    assert not any(f.severity == Severity.CRITICAL for f in findings)


def test_sudden_drop_warn():
    # A fast -3V step that STAYS down (real brown-out, not a 1-sample glitch) -> WARN.
    volt = [16.0] * 25 + [13.0] * 25  # fast 3 V drop, sustained
    recs = build_series(0.0, 10.0, Volt=volt, Curr=[8.0] * 50)
    # High thresholds so the dip does not also trip the min-voltage rule.
    params = {"BATT_LOW_VOLT": 10.0, "BATT_CRT_VOLT": 9.0}
    findings = PowerCheck().run(_log(recs, params=params))
    drop = [f for f in findings if "sudden" in f.title.lower()]
    assert drop, "expected a sudden voltage drop finding"
    assert drop[0].severity == Severity.WARN
    assert drop[0].evidence.samples["drop_volt"] >= 2.0
    assert drop[0].evidence.samples["drop_volt"] == 3.0


def test_sudden_drop_critical():
    # A -5V fast collapse that stays down -> CRITICAL sudden-drop finding.
    volt = [16.0] * 25 + [11.0] * 25  # 5 V fast collapse, sustained
    recs = build_series(0.0, 10.0, Volt=volt, Curr=[8.0] * 50)
    params = {"BATT_LOW_VOLT": 8.0, "BATT_CRT_VOLT": 7.0}
    findings = PowerCheck().run(_log(recs, params=params))
    drop = [f for f in findings if "sudden" in f.title.lower()]
    assert drop, "expected a sudden voltage drop finding"
    assert drop[0].severity == Severity.CRITICAL
    assert drop[0].evidence.samples["drop_volt"] == 5.0


def test_slow_decline_is_not_a_sudden_drop():
    # Gradual discharge over many samples: no single fast step -> no drop finding.
    volt = [16.0 - 0.1 * i for i in range(50)]  # 16.0 down to ~11.1, 0.1V/step
    recs = build_series(0.0, 10.0, Volt=volt, Curr=[8.0] * 50)
    params = {"BATT_LOW_VOLT": 9.0, "BATT_CRT_VOLT": 8.0}
    findings = PowerCheck().run(_log(recs, params=params))
    assert not any("sudden" in f.title.lower() for f in findings)


def test_sag_under_load_info():
    # Big voltage spread while current is high -> INFO sag finding.
    volt = [16.0] * 25 + [13.5] * 25  # 2.5 V spread under load
    curr = [30.0] * 50  # all high current
    recs = build_series(0.0, 10.0, Volt=volt, Curr=curr)
    # Keep thresholds low so only the sag INFO fires (no min/drop findings).
    params = {"BATT_LOW_VOLT": 10.0, "BATT_CRT_VOLT": 9.0}
    findings = PowerCheck().run(_log(recs, params=params))
    sag = [f for f in findings if "sag" in f.title.lower()]
    assert sag, "expected a sag-under-load finding"
    assert sag[0].severity == Severity.INFO
    assert sag[0].evidence.samples["sag_volt"] == 2.5
    assert sag[0].evidence.samples["peak_current_a"] == 30.0
    # The 2.5 V step here happens over 0.1 s but DROP threshold is 2.0 V, so a
    # drop finding will ALSO fire; that's fine — assert the sag one exists.


def test_no_sag_when_current_low():
    # Same voltage spread but low current -> no sag finding.
    volt = [16.0] * 25 + [13.5] * 25
    curr = [5.0] * 50  # below HIGH_CURRENT_A
    recs = build_series(0.0, 10.0, Volt=volt, Curr=curr)
    params = {"BATT_LOW_VOLT": 10.0, "BATT_CRT_VOLT": 9.0}
    findings = PowerCheck().run(_log(recs, params=params))
    assert not any("sag" in f.title.lower() for f in findings)


def test_single_sample_glitch_ignored():
    # A 1-sample -4V dip that immediately recovers is sensor noise, not a brown-out.
    volt = [16.0] * 25 + [12.0] + [16.0] * 24
    recs = build_series(0.0, 10.0, Volt=volt, Curr=[8.0] * 50)
    params = {"BATT_LOW_VOLT": 8.0, "BATT_CRT_VOLT": 7.0}
    findings = PowerCheck().run(_log(recs, params=params))
    assert not any("sudden" in f.title.lower() for f in findings)


def test_implausible_voltage_scale_skipped():
    # Some logs have no real battery monitor / an odd scale (e.g. raw ~2400).
    recs = build_series(0.0, 10.0, Volt=[2447.0] * 30 + [2407.0] * 20, Curr=[0.0] * 50)
    assert PowerCheck().run(_log(recs)) == []


def test_works_on_older_curr_message():
    # Copter 3.x logs the battery as CURR (same Volt/Curr fields), not BAT.
    volt = [16.0] * 30 + [13.0] * 20
    recs = build_series(0.0, 10.0, Volt=volt, Curr=[10.0] * 50)
    params = {"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.2}
    log = make_flight_log({"CURR": recs}, params=params)
    # It must run (not skip) and find the critical voltage from CURR.
    result = PowerCheck().execute(log)
    assert result.status == CheckStatus.RAN
    crit = [f for f in result.findings if f.severity == Severity.CRITICAL]
    assert crit and crit[0].evidence.message_types == ["CURR"]


def test_skipped_when_no_power_message():
    result = PowerCheck().execute(make_flight_log({"ATT": build_series(0.0, 10.0, Roll=[0.0] * 10)}))
    assert result.status == CheckStatus.SKIPPED


def test_board_voltage_only_still_runs():
    # Only board-voltage POWR present (Vcc), no BAT/CURR -> the Vcc check runs; stable 5 V is clean.
    log = make_flight_log({"POWR": build_series(0.0, 10.0, Vcc=[5.0] * 10)})
    result = PowerCheck().execute(log)
    assert result.status == CheckStatus.RAN
    assert result.findings == []


def _armed(extra: dict) -> dict:
    return {"EV": [{"TimeUS": 1_000_000, "Id": 10}, {"TimeUS": 9_000_000, "Id": 11}], **extra}


def test_vcc_brownout_while_armed_is_critical():
    vcc = [5.05] * 40 + [4.3] * 5 + [5.05] * 55
    log = make_flight_log(_armed({"POWR": build_series(0.0, 10.0, Vcc=vcc)}))
    f = [x for x in PowerCheck().run(log) if "Vcc" in x.title]
    assert f and f[0].severity == Severity.CRITICAL
    assert f[0].evidence.samples["min_vcc"] == 4.3


def test_vcc_brownout_before_arming_is_ignored():
    vcc = [4.3] * 5 + [5.05] * 95  # USB/bench sag before arming at t=1 s
    log = make_flight_log(_armed({"POWR": build_series(0.0, 10.0, Vcc=vcc)}))
    assert [x for x in PowerCheck().run(log) if "Vcc" in x.title] == []


def test_vcc_ripple_is_warn():
    vcc = [5.0, 5.35] * 50
    log = make_flight_log(_armed({"POWR": build_series(0.0, 10.0, Vcc=vcc)}))
    f = [x for x in PowerCheck().run(log) if "Vcc" in x.title]
    assert f and f[0].severity == Severity.WARN


def test_second_battery_uses_its_own_thresholds():
    from tests.helpers import interleave_instances

    n = 60
    main = build_series(0.0, 10.0, Volt=[16.4 - 0.01 * i for i in range(n)], Curr=[10.0] * n)
    payload = build_series(0.0, 10.0, Volt=[7.4] * n, Curr=[0.5] * n)
    bat = interleave_instances("Inst", {0: main, 1: payload})
    log = make_flight_log({"BAT": bat}, params={"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.2})
    assert PowerCheck().run(log) == []  # 7.4 V is healthy for the 2S payload pack
    log = make_flight_log({"BAT": bat}, params={"BATT2_LOW_VOLT": 7.6})
    f = PowerCheck().run(log)
    assert [x.title for x in f] == ["Battery reached low-voltage threshold (battery 2)"]


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("power") is not None


def test_low_voltage_window_starts_at_the_first_crossing():
    # The failsafe timer counts from the first sample at the threshold, not the minimum.
    from ardupilot_mcp.checks.power import PowerCheck

    volts = [12.0] * 10 + [10.9] * 10 + [10.5] * 10
    log = make_flight_log(
        {"BAT": build_series(0.0, 1.0, Volt=volts, Curr=[5.0] * 30)}, params={"BATT_LOW_VOLT": 11.0}
    )
    [f] = [x for x in PowerCheck().run(log) if x.title == "Battery reached low-voltage threshold"]
    assert f.evidence.time_start_s == 10.0 and f.evidence.time_end_s == 20.0


def test_a_momentary_sag_does_not_start_the_low_voltage_window():
    # A dip under load at takeoff, recovery, then a sustained fall (BATT_LOW_TIMER 10 s).
    volts = [12.0] * 5 + [10.8] + [12.0] * 10 + [10.9] * 15
    log = make_flight_log(
        {"BAT": build_series(0.0, 1.0, Volt=volts, Curr=[5.0] * len(volts))}, params={"BATT_LOW_VOLT": 11.0}
    )
    [f] = [x for x in PowerCheck().run(log) if x.title == "Battery reached low-voltage threshold"]
    assert f.evidence.time_start_s == 16.0


def test_a_persistent_drop_is_found_behind_a_larger_glitch():
    volts = [16.0] * 400
    volts[100] = 11.0  # a one-sample glitch that recovers
    volts[250:] = [13.0] * 150  # a 3 V drop that stays
    findings = PowerCheck().run(_log(build_series(0.0, 10.0, Volt=volts, Curr=[10.0] * 400)))
    drop = [f for f in findings if f.title == "Sudden voltage drop"]
    assert len(drop) == 1
    assert drop[0].evidence.samples["drop_volt"] == 3.0
    assert drop[0].evidence.time_start_s == 24.9


def test_zero_volts_before_arming_is_not_a_critical_battery():
    volts = [0.0] * 50 + [16.4] * 500
    log = make_flight_log(
        {
            "BAT": build_series(0.0, 10.0, Volt=volts, Curr=[5.0] * 550),
            "EV": [{"TimeUS": 5_000_000, "Id": 10}, {"TimeUS": 54_000_000, "Id": 11}],
        },
        params={"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.0},
    )
    assert PowerCheck().run(log) == []


def test_a_sag_shorter_than_the_failsafe_timer_is_not_a_threshold_finding():
    volts = [16.0] * 300
    volts[100:120] = [12.5] * 20  # 2 s below the 13 V critical level, then recovers
    params = {"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.0, "BATT_LOW_TIMER": 10.0}
    findings = PowerCheck().run(_log(build_series(0.0, 10.0, Volt=volts, Curr=[40.0] * 300), params=params))
    assert not [
        f
        for f in findings
        if "voltage" in f.title and "threshold" in f.title or "critical voltage" in f.title
    ]


def test_a_pack_that_collapses_at_the_end_is_critical_before_the_timer_expires():
    volts = [16.0] * 300 + [12.0] * 30  # 3 s below critical until the log ends
    params = {"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.0, "BATT_LOW_TIMER": 10.0}
    findings = PowerCheck().run(_log(build_series(0.0, 10.0, Volt=volts, Curr=[40.0] * 330), params=params))
    crit = [f for f in findings if f.title == "Battery reached critical voltage"]
    assert len(crit) == 1 and crit[0].evidence.time_start_s == 30.0


def test_a_low_run_does_not_span_a_disarmed_gap():
    # Each flight ends 3 s below BATT_LOW_VOLT; the 3 minutes between them are disarmed.
    volts = [16.0] * 270 + [13.8] * 30 + [13.8] * 1800 + [13.8] * 30 + [16.0] * 270
    log = make_flight_log(
        {
            "BAT": build_series(0.0, 10.0, Volt=volts, Curr=[20.0] * len(volts)),
            "EV": [
                {"TimeUS": 0, "Id": 10},
                {"TimeUS": 30_000_000, "Id": 11},
                {"TimeUS": 210_000_000, "Id": 10},
                {"TimeUS": 240_000_000, "Id": 11},
            ],
        },
        params={"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 12.0, "BATT_LOW_TIMER": 10.0},
    )
    findings = PowerCheck().run(log)
    # The first flight's run reaches the end of its armed period and counts; the
    # second flight's ends above the threshold and must not join the first one.
    low = [f for f in findings if f.title == "Battery reached low-voltage threshold"]
    assert len(low) == 1 and low[0].evidence.time_start_s == 27.0


def test_the_last_sample_of_a_crash_spike_is_not_a_critical_battery():
    volts = [16.0] * 300 + [12.0]  # one sample below critical as the log ends
    params = {"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.0, "BATT_LOW_TIMER": 10.0}
    findings = PowerCheck().run(_log(build_series(0.0, 10.0, Volt=volts, Curr=[40.0] * 301), params=params))
    assert not [f for f in findings if f.title == "Battery reached critical voltage"]


def test_a_zero_low_timer_still_requires_the_default_hold():
    volts = [16.0] * 300
    volts[100:120] = [12.5] * 20  # 2 s below critical
    params = {"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.0, "BATT_LOW_TIMER": 0.0}
    findings = PowerCheck().run(_log(build_series(0.0, 10.0, Volt=volts, Curr=[40.0] * 300), params=params))
    assert not [f for f in findings if f.title == "Battery reached critical voltage"]
