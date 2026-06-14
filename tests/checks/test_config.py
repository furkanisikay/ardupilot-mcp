"""Tests for the configuration safety check."""

from __future__ import annotations

from ardupilot_mcp.checks.config import ConfigSafetyCheck
from ardupilot_mcp.model import CheckStatus, Severity
from tests.helpers import make_flight_log


def _run(params, vehicle_kind="copter"):
    log = make_flight_log({}, params=params, vehicle_kind=vehicle_kind)
    return ConfigSafetyCheck().run(log)


def test_empty_params_no_findings():
    assert _run({}) == []


def test_check_always_runs_even_with_no_messages():
    # requires is empty, so the check runs (status RAN) regardless of messages.
    log = make_flight_log({}, params={}, vehicle_kind="copter")
    result = ConfigSafetyCheck().execute(log)
    assert result.status == CheckStatus.RAN
    assert result.findings == []


# -- ARMING_CHECK ------------------------------------------------------------


def test_arming_check_zero_warns():
    findings = _run({"ARMING_CHECK": 0})
    arm = [f for f in findings if "Arming safety checks disabled" in f.title]
    assert len(arm) == 1
    assert arm[0].severity == Severity.WARN
    assert arm[0].evidence.samples["ARMING_CHECK"] == 0


def test_arming_check_one_no_finding():
    findings = _run({"ARMING_CHECK": 1})
    assert not any("Arming" in f.title for f in findings)


def test_arming_check_negative_bitmask_no_finding():
    # -9 is a bitmask (checks enabled), must NOT be flagged.
    findings = _run({"ARMING_CHECK": -9})
    assert findings == []


def test_arming_check_large_bitmask_no_finding():
    findings = _run({"ARMING_CHECK": 6138})
    assert findings == []


# -- BATT_MONITOR ------------------------------------------------------------


def test_batt_monitor_zero_info():
    findings = _run({"BATT_MONITOR": 0})
    batt = [f for f in findings if "battery monitor" in f.title.lower()]
    assert len(batt) == 1
    assert batt[0].severity == Severity.INFO
    assert batt[0].evidence.samples["BATT_MONITOR"] == 0


def test_batt_monitor_configured_no_finding():
    findings = _run({"BATT_MONITOR": 4})
    assert findings == []


# -- FS_THR_ENABLE -----------------------------------------------------------


def test_fs_thr_disabled_info_on_copter():
    findings = _run({"FS_THR_ENABLE": 0}, vehicle_kind="copter")
    fs = [f for f in findings if "failsafe disabled" in f.title.lower()]
    assert len(fs) == 1
    assert fs[0].severity == Severity.INFO
    assert fs[0].evidence.samples["FS_THR_ENABLE"] == 0


def test_fs_thr_disabled_skipped_on_rover():
    # FS_THR_ENABLE is copter/plane/heli only; rover must not produce it.
    findings = _run({"FS_THR_ENABLE": 0}, vehicle_kind="rover")
    assert findings == []


def test_fs_thr_disabled_runs_on_unknown_kind():
    # Unknown/None vehicle kind still evaluates (avoid false negatives).
    findings = _run({"FS_THR_ENABLE": 0}, vehicle_kind=None)
    assert any("failsafe disabled" in f.title.lower() for f in findings)


def test_fs_thr_enabled_no_finding():
    findings = _run({"FS_THR_ENABLE": 1}, vehicle_kind="copter")
    assert findings == []


# -- combined / registry -----------------------------------------------------


def test_multiple_risky_params_all_flagged():
    findings = _run({"ARMING_CHECK": 0, "BATT_MONITOR": 0, "FS_THR_ENABLE": 0})
    titles = {f.title for f in findings}
    assert "Arming safety checks disabled" in titles
    assert "No battery monitor configured" in titles
    assert "RC/throttle failsafe disabled" in titles
    sev = {f.title: f.severity for f in findings}
    assert sev["Arming safety checks disabled"] == Severity.WARN
    assert sev["No battery monitor configured"] == Severity.INFO
    assert sev["RC/throttle failsafe disabled"] == Severity.INFO


def test_clean_config_no_findings():
    findings = _run({"ARMING_CHECK": 1, "BATT_MONITOR": 4, "FS_THR_ENABLE": 1})
    assert findings == []


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    assert get_check("config") is not None
