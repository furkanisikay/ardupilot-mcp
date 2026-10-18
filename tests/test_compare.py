"""Tests for compare_logs (what changed between two flights)."""

from __future__ import annotations

from ardupilot_mcp.compare import compare_logs
from tests.helpers import build_series, make_flight_log


def _log(params, vibe_z=10.0, fw="ArduCopter V4.6.3"):
    n = 50
    vibe = build_series(0.0, 10.0, VibeX=[5.0] * n, VibeY=[5.0] * n, VibeZ=[vibe_z] * n)
    return make_flight_log({"VIBE": vibe}, params=params, firmware_version=fw)


def test_configuration_changes_are_separated_from_learned_values():
    a = _log({"ATC_RAT_RLL_P": 0.135, "STAT_BOOTCNT": 10, "INS_GYROFFS_X": 0.01, "MOT_THST_HOVER": 0.35})
    b = _log({"ATC_RAT_RLL_P": 0.2, "STAT_BOOTCNT": 11, "INS_GYROFFS_X": 0.02, "MOT_THST_HOVER": 0.36})
    c = compare_logs(a, b)
    assert [(x.name, x.a, x.b) for x in c.changed] == [("ATC_RAT_RLL_P", 0.135, 0.2)]
    assert {x.name for x in c.learned_changed} == {"STAT_BOOTCNT", "INS_GYROFFS_X", "MOT_THST_HOVER"}


def test_added_removed_firmware_and_metrics():
    a = _log({"X": 1.0, "OLD": 2.0}, vibe_z=10.0, fw="ArduCopter V4.5.7")
    b = _log({"X": 1.0, "NEW": 3.0}, vibe_z=45.0)
    c = compare_logs(a, b)
    assert c.added == ["NEW"] and c.removed == ["OLD"]
    assert c.firmware_changed
    assert c.metrics["max_VibeZ"].a == 10.0 and c.metrics["max_VibeZ"].b == 45.0
    assert not c.changed


def test_float32_noise_is_not_a_change():
    c = compare_logs(_log({"P": 0.135}), _log({"P": 0.13500000536441803}))
    assert c.changed == []
