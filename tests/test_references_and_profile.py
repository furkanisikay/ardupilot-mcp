"""Tests for documentation references, version-aware doc links, and vehicle profile."""

from __future__ import annotations

from ardupilot_mcp import ardupilot_docs as docs
from ardupilot_mcp.ardupilot_meta import (
    estimate_cells,
    frame_class_name,
    motor_count_for_frame,
)
from ardupilot_mcp.orchestrator import diagnose
from ardupilot_mcp.profile import build_profile
from tests.helpers import build_series, make_flight_log

# ---- references on findings/report ------------------------------------------


def _vibe_log():
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
    return make_flight_log({"VIBE": recs}, vehicle_kind="copter")


def test_findings_carry_doc_references():
    rep = diagnose(_vibe_log())
    vib = next(f for f in rep.findings if f.check_id == "vibration")
    assert vib.references, "vibration findings should carry doc references"
    assert any("vibration" in r.url for r in vib.references)


def test_report_aggregates_references_and_guidance():
    rep = diagnose(_vibe_log())
    assert rep.references  # de-duplicated across findings + baseline
    urls = [r.url for r in rep.references]
    assert len(urls) == len(set(urls)), "report references must be de-duplicated"
    assert any("parameters" in u for u in urls)  # baseline param reference
    assert "firmware version" in rep.guidance.lower()


def test_vehicle_aware_reference_swaps_common_page():
    refs = docs.references_for("vibration", "plane")
    assert any("/plane/" in r.url for r in refs)  # common-* page served under plane path


# ---- version-specific parameter docs ----------------------------------------


def test_versioned_param_doc_for_modern_firmware():
    url = docs.versioned_param_doc_url("ArduCopter V4.5.7 (abc1234)", "copter")
    assert url == "https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml"


def test_no_versioned_param_doc_for_old_firmware():
    assert docs.versioned_param_doc_url("APM:Copter V3.6.4 (abc)", "copter") is None
    assert docs.versioned_param_doc_url(None, "copter") is None


def test_version_reference_in_report_for_4x_only():
    log4 = make_flight_log(
        {"VIBE": _vibe_log().get("VIBE")}, vehicle_kind="copter", firmware_version="ArduCopter V4.6.0 (x)"
    )
    rep4 = diagnose(log4)
    assert any("versioned" in r.url for r in rep4.references)


# ---- cell / frame helpers ---------------------------------------------------


def test_estimate_cells_high_cell_counts():
    # 47.6 V fits 11S (4.33 V/cell) and 12S (3.97 V/cell); a real 12S forum log read 11S.
    assert estimate_cells(47.6) == 12
    assert estimate_cells(23.05) == 6  # 3.84 V/cell; rounding V/4.2 gave 5S at 4.6 V/cell -> None
    # A pack resting at 3.8 V/cell is ambiguous by voltage alone; MOT_BAT_VOLT_MAX (4.2 V x cells,
    # ArduPilot "Setting up for tuning") decides.
    assert estimate_cells(45.6, {"MOT_BAT_VOLT_MAX": 50.4}) == 12
    assert estimate_cells(45.6, {"MOT_BAT_VOLT_MIN": 39.6}) == 12
    # A hint the measured voltage contradicts (set for 6S, flying 12S) is ignored.
    assert estimate_cells(47.3, {"MOT_BAT_VOLT_MAX": 25.2}) == 12


def test_estimate_cells():
    assert estimate_cells(16.8) == 4  # 4S
    assert estimate_cells(25.2) == 6  # 6S
    assert estimate_cells(12.6) == 3  # 3S
    assert estimate_cells(2447.0) is None  # implausible scale
    assert estimate_cells(None) is None


def test_frame_helpers():
    assert frame_class_name(1) == "Quad"
    assert motor_count_for_frame(2) == 6  # hexa
    assert motor_count_for_frame(6) is None  # heli -> no simple motor count


# ---- vehicle profile --------------------------------------------------------


def test_vehicle_profile_extracts_physical_config():
    bat = build_series(0.0, 10.0, Volt=[16.6] * 30 + [15.0] * 20, Curr=[20.0] * 50)
    log = make_flight_log(
        {"BAT": bat},
        params={"FRAME_CLASS": 1, "FRAME_TYPE": 1, "MOT_THST_HOVER": 0.5, "BATT_CAPACITY": 5000},
        vehicle_kind="copter",
    )
    p = build_profile(log)
    assert p.frame_class_name == "Quad"
    assert p.frame_type_name == "X"
    assert p.motor_count == 4
    assert p.battery_cells == 4
    assert p.battery_capacity_mah == 5000
    assert abs(p.hover_throttle - 0.5) < 1e-6
    assert abs(p.power_margin_pct - 50.0) < 1e-6
    assert abs(p.thrust_to_weight_estimate - 2.0) < 1e-6
    assert any("Quad" in a for a in p.assessment)


def test_profile_flags_underpowered():
    bat = build_series(0.0, 10.0, Volt=[25.2] * 50, Curr=[40.0] * 50)
    log = make_flight_log(
        {"BAT": bat},
        params={"FRAME_CLASS": 2, "MOT_THST_HOVER": 0.7},  # hexa hovering at 70%
        vehicle_kind="copter",
    )
    p = build_profile(log)
    assert p.battery_cells == 6
    assert any("underpowered" in a.lower() for a in p.assessment)


def test_plane_cruise_throttle_is_not_a_hover_throttle():
    from ardupilot_mcp.profile import _hover_throttle

    ctun = build_series(0.0, 10.0, ThO=[0.55] * 100)
    plane = make_flight_log({"CTUN": ctun}, vehicle_type="ArduPlane", vehicle_kind="plane")
    copter = make_flight_log({"CTUN": ctun}, vehicle_kind="copter")
    assert _hover_throttle(plane) == (None, False)
    assert _hover_throttle(copter) == (0.55, True)
