"""Tests for parameter-file ingestion and the param completeness signals."""

from __future__ import annotations

from ardupilot_mcp.param_file import parse_param_file
from ardupilot_mcp.server import get_params, load_param_file
from tests.synth_bin import build_bin


def test_parse_mission_planner_format(tmp_path):
    p = tmp_path / "mp.param"
    p.write_text("# Mission Planner params\nWPNAV_SPEED,500\nATC_RAT_RLL_P, 0.135\nFRAME_CLASS 1\n")
    params = parse_param_file(str(p))
    assert params["WPNAV_SPEED"] == 500.0
    assert abs(params["ATC_RAT_RLL_P"] - 0.135) < 1e-6
    assert params["FRAME_CLASS"] == 1.0


def test_parse_qgc_format(tmp_path):
    p = tmp_path / "qgc.params"
    p.write_text(
        "# Onboard parameters\n#\n# Vehicle-Id Component-Id Name Value Type\n"
        "1\t1\tWPNAV_SPEED\t500.000000\t9\n"
        "1\t1\tINS_HNTCH_FREQ\t80.000000\t9\n"
    )
    params = parse_param_file(str(p))
    assert params["WPNAV_SPEED"] == 500.0
    assert params["INS_HNTCH_FREQ"] == 80.0


def test_parse_skips_comments_and_junk(tmp_path):
    p = tmp_path / "x.param"
    p.write_text("# comment\n// also comment\n\nGOOD,1.5\nBAD_LINE_NO_VALUE\nALSO,notanumber\n")
    params = parse_param_file(str(p))
    assert params == {"GOOD": 1.5}


def test_load_param_file_tool(tmp_path):
    p = tmp_path / "full.param"
    p.write_text("WPNAV_SPEED,500\nINS_HNTCH_ENABLE,1\nINS_HNTCH_FREQ,80\n")
    res = load_param_file(str(p))
    assert res.source == "file"
    assert res.total == 3
    res2 = load_param_file(str(p), name_glob="INS_HNTCH_*")
    assert set(res2.params) == {"INS_HNTCH_ENABLE", "INS_HNTCH_FREQ"}
    assert res2.matched == 2


def _synth(tmp_path, params, firmware="ArduCopter V4.5.7 (x)"):
    p = str(tmp_path / "s.bin")
    msgs = [
        (
            "ATT",
            {"TimeUS": i * 100000, "DesRoll": 0, "Roll": 0, "DesPitch": 0, "Pitch": 0, "DesYaw": 0, "Yaw": 0},
        )
        for i in range(20)
    ]
    build_bin(p, msgs, params=params, firmware=firmware)
    return p


def test_get_params_reports_total_and_metadata_url(tmp_path):
    p = _synth(tmp_path, {"INS_HNTCH_ENABLE": 1.0, "FRAME_CLASS": 1.0, "WPNAV_SPEED": 500.0})
    res = get_params(p)
    assert res.source == "log"
    assert res.total == 3
    # 4.5.7 firmware -> a version-specific metadata URL is offered
    assert res.metadata_url and "stable-4.5.7" in res.metadata_url


def test_get_params_notes_incomplete_snapshot(tmp_path):
    # Only a few params logged -> note suggests the dump may be incomplete.
    p = _synth(tmp_path, {"WPNAV_SPEED": 500.0})
    res = get_params(p)
    assert res.note and "incomplete" in res.note.lower()
