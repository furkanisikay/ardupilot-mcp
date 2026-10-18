"""Bundled parameter metadata (ardupilot_mcp.param_meta) and its use by the param tools."""

from __future__ import annotations

import gzip
import json
from importlib import resources

import pytest

from ardupilot_mcp.param_meta import bundled, catalog_for, choose
from ardupilot_mcp.server import get_params, load_param_file
from scripts.gen_param_meta import extract
from tests.synth_bin import build_bin

VERSIONS = [(4, 4, 4), (4, 5, 7), (4, 6, 3)]


@pytest.mark.parametrize(
    ("want", "expect"),
    [
        ((4, 5, 7), ((4, 5, 7), "exact")),
        ((4, 5, 2), ((4, 5, 7), "same release series")),
        ((4, 8, 0), ((4, 6, 3), "nearest older release")),
        ((4, 2, 3), ((4, 4, 4), "nearest newer release")),
        ((3, 6, 12), None),  # another major version: parameters changed too much
    ],
)
def test_choose_the_release_that_describes_the_firmware(want, expect):
    assert choose(VERSIONS, want) == expect


def test_every_vehicle_kind_with_an_archive_has_bundled_metadata():
    have = bundled()
    for vehicle in ("Copter", "Plane", "Rover", "Sub"):
        assert have.get(vehicle), vehicle
    # A traditional helicopter runs Copter firmware; its H_ parameters are in Copter's set.
    heli = catalog_for("ArduCopter V4.6.3", "heli")
    assert heli is not None and heli.doc("H_RSC_MODE") is not None
    assert catalog_for("AntennaTracker V4.6.3", "tracker") is None
    assert catalog_for(None, "copter") is None


def test_documented_facts_for_a_known_parameter():
    cat = catalog_for("ArduCopter V4.6.3 (abc123)", "copter")
    assert cat is not None and cat.label == "Copter 4.6.3 (exact)"
    timer = cat.doc("BATT_LOW_TIMER")
    assert timer is not None and timer.range == [0, 120] and timer.units == "s"
    mode = cat.doc("INS_HNTCH_MODE")
    assert mode is not None and mode.values == [0, 1, 2, 3, 4, 5]
    assert cat.doc("LOG_BITMASK").bits[:3] == [0, 1, 2]
    assert cat.doc("NOT_A_PARAM") is None


def test_bundled_files_hold_facts_only():
    # Licence boundary: only names, numbers, units and flags, never ArduPilot's text.
    allowed = {"r", "u", "v", "b", "ro", "rb"}
    folder = resources.files("ardupilot_mcp").joinpath("data", "params")
    for entry in folder.iterdir():
        if not entry.name.endswith(".json.gz"):
            continue
        doc = json.loads(gzip.decompress(entry.read_bytes()))
        assert set(doc) == {"vehicle", "version", "source", "params"}
        for name, d in doc["params"].items():
            assert set(d) <= allowed, (entry.name, name)
            assert all(isinstance(x, (int, float)) for x in d.get("v", []) + d.get("r", []))


def test_generator_keeps_facts_and_drops_text(tmp_path):
    xml = tmp_path / "apm.pdef.xml"
    xml.write_text(
        """<paramfile><vehicles><parameters name="ArduCopter">
        <param humanName="Battery timer" name="ArduCopter:BATT_LOW_TIMER" documentation="Long text">
          <field name="Range">0 120</field><field name="Units">s</field>
          <field name="RebootRequired">True</field>
        </param>
        <param humanName="Mode" name="MODE_X" documentation="text">
          <values><value code="0">Off</value><value code="2">On</value></values>
        </param>
        <param humanName="Mask" name="MASK_X" documentation="text">
          <field name="Bitmask">0:Roll,1:Pitch,3:Yaw</field><field name="Range">oops</field>
        </param>
        </parameters></vehicles></paramfile>"""
    )
    assert extract(xml) == {
        "BATT_LOW_TIMER": {"r": [0, 120], "u": "s", "rb": 1},
        "MODE_X": {"v": [0, 2]},
        "MASK_X": {"b": [0, 1, 3]},  # the malformed range is left out
    }


def test_generator_merges_a_name_documented_twice(tmp_path):
    # Copter documents ATC_RAT_* in both the multirotor and the heli controller; the
    # heli one comes last and must not replace the multirotor's range or bits.
    xml = tmp_path / "apm.pdef.xml"
    xml.write_text(
        """<paramfile><libraries><parameters name="ATC_">
        <param name="ATC_RAT_RLL_I"><field name="Range">0.01 2.0</field></param>
        <param name="AUTOTUNE_AXES"><field name="Bitmask">0:Roll,1:Pitch,2:Yaw,3:YawD</field></param>
        <param name="ATC_RAT_RLL_I"><field name="Range">0.0 0.6</field></param>
        <param name="AUTOTUNE_AXES"><field name="Bitmask">0:Roll,1:Pitch,2:Yaw</field></param>
        </parameters></libraries></paramfile>"""
    )
    assert extract(xml) == {"ATC_RAT_RLL_I": {"r": [0, 2]}, "AUTOTUNE_AXES": {"b": [0, 1, 2, 3]}}


@pytest.fixture
def param_log(tmp_path):
    path = str(tmp_path / "p.bin")
    params = {"BATT_LOW_TIMER": 10.0, "INS_HNTCH_MODE": 1.0, "MYSCRIPT_GAIN": 2.0}
    msgs = [("ATT", {"TimeUS": i * 100_000, "Roll": 0.0}) for i in range(5)]
    return build_bin(path, msgs, params=params, firmware="ArduCopter V4.6.2 (synthetic)")


def test_get_params_attaches_documentation(param_log):
    r = get_params(param_log)
    assert r.metadata_version == "Copter 4.6.3 (same release series)"
    assert r.documented["BATT_LOW_TIMER"].range == [0, 120]
    assert r.undocumented == ["MYSCRIPT_GAIN"]


def test_get_params_skips_documentation_for_a_full_dump(param_log, monkeypatch):
    monkeypatch.setattr("ardupilot_mcp.server._DOC_LIMIT", 2)
    r = get_params(param_log)
    assert r.documented == {}
    assert r.undocumented == ["MYSCRIPT_GAIN"]
    assert "narrow with name_glob" in (r.note or "")


def test_load_param_file_documents_with_a_firmware(tmp_path):
    f = tmp_path / "x.param"
    f.write_text("BATT_LOW_TIMER,10\nBATT_LOW_TIMR,10\n")
    plain = load_param_file(str(f))
    assert plain.documented == {} and plain.metadata_version is None
    r = load_param_file(str(f), firmware="ArduCopter V4.6.3")
    assert r.metadata_version == "Copter 4.6.3 (exact)"
    assert r.metadata_url and "Copter/stable-4.6.3" in r.metadata_url
    assert r.undocumented == ["BATT_LOW_TIMR"]  # the typo stands out
    bare = load_param_file(str(f), firmware="4.6.3")  # no vehicle: nothing fits, and it says so
    assert bare.metadata_version is None and "no bundled parameter metadata" in (bare.note or "")
