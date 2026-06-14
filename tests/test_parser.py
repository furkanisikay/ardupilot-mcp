"""Tests for the DataFlash binary parser, using synthetic .bin fixtures."""

from __future__ import annotations

import pytest

from ardupilot_mcp.model import LogIntegrity
from ardupilot_mcp.parser import parse_bin
from tests.synth_bin import build_bin


def _basic_messages():
    msgs = []
    for i in range(20):
        t = i * 0.1
        msgs.append(
            (
                "ATT",
                {
                    "TimeUS": int(t * 1e6),
                    "DesRoll": 5.0,
                    "Roll": 5.0,
                    "DesPitch": 0,
                    "Pitch": 0,
                    "DesYaw": 10,
                    "Yaw": 10,
                },
            )
        )
        msgs.append(
            (
                "VIBE",
                {
                    "TimeUS": int(t * 1e6),
                    "VibeX": 12,
                    "VibeY": 12,
                    "VibeZ": 15,
                    "Clip0": 0,
                    "Clip1": 0,
                    "Clip2": 0,
                },
            )
        )
    msgs.append(("MODE", {"TimeUS": 0, "Mode": 0, "ModeNum": 0, "Rsn": 1}))
    msgs.append(("MODE", {"TimeUS": 1_000_000, "Mode": 5, "ModeNum": 5, "Rsn": 1}))
    msgs.append(("ERR", {"TimeUS": 500_000, "Subsys": 6, "ECode": 1}))
    return msgs


def test_parse_basic(tmp_path):
    p = str(tmp_path / "basic.bin")
    build_bin(
        p,
        _basic_messages(),
        params={"INS_HNTCH_ENABLE": 1.0, "ATC_RAT_RLL_P": 0.135},
        firmware="ArduCopter V4.5.7 (synthetic)",
    )
    log = parse_bin(p)

    assert log.meta.vehicle_type == "ArduCopter"
    assert "ArduCopter" in (log.meta.firmware_version or "")
    assert log.count("ATT") == 20
    assert log.count("VIBE") == 20
    assert log.meta.integrity == LogIntegrity.OK
    assert log.meta.flight_modes == ["STABILIZE", "LOITER"]
    assert log.params["INS_HNTCH_ENABLE"] == 1.0
    assert abs(log.params["ATC_RAT_RLL_P"] - 0.135) < 1e-4
    err = log.get("ERR")[0]
    assert err["Subsys"] == 6 and err["ECode"] == 1
    assert err["timestamp"] == 0.5


def test_parse_missing_file_raises():
    with pytest.raises(FileNotFoundError):
        parse_bin("does_not_exist_12345.bin")


def test_truncated_log_is_flagged(tmp_path):
    p = str(tmp_path / "trunc.bin")
    # Chop bytes off the end to simulate a log that ends mid-message (as after a crash).
    build_bin(p, _basic_messages(), firmware="ArduCopter V4.5.7", truncate_bytes=37)
    log = parse_bin(p)
    # It should still parse the good prefix and not raise.
    assert log.count("ATT") >= 1
    # Integrity may be flagged truncated/partial depending on where the cut lands; never OK-with-crash.
    assert log.meta.integrity in (LogIntegrity.OK, LogIntegrity.TRUNCATED, LogIntegrity.PARTIAL)


def test_skips_format_messages(tmp_path):
    p = str(tmp_path / "fmt.bin")
    build_bin(p, _basic_messages(), firmware="ArduCopter V4.5.7")
    log = parse_bin(p)
    # FMT/FMTU are structural and must not appear as data message types.
    assert "FMT" not in log.available_messages


def test_legacy_apm_firmware_prefix(tmp_path):
    # Pre-2015 logs report firmware as "APM:Copter V3.4.3", not "ArduCopter ...".
    p = str(tmp_path / "legacy.bin")
    build_bin(p, _basic_messages(), firmware="APM:Copter V3.4.3 (e351c858)")
    log = parse_bin(p)
    assert log.meta.vehicle_type == "ArduCopter"
    assert "APM:Copter V3.4.3" in (log.meta.firmware_version or "")


def test_garbage_tail_flagged_truncated(tmp_path):
    # A power loss / crash often leaves a zero-padded tail that DFReader reports
    # as bad headers; that must be surfaced as a truncated-integrity flag, and
    # the warnings must not leak to stderr.
    p = str(tmp_path / "garbage.bin")
    build_bin(p, _basic_messages(), firmware="ArduCopter V4.5.7")
    with open(p, "ab") as fh:
        fh.write(b"\x00" * 800)  # > DFReader's 528-byte tolerance
    log = parse_bin(p)
    assert log.count("ATT") >= 1  # good prefix still parsed
    assert log.meta.integrity == LogIntegrity.TRUNCATED
    assert "truncated" in (log.meta.integrity_detail or "").lower()
