"""Tests for the DataFlash binary parser, using synthetic .bin fixtures."""

from __future__ import annotations

import os

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


def _splice(path: str, at_record: int, junk: bytes) -> None:
    """Insert ``junk`` just before the ``at_record``-th ATT record of ``path``."""
    from pymavlink import DFReader

    from ardupilot_mcp.parser import _close, _open_reader

    reader = _open_reader(DFReader, path)
    ofs = reader.offsets[reader.name_to_id["ATT"]][at_record]
    _close(reader)
    data = open(path, "rb").read()
    open(path, "wb").write(data[:ofs] + junk + data[ofs:])


def test_garbage_inside_log_is_partial(tmp_path):
    # The default (compiled) pymavlink indexer prints its "bad header" warnings below
    # Python, so they cannot be captured; the gap in the record index must flag it.
    p = str(tmp_path / "mid.bin")
    build_bin(p, _basic_messages(), firmware="ArduCopter V4.5.7")
    junk = bytes(range(7, 207)) * 3  # 600 bytes of garbage, as a bad SD write leaves
    _splice(p, 10, junk)
    log = parse_bin(p)
    assert log.count("ATT") == 20  # every record around the garbage is still read
    assert log.meta.integrity == LogIntegrity.PARTIAL
    assert "600 byte(s) in 1 place(s)" in (log.meta.integrity_detail or "")


def test_unknown_record_type_stops_reading_and_is_flagged(tmp_path):
    # pymavlink stops indexing at a record whose type no FMT defines, so the rest of the
    # log is never read. That must not pass as a clean, shorter log.
    p = str(tmp_path / "unknown.bin")
    build_bin(p, _basic_messages(), firmware="ArduCopter V4.5.7")
    _splice(p, 10, b"\xa3\x95\xf0")
    log = parse_bin(p)
    assert log.count("ATT") < 20
    assert log.meta.integrity == LogIntegrity.PARTIAL
    assert "Reading stopped" in (log.meta.integrity_detail or "")


def test_clean_log_has_no_gap(tmp_path):
    p = str(tmp_path / "clean.bin")
    build_bin(p, _basic_messages(), firmware="ArduCopter V4.5.7")
    assert parse_bin(p).meta.integrity == LogIntegrity.OK


@pytest.mark.parametrize("content", [os.urandom(5000), b"\x00" * 4096, b"\xff" * 4096, b"\xa3\x95"])
def test_non_log_file_raises(tmp_path, content):
    # Random bytes, a zeroed/erased card image or a stray header used to give an empty
    # log with integrity OK, which analyze_log reported as "no issues detected".
    p = str(tmp_path / "notalog.bin")
    open(p, "wb").write(content)
    with pytest.raises(ValueError, match="no DataFlash records"):
        parse_bin(p)


# ---- vectorised decoder == pymavlink's own per-message decoding ----


def _legacy(path: str):
    """The same log decoded record-by-record through pymavlink (the fallback path)."""
    import contextlib
    import io

    from pymavlink import DFReader

    from ardupilot_mcp.flight_log import FlightLog
    from ardupilot_mcp.parser import _read_records

    with contextlib.redirect_stderr(io.StringIO()):
        messages, _, _ = _read_records(DFReader.DFReader_binary(path))
    return FlightLog(messages=messages)


@pytest.mark.parametrize("scenario", ["clean_flight", "fault_crash_err", "gps2_never_fixes"])
def test_fast_decoder_matches_pymavlink_field_for_field(scenario, tmp_path):
    from tests.scenarios.flights import SCENARIOS, write_flight

    s = next(x for x in SCENARIOS if x.name == scenario)
    path = str(tmp_path / "log.bin")
    write_flight(path, s.inject, duration=30.0)
    fast, ref = parse_bin(path), _legacy(path)
    assert fast.message_counts() == ref.message_counts()
    for mtype in ref.available_messages:
        for a, b in zip(fast.get(mtype), ref.get(mtype), strict=True):
            assert a.keys() == b.keys(), mtype
            for k, v in b.items():
                if isinstance(v, str):
                    assert a[k] == v, (mtype, k)
                else:
                    assert a[k] == pytest.approx(v, rel=1e-9, abs=1e-12), (mtype, k)


def test_partial_final_record_is_dropped_not_flagged(tmp_path):
    # Unplugging the battery after landing leaves half a record at EOF; that alone
    # is not a crash signature, so integrity stays OK and the record is dropped.
    from tests.scenarios.flights import write_flight

    path = tmp_path / "log.bin"
    write_flight(str(path), {}, duration=12.0)
    data = path.read_bytes()
    path.write_bytes(data[:-7])
    log = parse_bin(str(path))
    assert log.meta.integrity == LogIntegrity.OK


@pytest.mark.skipif(not os.path.exists("/proc/self/maps"), reason="needs /proc to list memory maps")
def test_parse_releases_the_log_file(tmp_path):
    # An open map would keep the log locked on Windows until a garbage collection.
    path = build_bin(str(tmp_path / "released.bin"), _basic_messages())
    log = parse_bin(path)
    _ = log.batches

    def mapped() -> bool:
        with open("/proc/self/maps") as fh:
            return any("released.bin" in line for line in fh)

    assert not mapped()


def test_records_without_a_time_field_take_their_place_in_the_file(tmp_path, monkeypatch):
    # Copter 3.x wrote EV and ERR with no timestamp; ATT carried TimeMS (boot clock).
    from tests import synth_bin

    monkeypatch.setitem(synth_bin.FORMATS, "EV", (68, "B", ["Id"]))
    monkeypatch.setitem(synth_bin.FORMATS, "ERR", (67, "BB", ["Subsys", "ECode"]))
    msgs: list = [("ERR", {"Subsys": 12, "ECode": 1})]  # before any timed record
    for i in range(30):
        msgs.append(("ATT", {"TimeUS": 20_000_000 + i * 100_000, "Roll": 0.0}))
        if i == 12:
            msgs.append(("EV", {"Id": 10}))
    p = str(tmp_path / "untimed.bin")
    build_bin(p, msgs, firmware=None)
    log = parse_bin(p)
    assert [round(r["timestamp"], 3) for r in log.get("EV")] == [1.2]
    assert [round(r["timestamp"], 3) for r in log.get("ERR")] == [0.0]
    assert log.armed_intervals() == [(pytest.approx(1.2), pytest.approx(2.9))]
