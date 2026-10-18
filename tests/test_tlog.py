"""MAVLink telemetry logs (ardupilot_mcp.tlog) read into a FlightLog."""

from __future__ import annotations

import math

from pymavlink.dialects.v20 import ardupilotmega as mav

from ardupilot_mcp.checks.integrity import IntegrityCheck
from ardupilot_mcp.checks.timing import TimingCheck
from ardupilot_mcp.model import CheckStatus
from ardupilot_mcp.server import analyze_log, log_summary
from ardupilot_mcp.tlog import is_tlog, parse_tlog
from tests.synth_tlog import GCS, build_tlog, heartbeat


def _flight(tmp_path, *, vibe_z: float = 15.0, name: str = "flight.tlog"):
    msgs: list[tuple[float, object]] = [
        (0.0, mav.MAVLink_statustext_message(6, b"ArduCopter V4.6.3 (92b0cd78)")),
        (0.1, mav.MAVLink_param_value_message(b"BATT_LOW_VOLT", 10.5, 9, 2, 0)),
        (0.2, mav.MAVLink_param_value_message(b"FRAME_CLASS", 1, 2, 2, 1)),
        (0.5, heartbeat(0, False)),
        (0.6, heartbeat(0, False, gcs=True)),  # the ground station's own heartbeat
    ]
    source = {4: GCS}
    for i in range(40):
        t = 1.0 + i * 0.25
        armed = 2.0 <= t < 10.0
        if i % 4 == 0:
            msgs.append((t, heartbeat(5 if t >= 6.0 else 0, armed)))
        msgs.append((t, mav.MAVLink_nav_controller_output_message(10.0, -5.0, 90, 0, 0, 0, 0, 0)))
        msgs.append(
            (
                t + 0.01,
                mav.MAVLink_attitude_message(
                    int(t * 1000), math.radians(9.0), math.radians(-4.0), math.radians(91.0), 0, 0, 0
                ),
            )
        )
        msgs.append((t, mav.MAVLink_vibration_message(int(t * 1e6), 10.0, 11.0, vibe_z, 0, 0, 0)))
        msgs.append((t, mav.MAVLink_ekf_status_report_message(0, 0.2, 0.1, 0.3, 0.05, 0, 0)))
        msgs.append((t, mav.MAVLink_sys_status_message(0, 0, 0, 500, 12_400, 1_500, 80, 0, 0, 0, 0, 0, 0)))
        msgs.append((t, mav.MAVLink_gps_raw_int_message(int(t * 1e6), 3, 0, 0, 0, 80, 65535, 120, 0, 14)))
        msgs.append((t, mav.MAVLink_gps2_raw_message(int(t * 1e6), 1, 0, 0, 0, 200, 65535, 0, 0, 5, 0, 0)))
        msgs.append((t, mav.MAVLink_vfr_hud_message(0, 0, 90, 35, 5.0, 0)))
    # Another vehicle on the same link: never read.
    msgs.append((5.0, mav.MAVLink_attitude_message(5000, 1.0, 1.0, 1.0, 0, 0, 0)))
    source[len(msgs) - 1] = (2, 1)
    return build_tlog(str(tmp_path / name), msgs, source)


def test_is_tlog_by_name_and_content(tmp_path):
    path = _flight(tmp_path, name="noext")
    assert is_tlog(path)
    assert is_tlog(str(tmp_path / "x.tlog"))
    bin_path = tmp_path / "x.bin"
    bin_path.write_bytes(b"\xa3\x95\x80" + b"\x00" * 20)
    assert not is_tlog(str(bin_path))


def test_messages_map_onto_dataflash_names_and_units(tmp_path):
    log = parse_tlog(_flight(tmp_path))
    assert log.meta.source == "tlog"
    assert log.meta.vehicle_kind == "copter" and log.meta.firmware_version.startswith("ArduCopter V4.6.3")
    assert log.params["BATT_LOW_VOLT"] == 10.5
    att = log.get("ATT")[0]
    assert abs(att["Roll"] - 9.0) < 1e-3 and att["DesRoll"] == 10.0 and abs(att["Yaw"] - 91.0) < 1e-3
    assert log.get("VIBE")[0]["VibeZ"] == 15.0
    assert abs(log.get("XKF4")[0]["SV"] - 0.2) < 1e-6  # MAVLink floats are float32
    bat = log.get("BAT")[0]
    assert bat["Volt"] == 12.4 and bat["Curr"] == 15.0
    assert log.instances("GPS") == [0, 1]
    assert log.get("GPS", 0)[0]["HDop"] == 0.8 and log.get("GPS", 0)[0]["NSats"] == 14
    assert log.get("CTUN")[0]["ThO"] == 0.35
    # Only the vehicle: the ground station's heartbeat and the other system's attitude are dropped.
    assert all(abs(r["Roll"] - 9.0) < 1e-3 for r in log.get("ATT"))


def test_arming_and_modes_come_from_heartbeats(tmp_path):
    log = parse_tlog(_flight(tmp_path))
    ivs = log.armed_intervals()
    assert len(ivs) == 1
    t0, t1 = ivs[0]
    # Armed from t=2 s to t=10 s of the recording (times are rebased to the log's start).
    assert abs((t1 - t0) - 8.0) < 0.3
    assert log.meta.flight_modes == ["STABILIZE", "LOITER"]


def test_link_dependent_checks_do_not_run_on_telemetry(tmp_path):
    log = parse_tlog(_flight(tmp_path))
    for check in (IntegrityCheck(), TimingCheck()):
        assert check.execute(log).status == CheckStatus.SKIPPED


def test_the_server_diagnoses_a_tlog(tmp_path):
    path = _flight(tmp_path, vibe_z=75.0)
    report = analyze_log(path)
    assert any(f.check_id == "vibration" for f in report.findings)
    assert log_summary(path).message_counts["ATT"] == 40


def test_the_demand_is_interpolated_at_each_attitude_sample(tmp_path):
    # Demand ramps 0 -> 20 deg over 1 s (two samples); the attitude between them sees 10.
    msgs = [
        (0.0, mav.MAVLink_statustext_message(6, b"ArduCopter V4.6.3 (92b0cd78)")),
        (0.0, heartbeat(0, True)),
        (1.0, mav.MAVLink_nav_controller_output_message(0.0, 0.0, 350, 0, 0, 0, 0, 0)),
        (1.5, mav.MAVLink_attitude_message(1500, math.radians(10.0), 0, math.radians(0.0), 0, 0, 0)),
        (2.0, mav.MAVLink_nav_controller_output_message(20.0, 0.0, 10, 0, 0, 0, 0, 0)),
        (4.5, mav.MAVLink_attitude_message(4500, 0, 0, 0, 0, 0, 0)),  # no demand after it
        (6.0, mav.MAVLink_nav_controller_output_message(0.0, 0.0, 0, 0, 0, 0, 0, 0)),
    ]
    log = parse_tlog(build_tlog(str(tmp_path / "demand.tlog"), msgs))
    first, second = log.get("ATT")
    assert abs(first["DesRoll"] - 10.0) < 1e-6
    assert abs(first["DesYaw"] - 0.0) < 1e-6  # 350 -> 10 interpolates across north
    assert "DesRoll" not in second  # the demand samples around it are 4 s apart


def test_a_plane_takes_roll_and_pitch_demand_only(tmp_path):
    msgs = [
        (0.0, mav.MAVLink_statustext_message(6, b"ArduPlane V4.6.3 (92b0cd78)")),
        (0.0, heartbeat(0, True)),
        (1.0, mav.MAVLink_nav_controller_output_message(5.0, 2.0, 90, 0, 0, 0, 0, 0)),
        (1.2, mav.MAVLink_attitude_message(1200, 0, 0, 0, 0, 0, 0)),
        (1.5, mav.MAVLink_nav_controller_output_message(5.0, 2.0, 90, 0, 0, 0, 0, 0)),
    ]
    rec = parse_tlog(build_tlog(str(tmp_path / "plane.tlog"), msgs)).get("ATT")[0]
    assert rec["DesRoll"] == 5.0 and rec["DesPitch"] == 2.0
    assert "DesYaw" not in rec  # nav_bearing is the course, not a yaw demand


def test_gps_before_arming_is_not_a_flight_fault(tmp_path):
    # Satellites at 0 during boot (before arming) must not raise a satellite finding.
    msgs = [(0.0, mav.MAVLink_statustext_message(6, b"ArduCopter V4.6.3 (92b0cd78)"))]
    for i in range(40):
        t = i * 0.5
        armed = t >= 10.0
        msgs.append((t, heartbeat(0, armed)))
        sats, fix = (0, 0) if t < 5.0 else (14, 3)
        msgs.append((t, mav.MAVLink_gps_raw_int_message(int(t * 1e6), fix, 0, 0, 0, 80, 65535, 0, 0, sats)))
        msgs.append((t, mav.MAVLink_attitude_message(int(t * 1000), 0, 0, 0, 0, 0, 0)))
    report = analyze_log(build_tlog(str(tmp_path / "boot.tlog"), msgs))
    assert not [f for f in report.findings if f.check_id == "gps"]


def test_telemetry_that_stops_while_armed_is_not_an_impact(tmp_path):
    from ardupilot_mcp.server import incident_timeline

    msgs = [(0.0, mav.MAVLink_statustext_message(6, b"ArduCopter V4.6.3 (92b0cd78)"))]
    for i in range(20):
        t = i * 0.5
        msgs.append((t, heartbeat(0, t >= 2.0)))  # armed, and the recording stops armed
        msgs.append((t, mav.MAVLink_attitude_message(int(t * 1000), 0, 0, 0, 0, 0, 0)))
    tl = incident_timeline(build_tlog(str(tmp_path / "cut.tlog"), msgs))
    assert tl.impact_s is None
    assert any("telemetry stopped" in c for c in tl.caveats)
