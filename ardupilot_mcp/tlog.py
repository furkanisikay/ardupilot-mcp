"""MAVLink telemetry logs (``.tlog``) read into the same ``FlightLog`` the checks use.

A ground station records every MAVLink message it receives, each prefixed with the
reception time (pymavlink's ``mavlogfile``). The stream is the vehicle's telemetry, not
its onboard log: a few Hz instead of hundreds, the primary sensor and EKF core only,
and gaps wherever the radio link dropped. The messages are mapped onto the DataFlash
names and units the checks already read, from the firmware code that fills them:

* ``ATTITUDE`` (rad) -> ``ATT`` Roll/Pitch/Yaw (deg). The demand comes from
  ``NAV_CONTROLLER_OUTPUT`` where it is one (send_nav_controller_output in each vehicle's
  GCS_Mavlink.cpp): Copter sends its attitude controller's roll/pitch/yaw target; Plane
  its demanded roll and pitch, with nav_bearing the course, not a yaw target; Rover a roll
  of 0, so none is taken.
* ``VIBRATION`` -> ``VIBE`` VibeX/Y/Z and Clip0-2 (GCS_Common.cpp send_vibration: the same
  vibration levels and clip counters the VIBE message logs).
* ``EKF_STATUS_REPORT`` -> ``XKF4`` SV/SP/SH/SM/SVT: the variances the XKF4 message logs
  (AP_NavEKF3_Outputs.cpp send_status_report, AP_NavEKF3_Logging.cpp Log_Write_XKF4).
* ``SYS_STATUS`` -> ``BAT`` Volt (mV / 1000) and Curr (cA / 100; -1 = unknown).
* ``GPS_RAW_INT`` / ``GPS2_RAW`` -> ``GPS`` instance 0 / 1: fix type, satellites, HDop
  (eph / 100), speed (cm/s / 100).
* ``RAW_IMU`` / ``SCALED_IMU2`` / ``SCALED_IMU3`` -> ``IMU`` and ``MAG`` instances 0-2
  (accel mG -> m/s/s, gyro mrad/s -> rad/s, field in mGauss as MAG logs it).
* ``RC_CHANNELS`` -> ``RCIN`` C1-C18; ``SERVO_OUTPUT_RAW`` -> ``RCOU`` C1-C32 (port 1 holds
  outputs 17-32).
* ``VFR_HUD`` throttle (%) -> ``CTUN`` ThO (0-1); ``GLOBAL_POSITION_INT`` relative_alt ->
  ``POS`` RelHomeAlt (positions are not kept).
* ``HEARTBEAT`` -> ``MODE`` on each flight-mode change and ``ARM`` on each arm/disarm.
* ``STATUSTEXT`` -> ``MSG``; ``PARAM_VALUE`` -> the parameters.

Only the vehicle's autopilot is read: the system/component that sends the most
heartbeats with a real autopilot type, so a ground station or companion computer on
the same link never mixes into the vehicle's data.
"""

from __future__ import annotations

import math
from collections import Counter

import numpy as np

from . import ardupilot_meta as meta
from .flight_log import FlightLog, LogMeta
from .model import LogIntegrity

# MAVLink common.xml: MAV_AUTOPILOT_INVALID, MAV_TYPE_GCS, MAV_MODE_FLAG_SAFETY_ARMED.
_AUTOPILOT_INVALID = 8
_TYPE_GCS = 6
_ARMED_FLAG = 128
# Standard gravity used by GCS_Common.cpp to send accel in mG (GRAVITY_MSS).
_GRAVITY_MSS = 9.80665
# Design choice: the demand (NAV_CONTROLLER_OUTPUT, typically 2 Hz) is interpolated at each
# ATTITUDE sample (typically 4 Hz) between the demand samples either side of it, when those
# are at most this far apart. Holding the last demand instead would read a turning
# vehicle as lagging its demand by up to one sample period.
DEMAND_MAX_GAP_S = 1.0

# Axes of NAV_CONTROLLER_OUTPUT that are an attitude demand, by vehicle kind (see above).
_DEMANDED_AXES = {
    "copter": ("DesRoll", "DesPitch", "DesYaw"),
    "heli": ("DesRoll", "DesPitch", "DesYaw"),
    "plane": ("DesRoll", "DesPitch"),
}
# HEARTBEAT.type (MAVLink MAV_TYPE, minimal.xml) -> vehicle kind, for a tlog without the
# firmware banner.
_HEARTBEAT_KIND = {
    1: "plane", 2: "copter", 3: "copter", 4: "heli", 10: "rover", 11: "rover", 12: "sub",
    13: "copter", 14: "copter", 15: "copter", 19: "plane", 20: "plane", 21: "plane",
    22: "plane", 23: "plane", 24: "plane", 25: "plane", 29: "copter", 35: "copter",
}  # fmt: skip
# ArduPilot sends instance 0 as RAW_IMU, already scaled (GCS_Common.cpp send_raw_imu).
_IMU_SOURCES = {"RAW_IMU": 0, "SCALED_IMU2": 1, "SCALED_IMU3": 2}
_GPS_SOURCES = {"GPS_RAW_INT": 0, "GPS2_RAW": 1}
# Every message type parse_tlog maps (the rest are not decoded).
_MAPPED_TYPES = [
    "HEARTBEAT", "NAV_CONTROLLER_OUTPUT", "ATTITUDE", "VIBRATION", "EKF_STATUS_REPORT",
    "SYS_STATUS", "RC_CHANNELS", "SERVO_OUTPUT_RAW", "VFR_HUD", "GLOBAL_POSITION_INT",
    "STATUSTEXT", "PARAM_VALUE", *_IMU_SOURCES, *_GPS_SOURCES,
]  # fmt: skip


def is_tlog(path: str) -> bool:
    """A MAVLink telemetry log: ``.tlog`` by name, or a file not starting with DataFlash's header."""
    if path.lower().endswith(".tlog"):
        return True
    try:
        with open(path, "rb") as fh:
            head = fh.read(11)
    except OSError:
        return False
    # DataFlash records start with 0xA3 0x95; a tlog starts with an 8-byte timestamp and
    # then a MAVLink 1 (0xFE) or 2 (0xFD) start byte.
    return len(head) == 11 and head[:2] != b"\xa3\x95" and head[8] in (0xFE, 0xFD)


def _vehicle_source(messages: list) -> tuple[int, int] | None:
    beats: Counter[tuple[int, int]] = Counter()
    for m in messages:
        if m.get_type() == "HEARTBEAT" and m.autopilot != _AUTOPILOT_INVALID and m.type != _TYPE_GCS:
            beats[(m.get_srcSystem(), m.get_srcComponent())] += 1
    return beats.most_common(1)[0][0] if beats else None


def _num(v: float | int | None, scale: float = 1.0) -> float:
    if v is None:
        return math.nan
    return float(v) * scale


def _add_demand(
    att: list[dict], demands: list[tuple[float, float, float, float]], t0: float, axes: tuple[str, ...]
) -> None:
    """Give each ATT record the demand interpolated at its time (see DEMAND_MAX_GAP_S)."""
    if not att or len(demands) < 2 or not axes:
        return
    dt = np.array([d[0] - t0 for d in demands])
    cols = {
        "DesRoll": np.array([d[1] for d in demands]),
        "DesPitch": np.array([d[2] for d in demands]),
        # Unwrapped so a demand crossing north interpolates through 0, not through 180.
        "DesYaw": np.degrees(np.unwrap(np.radians([d[3] for d in demands]))),
    }
    t = np.array([r["TimeUS"] / 1e6 for r in att])
    # The bracketing demand pair; a sample exactly on the first or last demand is inside.
    after = np.clip(np.searchsorted(dt, t, side="left"), 1, dt.size - 1)
    inside = (t >= dt[0]) & (t <= dt[-1])
    gap = np.full(t.size, np.inf)
    gap[inside] = dt[after[inside]] - dt[after[inside] - 1]
    ok = inside & (gap <= DEMAND_MAX_GAP_S)
    for axis in axes:
        vals = np.interp(t, dt, cols[axis])
        if axis == "DesYaw":
            vals = vals % 360
        for rec, v, use in zip(att, vals, ok, strict=True):
            if use:
                rec[axis] = float(v)


def parse_tlog(path: str) -> FlightLog:
    """Read a MAVLink telemetry log into a FlightLog (see the module docstring)."""
    from pymavlink import mavutil  # local import keeps the dependency at the edge

    # The log reader is built directly: mavlink_connection() picks its transport from the
    # path string, so a path containing "/bin/" is executed as a program, one with a ":"
    # becomes a UDP socket, and a tlog named *.bin is read as DataFlash.
    mavutil.set_dialect("ardupilotmega")
    try:
        conn = mavutil.mavmmaplog(path)
    except Exception as exc:  # noqa: BLE001
        raise ValueError(f"could not open telemetry log {path!r}: {exc}") from exc
    raw = []
    try:
        # Only the mapped types are decoded (the mmap reader skips the rest undecoded).
        while (m := conn.recv_match(type=_MAPPED_TYPES)) is not None:
            raw.append(m)
    finally:
        conn.close()
    source = _vehicle_source(raw)
    if source is None:
        raise ValueError(f"no vehicle heartbeat in telemetry log {path!r}")
    own = [m for m in raw if (m.get_srcSystem(), m.get_srcComponent()) == source]
    vehicle_type = next((int(m.type) for m in own if m.get_type() == "HEARTBEAT"), -1)
    t0 = own[0]._timestamp if own else 0.0

    out: dict[str, list[dict]] = {}
    params: dict[str, float] = {}
    demands: list[tuple[float, float, float, float]] = []
    mode: int | None = None
    armed: bool | None = None

    def add(name: str, t: float, **fields: object) -> None:
        out.setdefault(name, []).append({"TimeUS": int(round((t - t0) * 1e6)), **fields})

    for m in own:
        t = float(m._timestamp)
        kind = m.get_type()
        if kind == "HEARTBEAT":
            now_armed = bool(m.base_mode & _ARMED_FLAG)
            # An edge on each change; the first heartbeat records only an armed start (a
            # ground station that connects mid-flight), as an onboard log has no disarm
            # record before the first arm either.
            if now_armed != armed and (armed is not None or now_armed):
                add("ARM", t, ArmState=int(now_armed))
            armed = now_armed
            if m.custom_mode != mode:
                add("MODE", t, Mode=int(m.custom_mode), ModeNum=int(m.custom_mode))
                mode = int(m.custom_mode)
        elif kind == "NAV_CONTROLLER_OUTPUT":
            demands.append((t, float(m.nav_roll), float(m.nav_pitch), float(m.nav_bearing)))
        elif kind == "ATTITUDE":
            add(
                "ATT",
                t,
                Roll=math.degrees(m.roll),
                Pitch=math.degrees(m.pitch),
                Yaw=math.degrees(m.yaw) % 360,
            )
        elif kind == "VIBRATION":
            add(
                "VIBE",
                t,
                VibeX=float(m.vibration_x),
                VibeY=float(m.vibration_y),
                VibeZ=float(m.vibration_z),
                Clip0=int(m.clipping_0),
                Clip1=int(m.clipping_1),
                Clip2=int(m.clipping_2),
            )
        elif kind == "EKF_STATUS_REPORT":
            add(
                "XKF4",
                t,
                SV=float(m.velocity_variance),
                SP=float(m.pos_horiz_variance),
                SH=float(m.pos_vert_variance),
                SM=float(m.compass_variance),
                SVT=_num(getattr(m, "airspeed_variance", None)),
            )
        elif kind == "SYS_STATUS":
            if m.voltage_battery not in (0, 65535):
                add(
                    "BAT",
                    t,
                    Volt=m.voltage_battery / 1000.0,
                    Curr=m.current_battery / 100.0 if m.current_battery >= 0 else math.nan,
                )
        elif kind in _GPS_SOURCES:
            add(
                "GPS",
                t,
                I=_GPS_SOURCES[kind],
                Status=int(m.fix_type),
                NSats=int(m.satellites_visible) if m.satellites_visible != 255 else math.nan,
                HDop=m.eph / 100.0 if m.eph != 65535 else math.nan,
                Spd=m.vel / 100.0 if m.vel != 65535 else math.nan,
            )
        elif kind in _IMU_SOURCES:
            inst = _IMU_SOURCES[kind]
            add(
                "IMU",
                t,
                I=inst,
                AccX=m.xacc * _GRAVITY_MSS / 1000.0,
                AccY=m.yacc * _GRAVITY_MSS / 1000.0,
                AccZ=m.zacc * _GRAVITY_MSS / 1000.0,
                GyrX=m.xgyro / 1000.0,
                GyrY=m.ygyro / 1000.0,
                GyrZ=m.zgyro / 1000.0,
            )
            if (m.xmag, m.ymag, m.zmag) != (0, 0, 0):
                add("MAG", t, I=inst, MagX=float(m.xmag), MagY=float(m.ymag), MagZ=float(m.zmag))
        elif kind == "RC_CHANNELS":
            add("RCIN", t, **{f"C{k}": int(getattr(m, f"chan{k}_raw")) for k in range(1, 19)})
        elif kind == "SERVO_OUTPUT_RAW":
            base = 16 * int(getattr(m, "port", 0) or 0)
            outs = {f"C{base + k}": int(getattr(m, f"servo{k}_raw", 0) or 0) for k in range(1, 17)}
            add("RCOU", t, **outs)
        elif kind == "VFR_HUD":
            add("CTUN", t, ThO=float(m.throttle) / 100.0)
        elif kind == "GLOBAL_POSITION_INT":
            add("POS", t, RelHomeAlt=m.relative_alt / 1000.0)
        elif kind == "STATUSTEXT":
            add("MSG", t, Message=str(m.text))
        elif kind == "PARAM_VALUE":
            name = str(m.param_id).rstrip("\x00")
            if name and math.isfinite(float(m.param_value)):
                params[name] = float(m.param_value)

    vehicle = next(
        (v for r in out.get("MSG", []) if (v := meta.vehicle_from_message(str(r["Message"]))) is not None),
        None,
    )
    kind_hint = meta.vehicle_kind(vehicle) if vehicle else _HEARTBEAT_KIND.get(vehicle_type, "unknown")
    _add_demand(out.get("ATT", []), demands, t0, _DEMANDED_AXES.get(kind_hint, ()))

    log = FlightLog(
        messages=out,
        params=params,
        meta=LogMeta(integrity=LogIntegrity.OK, source="tlog", arming_recorded=True),
        path=path,
        instance_fields={"GPS": "I", "IMU": "I", "MAG": "I"},
    )
    from .parser import _extract_firmware, _extract_max_alt, _extract_modes

    fw, vehicle = _extract_firmware(log)
    kind = meta.vehicle_kind(vehicle, log.params.get("FRAME_CLASS")) if vehicle else kind_hint
    log.meta.vehicle_type = vehicle
    log.meta.vehicle_kind = kind
    log.meta.firmware_version = fw
    log.meta.flight_modes = _extract_modes(log, kind)
    log.meta.max_altitude_m = _extract_max_alt(log)
    return log
