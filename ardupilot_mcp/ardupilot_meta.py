"""Static ArduPilot reference data used by the parser and the events check.

Kept deliberately small and in one place so decoding (mode numbers, ERR
subsystems, EV ids) is consistent everywhere. Sourced from ArduPilot's
``LogStructure``/mode enumerations.
"""

from __future__ import annotations

# ArduCopter flight modes (mode number -> name). Copter is the MVP focus.
COPTER_MODES: dict[int, str] = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
}

# ERR subsystem id -> name.
ERR_SUBSYSTEMS: dict[int, str] = {
    1: "MAIN",
    2: "RADIO",
    3: "COMPASS",
    4: "OPTFLOW",
    5: "FAILSAFE_RADIO",
    6: "FAILSAFE_BATT",
    7: "FAILSAFE_GPS",
    8: "FAILSAFE_GCS",
    9: "FAILSAFE_FENCE",
    10: "FLIGHT_MODE",
    11: "GPS",
    12: "CRASH_CHECK",
    13: "FLIP",
    14: "AUTOTUNE",
    15: "PARACHUTE",
    16: "EKFCHECK",
    17: "FAILSAFE_EKFINAV",
    18: "BARO",
    19: "CPU",
    20: "FAILSAFE_ADSB",
    21: "TERRAIN",
    22: "NAVIGATION",
    23: "FAILSAFE_TERRAIN",
    24: "EKF_PRIMARY",
    25: "THRUST_LOSS_CHECK",
    26: "FAILSAFE_SENSORS",
    27: "FAILSAFE_LEAK",
    28: "PILOT_INPUT",
    29: "FAILSAFE_VIBE",
    30: "INTERNAL_ERROR",
    31: "FAILSAFE_DEADRECKON",
}

# Subsystems whose non-zero ECode is a hard, safety-relevant failure.
CRITICAL_ERR_SUBSYSTEMS: set[int] = {
    5,  # FAILSAFE_RADIO
    6,  # FAILSAFE_BATT
    7,  # FAILSAFE_GPS
    9,  # FAILSAFE_FENCE
    12,  # CRASH_CHECK
    16,  # EKFCHECK
    17,  # FAILSAFE_EKFINAV
    25,  # THRUST_LOSS_CHECK
    29,  # FAILSAFE_VIBE
    30,  # INTERNAL_ERROR
}

# EV (event) id -> name (useful subset). Values verified against ArduPilot's
# LogEvent enum in AP_Logger.h (Copter-4.5.7):
# https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/AP_Logger.h
# (NB: there is no TAKEOFF event id; AUTOTUNE_* occupy 30-37; FENCE_ENABLE is 41,
#  FENCE_FLOOR_ENABLE is 80; EKF_ALT_RESET/EKF_YAW_RESET are 60/62.)
EV_IDS: dict[int, str] = {
    10: "ARMED",
    11: "DISARMED",
    15: "AUTO_ARMED",
    17: "LAND_COMPLETE_MAYBE",
    18: "LAND_COMPLETE",
    19: "LOST_GPS",
    21: "FLIP_START",
    22: "FLIP_END",
    25: "SET_HOME",
    26: "SET_SIMPLE_ON",
    27: "SET_SIMPLE_OFF",
    28: "NOT_LANDED",
    29: "SET_SUPERSIMPLE_ON",
    30: "AUTOTUNE_INITIALISED",
    31: "AUTOTUNE_OFF",
    33: "AUTOTUNE_SUCCESS",
    34: "AUTOTUNE_FAILED",
    41: "FENCE_ENABLE",
    42: "FENCE_DISABLE",
    49: "PARACHUTE_DISABLED",
    51: "PARACHUTE_RELEASED",
    54: "MOTORS_EMERGENCY_STOPPED",
    58: "ROTOR_RUNUP_COMPLETE",
    59: "ROTOR_SPEED_BELOW_CRITICAL",
    60: "EKF_ALT_RESET",
    61: "LAND_CANCELLED_BY_PILOT",
    62: "EKF_YAW_RESET",
    67: "GPS_PRIMARY_CHANGED",
    71: "ZIGZAG_STORE_A",
    80: "FENCE_FLOOR_ENABLE",
}

# Firmware-string prefix -> canonical vehicle name. Covers the modern form
# ("ArduCopter V4.5.7") and the legacy form older logs use ("APM:Copter V3.4.3").
_VEHICLE_PREFIX_MAP = {
    "ArduCopter": "ArduCopter",
    "APM:Copter": "ArduCopter",
    "ArduPlane": "ArduPlane",
    "APM:Plane": "ArduPlane",
    "ArduRover": "Rover",
    "APM:Rover": "Rover",
    "Rover": "Rover",
    "ArduSub": "ArduSub",
    "APM:Sub": "ArduSub",
    "Blimp": "Blimp",
    "AntennaTracker": "AntennaTracker",
    "APM:Tracker": "AntennaTracker",
}


def mode_name(mode_num: int) -> str:
    return COPTER_MODES.get(int(mode_num), f"MODE_{int(mode_num)}")


def err_subsystem_name(subsys: int) -> str:
    return ERR_SUBSYSTEMS.get(int(subsys), f"SUBSYS_{int(subsys)}")


def ev_name(ev_id: int) -> str:
    return EV_IDS.get(int(ev_id), f"EV_{int(ev_id)}")


def vehicle_from_message(text: str) -> str | None:
    for pref, canonical in _VEHICLE_PREFIX_MAP.items():
        if text.startswith(pref):
            return canonical
    return None


# FRAME_CLASS values that mean a traditional helicopter (single/dual/quad heli).
# Verified against the Copter FRAME_CLASS param values: 6=Heli, 11=Heli_Dual,
# 13=Heli_Quad (14 is Deca, a MULTIROTOR — not a heli).
# https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml
HELI_FRAME_CLASSES = {6, 11, 13}

# Copter FRAME_CLASS -> (name, lift-motor count). None count = not a simple multirotor.
FRAME_CLASSES: dict[int, tuple[str, int | None]] = {
    1: ("Quad", 4),
    2: ("Hexa", 6),
    3: ("Octo", 8),
    4: ("OctoQuad", 8),
    5: ("Y6", 6),
    6: ("Heli", None),
    7: ("Tri", 3),
    8: ("Single", 1),
    9: ("Coax", 2),
    10: ("BiCopter", 2),
    11: ("Heli_Dual", None),
    12: ("DodecaHexa", 12),
    13: ("HeliQuad", None),
    14: ("Deca", 10),
}

# Copter FRAME_TYPE -> name (geometry).
FRAME_TYPES: dict[int, str] = {
    0: "Plus",
    1: "X",
    2: "V",
    3: "H",
    4: "V-Tail",
    5: "A-Tail",
    10: "Y6B",
    11: "Y6F",
    12: "BetaFlightX",
    13: "DJIX",
    14: "ClockwiseX",
    15: "I",
    18: "BetaFlightXReversed",
    19: "Y4",
}


def frame_class_name(frame_class: float | int | None) -> str | None:
    if frame_class is None:
        return None
    entry = FRAME_CLASSES.get(int(frame_class))
    return entry[0] if entry else f"FrameClass{int(frame_class)}"


def motor_count_for_frame(frame_class: float | int | None) -> int | None:
    if frame_class is None:
        return None
    entry = FRAME_CLASSES.get(int(frame_class))
    return entry[1] if entry else None


def frame_type_name(frame_type: float | int | None) -> str | None:
    if frame_type is None:
        return None
    return FRAME_TYPES.get(int(frame_type), f"FrameType{int(frame_type)}")


def estimate_cells(max_voltage: float | None) -> int | None:
    """Estimate LiPo series-cell count from peak pack voltage (4.2 V/cell full).

    Returns None when the voltage is implausible (no real monitor / odd scale) or
    the implied per-cell voltage is out of a sane resting range.
    """
    if max_voltage is None or max_voltage < 4.0 or max_voltage > 70.0:
        return None
    cells = round(max_voltage / 4.2)
    if cells < 1:
        return None
    per_cell = max_voltage / cells
    return cells if 3.3 <= per_cell <= 4.4 else None


def vehicle_kind(vehicle_type: str | None, frame_class: float | int | None = None) -> str:
    """Coarse vehicle category that drives which checks apply.

    Returns one of: ``copter`` (multirotor), ``heli`` (traditional helicopter,
    a Copter firmware with a heli FRAME_CLASS), ``plane`` (incl. QuadPlane/VTOL,
    which run ArduPlane), ``rover``, ``sub``, ``tracker``, or ``unknown``.
    """
    vt = (vehicle_type or "").lower()
    if "plane" in vt:
        return "plane"
    if "rover" in vt:
        return "rover"
    if "sub" in vt:
        return "sub"
    if "tracker" in vt:
        return "tracker"
    if "copter" in vt:
        try:
            if frame_class is not None and int(frame_class) in HELI_FRAME_CLASSES:
                return "heli"
        except (TypeError, ValueError):
            pass
        return "copter"
    return "unknown"
