# SPDX-License-Identifier: AGPL-3.0-only
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
    28: "TURTLE",
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

# ERR EKF_PRIMARY carries the new primary core as its ECode, not an error code
# (ArduCopter/ekf_check.cpp: LOGGER_WRITE_ERROR(LogErrorSubsystem::EKF_PRIMARY, LogErrorCode(ekf_primary_core))).
ERR_EKF_PRIMARY = 24

# ERR FLIGHT_MODE carries the mode that could not be entered as its ECode
# (ArduCopter/mode.cpp mode_change_failed: LOGGER_WRITE_ERROR(FLIGHT_MODE, mode->mode_number())).
ERR_FLIGHT_MODE = 10

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
    26,  # FAILSAFE_SENSORS (Sub: depth sensor failed, mode forced to MANUAL)
    27,  # FAILSAFE_LEAK (Sub: water in the hull; ArduSub/failsafe.cpp)
    30,  # INTERNAL_ERROR
}
# Not FAILSAFE_VIBE (29): its only action is to switch the position controller to
# vibration-resistant gains, with no mode change, and it fires on an EKF pattern (vertical
# velocity and position innovations both positive, velocity variance above 1, for 1 s) that a
# fall produces as well as vibration (Copter::check_vibration, ArduCopter/ekf_check.cpp,
# Copter-4.6.3). It is reported as a warning, as its "Vibration compensation ON" text is.

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


# Mode numbers are per-vehicle: the same number means different modes on Copter,
# Plane, Rover and Sub (e.g. 0 = STABILIZE on Copter but MANUAL on Plane).
# Source: the `enum class Number` in ArduPlane/mode.h, Rover/mode.h and
# ArduSub/mode.h at Copter-4.6.3 (same release train as the Copter table above).
# https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/ArduPlane/mode.h
PLANE_MODES: dict[int, str] = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    13: "TAKEOFF",
    14: "AVOID_ADSB",
    15: "GUIDED",
    16: "INITIALISING",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
    24: "THERMAL",
    25: "LOITER_ALT_QLAND",
}

# https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/Rover/mode.h
ROVER_MODES: dict[int, str] = {
    0: "MANUAL",
    1: "ACRO",
    3: "STEERING",
    4: "HOLD",
    5: "LOITER",
    6: "FOLLOW",
    7: "SIMPLE",
    8: "DOCK",
    9: "CIRCLE",
    10: "AUTO",
    11: "RTL",
    12: "SMART_RTL",
    15: "GUIDED",
    16: "INITIALISING",
}

# https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/ArduSub/mode.h
SUB_MODES: dict[int, str] = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    7: "CIRCLE",
    9: "SURFACE",
    16: "POSHOLD",
    19: "MANUAL",
    20: "MOTOR_DETECT",
    21: "SURFTRAK",  # ArduSub/mode.h (Copter-4.6.3 tree)
}

_MODES_BY_KIND: dict[str, dict[int, str]] = {
    "copter": COPTER_MODES,
    "heli": COPTER_MODES,
    "plane": PLANE_MODES,
    "rover": ROVER_MODES,
    "sub": SUB_MODES,
}


# ARM.Method: how the vehicle was armed or disarmed (AP_Arming::Method, Copter-4.6.3
# libraries/AP_Arming/AP_Arming.h). Methods 5 and up are written by disarms only.
ARM_METHODS: dict[int, str] = {
    0: "rudder stick",
    1: "MAVLink command",
    2: "RC switch",
    3: "motor test",
    4: "script",
    5: "termination",
    6: "CPU failsafe",
    7: "battery failsafe",
    8: "Solo pause when landed",
    9: "advanced failsafe",
    10: "ADS-B collision action",
    11: "parachute release",
    12: "crash",
    13: "landed",
    14: "mission exit",
    15: "fence breach",
    16: "radio failsafe",
    17: "disarm delay",
    18: "GCS failsafe",
    19: "terrain failsafe",
    20: "failsafe action terminate",
    21: "terrain failsafe",
    22: "motor detect done",
    23: "bad flow of control",
    24: "EKF failsafe",
    25: "GCS failsafe surface failed",
    26: "GCS failsafe hold failed",
    27: "takeoff timeout",
    28: "auto landed",
    29: "pilot input failsafe",
    30: "toy mode land throttle",
    31: "toy mode land force",
    32: "landing",
    33: "dead-reckoning failsafe",
    34: "black box",
    35: "DDS",
    100: "unknown",
}
# Disarms a person or a script commanded (their own choice, often an emergency stop).
ARM_METHODS_COMMANDED = {0, 1, 2, 4}
# Disarms that are normal on the ground (after landing) or are the crash check itself,
# which the firmware check already reports.
ARM_METHODS_GROUND_OR_CRASH = {8, 12, 13, 17, 22, 28, 30, 31, 32}
ARM_METHOD_AUXSWITCH = 2

# RCx_OPTION aux functions that arm and disarm (libraries/RC_Channel/RC_Channel.h, Copter-4.6.3):
# 153 ARMDISARM, 154 ARMDISARM_AIRMODE, 165 ARM_EMERGENCY_STOP.
ARM_DISARM_AUX = {153: "ArmDisarm", 154: "ArmDisarm with AirMode", 165: "Arm / Emergency Stop"}
# The stick channels (RCMAP_*; default 1-4) and what they fly.
_STICK_MAPS = (("RCMAP_ROLL", 1, "roll"), ("RCMAP_PITCH", 2, "pitch"), ("RCMAP_THROTTLE", 3, "throttle"),
               ("RCMAP_YAW", 4, "yaw"))


def stick_arm_switch(params: dict[str, float]) -> tuple[int, str, str] | None:
    """(channel, stick, function) when an arm/disarm aux function sits on a stick channel."""
    for name, default, stick in _STICK_MAPS:
        ch = int(params.get(name, default) or default)
        option = params.get(f"RC{ch}_OPTION")
        if option is not None and int(option) in ARM_DISARM_AUX:
            return ch, stick, ARM_DISARM_AUX[int(option)]
    return None


def mode_name(mode_num: int | str, vehicle_kind: str | None = None) -> str:
    """Decode a MODE number for the vehicle that logged it.

    Unknown kinds fall back to the Copter table (historical behaviour). Old logs
    that already carry the mode as a string are passed through unchanged.
    """
    if isinstance(mode_num, str):
        return mode_num.strip().upper() or "UNKNOWN"
    table = _MODES_BY_KIND.get(vehicle_kind or "", COPTER_MODES)
    return table.get(int(mode_num), f"MODE_{int(mode_num)}")


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


# A pack's peak voltage in a log (at arming, resting) per cell: a LiPo charged to 4.2 V
# (LiHV 4.35) and not yet below 3.5 V. Every cell count that puts the peak in this band
# is a candidate.
CELL_V_MIN = 3.5
CELL_V_MAX = 4.4
# MOT_BAT_VOLT_MAX / _MIN are set to 4.2 V and 3.3 V per cell for standard LiPos ("Setting
# up for tuning", ArduPilot Copter wiki); 4.35 V per cell covers a LiHV pack's MAX.
_HINT_PER_CELL = (("MOT_BAT_VOLT_MAX", (4.2, 4.35)), ("MOT_BAT_VOLT_MIN", (3.3,)))
# Design choice: with no hint, the candidate nearest 4.1 V per cell, a pack charged a little
# while before arming. On the 48 benchmark logs it agrees with MOT_BAT_VOLT_MAX wherever that
# parameter is consistent with the measured voltage.
CELL_V_TYPICAL = 4.1


def estimate_cells(max_voltage: float | None, params: dict[str, float] | None = None) -> int | None:
    """Estimate the series-cell count from the pack's peak voltage.

    Several cell counts can fit one voltage (47.6 V is 11S at 4.33 V or 12S at 3.97 V), so a
    parameter that states the pack (MOT_BAT_VOLT_MAX / _MIN) decides when it is consistent
    with the voltage; otherwise the candidate nearest CELL_V_TYPICAL per cell. Returns None
    when the voltage is implausible (no real monitor / odd scale) or fits no cell count.
    """
    if max_voltage is None or max_voltage < 4.0 or max_voltage > 70.0:
        return None
    candidates = [n for n in range(1, 17) if CELL_V_MIN <= max_voltage / n <= CELL_V_MAX]
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0]
    for name, per_cell in _HINT_PER_CELL:
        v = (params or {}).get(name)
        if not v or v <= 0:
            continue
        for pc in per_cell:
            n = round(float(v) / pc)
            if n in candidates and abs(float(v) / n - pc) <= 0.05:
                return n
    return min(candidates, key=lambda n: abs(max_voltage / n - CELL_V_TYPICAL))


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
