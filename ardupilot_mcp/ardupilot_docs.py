"""Curated ArduPilot documentation references attached to findings.

The server stays offline and deterministic; it does not fetch the web. Instead it
hands the LLM authoritative *pointers* (plus the log's firmware version) so the
LLM — which has its own web access — can read the right page for that version and
cite it. ``DOC_URLS`` holds verified canonical URLs; ``CHECK_TOPICS`` maps each
check to its relevant topics; ``references_for`` builds vehicle-aware references.
"""

from __future__ import annotations

import re

from .model import Reference

# Vehicle kind -> directory name in the versioned parameter archive.
_VERSIONED_VEHICLE = {
    "copter": "Copter",
    "heli": "Copter",
    "plane": "Plane",
    "rover": "Rover",
    "sub": "Sub",
    "tracker": "AntennaTracker",
}


def versioned_param_doc_url(firmware_version: str | None, vehicle_kind: str | None) -> str | None:
    """URL of the machine-readable parameter definitions for this exact firmware.

    ArduPilot archives per-version ``apm.pdef.xml`` (authoritative parameter names,
    ranges, defaults, bitmasks) for recent (4.x+) releases. Lets the LLM ground
    parameter advice to the log's firmware version instead of latest-stable.
    Returns None for pre-4.x firmware (not archived) or unknown version/vehicle.
    """
    if not firmware_version or not vehicle_kind:
        return None
    m = re.search(r"(\d+)\.(\d+)\.(\d+)", firmware_version)
    if not m or int(m.group(1)) < 4:
        return None
    veh = _VERSIONED_VEHICLE.get(vehicle_kind)
    if not veh:
        return None
    return f"https://autotest.ardupilot.org/Parameters/versioned/{veh}/stable-{m.group(0)}/apm.pdef.xml"


def version_reference(firmware_version: str | None, vehicle_kind: str | None) -> Reference | None:
    url = versioned_param_doc_url(firmware_version, vehicle_kind)
    if url is None:
        return None
    return Reference(title="Version-specific parameter definitions (apm.pdef.xml)", url=url)


# topic key -> (title, canonical URL). URLs verified against ardupilot.org.
DOC_URLS: dict[str, tuple[str, str]] = {
    "log_diagnosis_general": (
        "Diagnosing problems using logs",
        "https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html",
    ),
    "vibration": (
        "Measuring vibration",
        "https://ardupilot.org/copter/docs/common-measuring-vibration.html",
    ),
    "vibration_damping": (
        "Vibration damping",
        "https://ardupilot.org/copter/docs/common-vibration-damping.html",
    ),
    "ekf_failsafe": (
        "EKF failsafe",
        "https://ardupilot.org/copter/docs/ekf-inav-failsafe.html",
    ),
    "ekf_overview": (
        "Extended Kalman Filter (EKF)",
        "https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html",
    ),
    "gps_failsafe": (
        "GPS failsafe & glitch protection",
        "https://ardupilot.org/copter/docs/gps-failsafe-glitch-protection.html",
    ),
    "compass_calibration": (
        "Compass calibration",
        "https://ardupilot.org/copter/docs/common-compass-calibration-in-mission-planner.html",
    ),
    "compass_interference": (
        "Compass setup (advanced / interference)",
        "https://ardupilot.org/copter/docs/common-compass-setup-advanced.html",
    ),
    "battery_failsafe": (
        "Battery failsafe",
        "https://ardupilot.org/copter/docs/failsafe-battery.html",
    ),
    "power_monitor": (
        "Power module / battery monitor",
        "https://ardupilot.org/copter/docs/common-powermodule-landingpage.html",
    ),
    "radio_failsafe": (
        "Radio failsafe",
        "https://ardupilot.org/copter/docs/radio-failsafe.html",
    ),
    "tuning": (
        "Tuning",
        "https://ardupilot.org/copter/docs/common-tuning.html",
    ),
    "tuning_process": (
        "Tuning Process Instructions",
        "https://ardupilot.org/copter/docs/tuning-process-instructions.html",
    ),
    "autotune": (
        "AutoTune",
        "https://ardupilot.org/copter/docs/autotune.html",
    ),
    "harmonic_notch": (
        "Managing gyro noise with the harmonic notch",
        "https://ardupilot.org/copter/docs/common-imu-notch-filtering.html",
    ),
    "prearm_checks": (
        "Prearm safety checks",
        "https://ardupilot.org/copter/docs/common-prearm-safety-checks.html",
    ),
    "esc_calibration": (
        "ESC calibration",
        "https://ardupilot.org/copter/docs/esc-calibration.html",
    ),
    "thrust_loss": (
        "Thrust Loss and Yaw Imbalance Warnings",
        "https://ardupilot.org/copter/docs/thrust_loss_yaw_imbalance.html",
    ),
    "motor_order": (
        "Connect ESCs and Motors (motor order & direction)",
        "https://ardupilot.org/copter/docs/connect-escs-and-motors.html",
    ),
    "parameters_reference": (
        "Complete parameter list",
        "https://ardupilot.org/copter/docs/parameters.html",
    ),
}

# check id -> ordered topic keys (most specific first).
CHECK_TOPICS: dict[str, list[str]] = {
    "vibration": ["vibration", "vibration_damping"],
    "ekf": ["ekf_failsafe", "ekf_overview"],
    "gps": ["gps_failsafe"],
    "compass": ["compass_interference", "compass_calibration"],
    "power": ["battery_failsafe", "power_monitor"],
    "attitude": ["tuning"],
    "rcin": ["radio_failsafe"],
    "motors": ["thrust_loss", "motor_order", "esc_calibration"],
    "events": ["log_diagnosis_general"],
    "timing": ["log_diagnosis_general"],
    "integrity": ["log_diagnosis_general"],
    "config": ["prearm_checks", "parameters_reference"],
    "param_audit": ["parameters_reference", "tuning"],
    "calibration": ["compass_calibration", "compass_interference"],
    "sensors": ["prearm_checks"],
    "prearm": ["prearm_checks"],
}

# tuning area -> topic keys.
TUNING_TOPICS: dict[str, list[str]] = {
    "notch": ["harmonic_notch", "vibration"],
    "pid": ["tuning"],
    "autotune": ["autotune"],
}

# common-*.html pages exist under every vehicle path; swap the segment so a plane
# log links to the plane copy. Vehicle-specific (non-common) pages stay on copter.
_VEHICLE_PATH = {"copter": "copter", "heli": "copter", "plane": "plane", "rover": "rover", "sub": "sub"}


def _vehicle_url(url: str, vehicle_kind: str | None) -> str:
    if not vehicle_kind or "/common-" not in url:
        return url
    seg = _VEHICLE_PATH.get(vehicle_kind)
    if not seg or seg == "copter":
        return url
    return url.replace("/copter/", f"/{seg}/", 1)


def _refs(topics: list[str], vehicle_kind: str | None) -> list[Reference]:
    out: list[Reference] = []
    for key in topics:
        entry = DOC_URLS.get(key)
        if entry:
            title, url = entry
            out.append(Reference(title=title, url=_vehicle_url(url, vehicle_kind)))
    return out


def references_for(check_id: str, vehicle_kind: str | None = None) -> list[Reference]:
    """Documentation references for a check's findings."""
    return _refs(CHECK_TOPICS.get(check_id, []), vehicle_kind)


def tuning_references(area: str, vehicle_kind: str | None = None) -> list[Reference]:
    return _refs(TUNING_TOPICS.get(area, []), vehicle_kind)


def baseline_references(vehicle_kind: str | None = None) -> list[Reference]:
    """Always-useful references for the report as a whole."""
    return _refs(["log_diagnosis_general", "parameters_reference"], vehicle_kind)
