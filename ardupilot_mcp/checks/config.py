"""Configuration safety check.

Inspects ``log.params`` (the parameter snapshot stored in the DataFlash log) for
risky configuration that materially weakens flight safety. This check reads only
parameters, so it always runs (``requires`` is empty) and guards each parameter
individually, contributing whatever subset is present and returning ``[]`` when
none of the watched parameters are configured dangerously.

Watched parameters:

* ``ARMING_CHECK`` — the pre-arm safety bitmask. *Exactly* ``0`` disables every
  pre-arm check, so a bad sensor, failed calibration or missing GPS will not stop
  the operator arming. Any non-zero value (including negative bitmasks such as
  ``-9``) leaves checks enabled and is *not* flagged.
* ``BATT_MONITOR`` — ``0`` means no battery monitor at all (no voltage/current
  sensing, hence no low-battery failsafe).
* ``FS_THR_ENABLE`` — ``0`` disables the RC/throttle failsafe (Copter/Heli only;
  ArduPlane uses ``THR_FAILSAFE`` instead); the vehicle will not act on RC loss.
"""

from __future__ import annotations

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# Exact "disabled" sentinels we flag. Any other value (incl. negative bitmasks)
# means the feature is configured/enabled and must NOT be flagged.
ARMING_CHECK_DISABLED = 0.0
BATT_MONITOR_NONE = 0.0
FS_THR_DISABLED = 0.0

# FS_THR_ENABLE is a Copter/Heli parameter (Copter firmware); ArduPlane uses a
# separate parameter, THR_FAILSAFE, so we only evaluate FS_THR_ENABLE there.
# https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml
FS_THR_VEHICLES = {"copter", "heli"}


@register_check
class ConfigSafetyCheck(Check):
    id = "config"
    title = "Configuration safety"
    category = "config"
    requires: set[str] = set()  # reads params; always runs, guards each param inside run()
    description = "Risky safety configuration in parameters (ARMING_CHECK, BATT_MONITOR, FS_THR_ENABLE)."

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        findings.extend(self._arming_check(log))
        findings.extend(self._batt_monitor(log))
        findings.extend(self._fs_thr_enable(log))
        return findings

    def _arming_check(self, log: FlightLog) -> list[Finding]:
        val = log.params.get("ARMING_CHECK")
        if val is None:
            return []
        # Only EXACTLY 0 disables all checks. Negative values are bitmasks (enabled).
        if float(val) != ARMING_CHECK_DISABLED:
            return []
        return [
            make_finding(
                self.id,
                Severity.WARN,
                "Arming safety checks disabled",
                "ARMING_CHECK is set to 0, so the autopilot performs NO pre-arm checks. A bad "
                "sensor, a failed calibration or a missing GPS fix will not stop the vehicle from "
                "arming. Operators often set ARMING_CHECK=0 to force past a real problem, which "
                "hides the underlying fault rather than fixing it.",
                recommendation=(
                    "Re-enable pre-arm checks (ARMING_CHECK=1 for all checks, or a bitmask) and "
                    "resolve whatever was failing instead of bypassing the checks."
                ),
                message_types=["PARM"],
                samples={"ARMING_CHECK": 0},
            )
        ]

    def _batt_monitor(self, log: FlightLog) -> list[Finding]:
        val = log.params.get("BATT_MONITOR")
        if val is None:
            return []
        if float(val) != BATT_MONITOR_NONE:
            return []
        return [
            make_finding(
                self.id,
                Severity.INFO,
                "No battery monitor configured",
                "BATT_MONITOR is set to 0, so no battery monitor is configured. The autopilot has "
                "no voltage or current sensing and therefore cannot run a low-battery failsafe, so "
                "an over-discharged or failing pack will not be detected in flight.",
                recommendation=(
                    "Configure BATT_MONITOR to match the installed power module/sensor so voltage "
                    "monitoring and the battery failsafe can protect the flight."
                ),
                message_types=["PARM"],
                samples={"BATT_MONITOR": 0},
            )
        ]

    def _fs_thr_enable(self, log: FlightLog) -> list[Finding]:
        # Vehicle-specific: only RC-piloted aircraft. A known kind outside the set
        # is skipped; an unknown/None kind still runs (avoids false negatives).
        kind = log.meta.vehicle_kind
        if kind is not None and kind not in FS_THR_VEHICLES:
            return []
        val = log.params.get("FS_THR_ENABLE")
        if val is None:
            return []
        if float(val) != FS_THR_DISABLED:
            return []
        return [
            make_finding(
                self.id,
                Severity.INFO,
                "RC/throttle failsafe disabled",
                "FS_THR_ENABLE is set to 0, so the RC/throttle failsafe is disabled. If the RC link "
                "is lost the vehicle will not take any failsafe action (RTL/Land/etc.) and may "
                "continue on its last command with no pilot control.",
                recommendation=(
                    "Enable the throttle failsafe (FS_THR_ENABLE) so the vehicle reacts safely to "
                    "loss of the RC link."
                ),
                message_types=["PARM"],
                samples={"FS_THR_ENABLE": 0},
            )
        ]
