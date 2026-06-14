"""Sensor presence & health check (wiring / connection / detection faults).

A sensor that is *configured* in the parameters but produces *no data* in the
log is a classic wiring/connection problem: the autopilot was told to expect the
device (a non-zero ``*_TYPE`` parameter) yet never logged a single message from
it, which usually means a loose cable, a wrong port, or the driver failing to
detect the unit. This check cross-references three independent sources and only
flags a sensor whose configuration and data clearly disagree:

* **Rangefinder** — ``RNGFND_TYPE`` / ``RNGFND1_TYPE`` > 0 declares a rangefinder
  on the first instance, but the log carries neither an ``RFND`` (modern) nor a
  ``RNGFND`` (legacy) message family. WARN.
* **GPS** — ``GPS_TYPE`` > 0 declares a GPS receiver, but the log carries no
  ``GPS`` message at all. WARN.
* **Compass health** — when ``MAG`` is logged it carries a per-sample ``Health``
  flag (1 = healthy, 0 = unhealthy). A healthy compass reads 1 almost always, so
  a large unhealthy fraction (> 20 %) indicates a failing/disconnected magnetometer.
  WARN.

This check always runs (``requires`` is empty) and guards every source
individually: a missing parameter or message simply skips that part, so the
check returns ``[]`` when nothing is both configured and silent. Being purely a
config/data cross-check it is deliberately conservative to avoid false positives.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# Parameters that declare a rangefinder on the first instance. >0 means "a
# rangefinder of this driver type is configured". Modern firmware uses
# RNGFND1_TYPE; older firmware used the un-numbered RNGFND_TYPE.
RNGFND_TYPE_PARAMS = ("RNGFND_TYPE", "RNGFND1_TYPE")
# Message families that prove a rangefinder actually logged data.
RNGFND_MESSAGES = ("RFND", "RNGFND")

# Parameter that declares a GPS receiver. >0 means GPS is configured.
GPS_TYPE_PARAM = "GPS_TYPE"
GPS_MESSAGE = "GPS"

# Compass: MAG.Health is 1 (healthy) / 0 (unhealthy) per sample. A healthy
# compass is ~always 1, so only a large unhealthy fraction is a real fault.
MAG_UNHEALTHY_FRACTION = 0.20


@register_check
class SensorsCheck(Check):
    id = "sensors"
    title = "Sensor presence & health"
    category = "sensors"
    requires: set[str] = set()  # always runs; each source is guarded inside run()
    description = "Configured-but-silent sensors and unhealthy sensors from params + RFND/GPS/MAG."

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        findings.extend(self._rangefinder_finding(log))
        findings.extend(self._gps_finding(log))
        findings.extend(self._compass_health_finding(log))
        return findings

    # -- Rangefinder: configured (TYPE>0) but no RFND/RNGFND data -------------

    def _rangefinder_finding(self, log: FlightLog) -> list[Finding]:
        rng_type = self._configured_type(log, RNGFND_TYPE_PARAMS)
        if rng_type is None:
            return []  # not configured -> nothing to check
        if any(log.has(m) for m in RNGFND_MESSAGES):
            return []  # configured and logged data -> healthy
        t = int(rng_type)
        return [
            make_finding(
                self.id,
                Severity.WARN,
                "Rangefinder configured but no data logged",
                f"A rangefinder is configured (type {t}) but the log contains neither an "
                "RFND nor a RNGFND message — the sensor produced no data. This is a classic "
                "wiring/connection/detection fault: check wiring, connection, and that the "
                "sensor is detected.",
                recommendation=(
                    "Check the rangefinder wiring and connector, confirm it is on the correct "
                    "port/bus, and verify the autopilot detects it (RNGFND1_TYPE matches the hardware)."
                ),
                message_types=list(RNGFND_MESSAGES),
                samples={"rngfnd_type": t},
            )
        ]

    # -- GPS: configured (GPS_TYPE>0) but no GPS data ------------------------

    def _gps_finding(self, log: FlightLog) -> list[Finding]:
        gps_type = log.params.get(GPS_TYPE_PARAM)
        if not (gps_type and gps_type > 0):
            return []  # not configured -> nothing to check
        if log.has(GPS_MESSAGE):
            return []  # configured and logged data -> healthy
        t = int(gps_type)
        return [
            make_finding(
                self.id,
                Severity.WARN,
                "GPS configured but no data logged",
                f"A GPS is configured (GPS_TYPE {t}) but no GPS data was logged — the receiver "
                "produced no data. Check the GPS connection.",
                recommendation=(
                    "Check the GPS cable and connector and confirm the receiver is wired to the "
                    "configured serial port; a silent GPS is usually a wiring/connection fault."
                ),
                message_types=[GPS_MESSAGE],
                samples={"gps_type": t},
            )
        ]

    # -- Compass health: MAG.Health unhealthy for too much of the flight -----

    def _compass_health_finding(self, log: FlightLog) -> list[Finding]:
        if not log.has("MAG"):
            return []
        times, health = log.series("MAG", "Health")
        if health.size == 0:
            return []
        unhealthy_frac = float(np.count_nonzero(health == 0)) / float(health.size)
        if unhealthy_frac <= MAG_UNHEALTHY_FRACTION:
            return []
        pct = unhealthy_frac * 100.0
        # Time window of the first/last unhealthy sample for context.
        bad_idx = np.nonzero(health == 0)[0]
        t0 = float(times[bad_idx[0]]) if times.size and bad_idx.size else None
        t1 = float(times[bad_idx[-1]]) if times.size and bad_idx.size else None
        return [
            make_finding(
                self.id,
                Severity.WARN,
                "Compass reported unhealthy",
                f"The compass (MAG.Health) reported unhealthy for {pct:.0f}% of the flight "
                f"({int(unhealthy_frac * health.size)} of {int(health.size)} samples). A healthy "
                "compass reads healthy almost always, so a sustained unhealthy reading points to a "
                "failing or disconnected magnetometer.",
                recommendation=(
                    "Inspect the compass wiring/connector (especially external compasses), check for "
                    "interference, and confirm the magnetometer is detected and calibrated."
                ),
                time_start_s=t0,
                time_end_s=t1,
                message_types=["MAG"],
                samples={
                    "unhealthy_fraction": round(unhealthy_frac, 3),
                    "mag_samples": int(health.size),
                },
            )
        ]

    # -- helpers -------------------------------------------------------------

    @staticmethod
    def _configured_type(log: FlightLog, param_names: tuple[str, ...]) -> float | None:
        """First positive ``*_TYPE`` value among ``param_names``, or None.

        A non-zero, present TYPE parameter means the device is configured. We
        return the type value (for evidence) and treat absent/zero as "not
        configured" so the check stays silent when nothing was declared.
        """
        for name in param_names:
            val = log.params.get(name)
            if val is not None and val > 0:
                return float(val)
        return None
