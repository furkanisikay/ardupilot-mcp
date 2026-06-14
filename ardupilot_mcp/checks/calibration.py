"""Sensor calibration check.

Inspects *parameters* (not time-series) to judge whether the magnetometers and
accelerometer were calibrated sensibly. This is a configuration/setup check, so
it always runs (``requires`` is empty) and guards each param family internally,
returning ``[]`` when none of the relevant params are present.

Compass offsets
---------------
For each compass instance the hard-iron offset vector COMPASS_OFS{n}_X/Y/Z is the
correction the calibration solved for. Its magnitude ``sqrt(x^2+y^2+z^2)`` (in
milligauss) is a direct quality signal:

* A well-sited, well-calibrated compass lands at roughly < 300 mGauss. Magnitudes
  above :data:`LARGE_OFFSET_MGAUSS` (600) mean either a poor calibration or strong
  onboard magnetic interference (power wiring / ferrous structure near the mag),
  and the EKF yaw estimate suffers — flagged WARN.
* A compass whose three offsets are *all exactly 0.0* was never calibrated. We
  only flag that for the **primary** compass (instance 1) and only when its
  device is actually present (``COMPASS_DEV_ID != 0``): secondary internal mags
  are routinely carried with default (zero) offsets and ``COMPASS_USE{n} == 1``
  by firmware default, so flagging them produces constant false positives.

A compass instance ``n`` is only considered when at least one of its OFS params
is present in the log.

Accelerometer (INFO only)
-------------------------
If INS_ACCSCAL_X/Y/Z are all exactly 1.0 and INS_ACCOFFS_X/Y/Z are all exactly
0.0, the accelerometer is sitting at factory defaults — it was never calibrated.
Informational, because the vehicle can still fly; just worth noting.
"""

from __future__ import annotations

import math

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# Hard-iron offset magnitude (mGauss) above which calibration is "large": poor
# calibration or strong onboard magnetic interference. A healthy compass is < ~300.
LARGE_OFFSET_MGAUSS = 600.0

# Accelerometer factory-default sentinels: scale exactly 1.0, offset exactly 0.0.
ACC_DEFAULT_SCALE = 1.0
ACC_DEFAULT_OFFSET = 0.0

# The three compass instances and the param suffix that distinguishes them.
# (instance number, OFS prefix, USE param, DEV_ID param)
_COMPASS_INSTANCES = (
    (1, "COMPASS_OFS_", "COMPASS_USE", "COMPASS_DEV_ID"),
    (2, "COMPASS_OFS2_", "COMPASS_USE2", "COMPASS_DEV_ID2"),
    (3, "COMPASS_OFS3_", "COMPASS_USE3", "COMPASS_DEV_ID3"),
)


@register_check
class CalibrationCheck(Check):
    id = "calibration"
    title = "Sensor calibration"
    category = "calibration"
    requires: set[str] = set()  # always runs; reads params, guarded internally
    description = "Compass offset magnitudes and accelerometer calibration from parameters."

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        findings.extend(self._compass_findings(log))
        findings.extend(self._accel_findings(log))
        return findings

    # -- compass offsets -----------------------------------------------------

    def _compass_findings(self, log: FlightLog) -> list[Finding]:
        out: list[Finding] = []
        params = log.params
        for n, ofs_prefix, use_key, devid_key in _COMPASS_INSTANCES:
            comps = [params.get(ofs_prefix + a) for a in ("X", "Y", "Z")]
            # Only consider this compass if at least one OFS param is present.
            if all(c is None for c in comps):
                continue
            # Treat missing axes as 0.0 so a partial set still yields a magnitude.
            ox, oy, oz = (float(c) if c is not None else 0.0 for c in comps)
            mag = math.sqrt(ox * ox + oy * oy + oz * oz)

            if mag > LARGE_OFFSET_MGAUSS:
                out.append(
                    make_finding(
                        self.id,
                        Severity.WARN,
                        f"Compass {n} offsets are large ({mag:.0f} mGauss)",
                        f"Compass {n} hard-iron offsets are large: magnitude {mag:.0f} mGauss "
                        f"({ofs_prefix}X/Y/Z = {ox:.0f}/{oy:.0f}/{oz:.0f}), above the "
                        f"{LARGE_OFFSET_MGAUSS:.0f} mGauss threshold. A healthy compass calibrates "
                        "to roughly under 300 mGauss; offsets this large indicate a poor calibration "
                        "or strong onboard magnetic interference, which degrades the EKF yaw estimate.",
                        recommendation="Re-run compass calibration and/or relocate the compass away from "
                        "power wiring and ferrous structure; an external compass usually helps.",
                        message_types=[],
                        samples={
                            f"compass{n}_offset_mag": round(mag, 1),
                            f"compass{n}_ofs_x": round(ox, 1),
                            f"compass{n}_ofs_y": round(oy, 1),
                            f"compass{n}_ofs_z": round(oz, 1),
                        },
                    )
                )
                continue

            # "Enabled but never calibrated": all three offsets exactly zero.
            # Restricted to the primary compass with a present device to avoid the
            # very common false positive of defaulted, unused secondary mags.
            if (
                n == 1
                and ox == 0.0
                and oy == 0.0
                and oz == 0.0
                and self._is_enabled(params, use_key)
                and self._device_present(params, devid_key)
            ):
                out.append(
                    make_finding(
                        self.id,
                        Severity.WARN,
                        f"Compass {n} enabled but never calibrated (offsets are zero)",
                        f"Compass {n} is enabled ({use_key} = 1) but its hard-iron offsets are all "
                        f"exactly zero ({ofs_prefix}X/Y/Z = 0/0/0), meaning it was never calibrated. "
                        "An uncalibrated compass that is in use feeds biased heading data to the EKF "
                        "and can cause yaw errors (toilet-bowling, fly-aways).",
                        recommendation="Run the compass calibration before flying; an enabled compass "
                        "should never have all-zero offsets.",
                        message_types=[],
                        samples={f"compass{n}_offset_mag": 0.0},
                    )
                )
        return out

    @staticmethod
    def _is_enabled(params: dict, use_key: str) -> bool:
        """A compass is enabled when USE==1 and (if present) COMPASS_ENABLE != 0."""
        use = params.get(use_key)
        if use is None or int(round(float(use))) != 1:
            return False
        enable = params.get("COMPASS_ENABLE")
        if enable is not None and int(round(float(enable))) == 0:
            return False
        return True

    @staticmethod
    def _device_present(params: dict, devid_key: str) -> bool:
        """A compass slot holds a real device when its DEV_ID is non-zero.

        When DEV_ID is absent we cannot tell, so default to ``True`` (present) so a
        log that simply doesn't record DEV_IDs still gets the primary check.
        """
        devid = params.get(devid_key)
        if devid is None:
            return True
        return float(devid) != 0.0

    # -- accelerometer -------------------------------------------------------

    def _accel_findings(self, log: FlightLog) -> list[Finding]:
        params = log.params
        scale = [params.get("INS_ACCSCAL_" + a) for a in ("X", "Y", "Z")]
        offset = [params.get("INS_ACCOFFS_" + a) for a in ("X", "Y", "Z")]
        # Only evaluate when all six params exist.
        if any(v is None for v in scale) or any(v is None for v in offset):
            return []
        scale_default = all(float(v) == ACC_DEFAULT_SCALE for v in scale)  # type: ignore[arg-type]
        offset_default = all(float(v) == ACC_DEFAULT_OFFSET for v in offset)  # type: ignore[arg-type]
        if not (scale_default and offset_default):
            return []
        return [
            make_finding(
                self.id,
                Severity.INFO,
                "Accelerometer appears uncalibrated (default scale/offset)",
                "The accelerometer is at factory defaults: INS_ACCSCAL_X/Y/Z are all exactly 1.0 and "
                "INS_ACCOFFS_X/Y/Z are all exactly 0.0, which means accelerometer calibration was never "
                "run. The vehicle can still fly, but an uncalibrated accelerometer can bias the level "
                "reference and the EKF.",
                recommendation="Run the accelerometer (3-axis level) calibration in the ground station.",
                message_types=[],
                samples={"acc_scale_x": 1.0, "acc_offset_x": 0.0},
            )
        ]
