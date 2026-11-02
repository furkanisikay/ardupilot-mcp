# SPDX-License-Identifier: AGPL-3.0-only
"""Parameter audit check.

Deterministic, high-confidence checks for *misconfigured* parameters — values
that are self-contradictory, disabled-when-they-shouldn't-be, or implausible.
This complements (does not replace) the per-version parameter metadata: the
report links the firmware's ``apm.pdef.xml`` so the LLM can audit the long tail
of parameters against their documented defaults and ranges; this check nails the
small set of well-known, vehicle-independent misconfigurations that cause crashes.

Grounded in the real-log corpus (e.g. an oscillation crash that ran
``ATC_ACCEL_P_MAX=0`` with a rate P of 0.027), and deliberately conservative:
``MOT_PWM_MIN/MAX=0`` means "use RC limits" (not an error) and ``BATT_CRT_VOLT=0``
means "disabled" (not a contradiction), so those are not flagged.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# Multicopter rate-controller P gain reference (AC_ATC_MULTI_RATE_RP_P 0.135,
# AC_AttitudeControl_Multi.h). Not used for a helicopter: its default is 0.024 and its
# rate loop flies mainly on feed-forward (AC_ATC_HELI_RATE_RP_P 0.024, _FF 0.15,
# AC_AttitudeControl_Heli.h), so a low or zero P is normal there.
RATE_P_DEFAULT = 0.135
RATE_P_LOW = 0.04  # well below default -> unusually low
RATE_P_HIGH = 0.5  # above the usual valid range -> unusually high
_MULTIROTOR = {"copter", "heli", None}
# Design choice: a pack is outside MOT_BAT_VOLT_MAX/_MIN only when it is 2% beyond it (the
# 5th / 95th voltage percentile while armed), so sensor calibration error is not a finding.
VOLT_COMP_MARGIN = 0.02
# MOT_THST_EXPO default (AP_MOTORS_THST_EXPO_DEFAULT, AP_MotorsMulticopter.h).
THST_EXPO_DEFAULT = 0.65
# Design choice: a plane above AIRSPEED_MAX for this share of its flight flies outside its
# configured speed range. On the benchmark planes the share was 0.33-0.86 where the thread
# found the airspeed parameters wrong and at most 0.04 elsewhere.
AIRSPEED_OVER_MAX_FRACTION = 0.20
# EK3_SRC1_POSZ values (EKF sources wiki: 1 Baro, 2 RangeFinder, 3 GPS, 4 Beacon, 6 ExternalNav).
HEIGHT_SRC_RANGEFINDER = 2
HEIGHT_SRC_GPS = 3  # None = unknown kind, still evaluate


def _p(params: dict[str, float], name: str) -> float | None:
    v = params.get(name)
    return float(v) if v is not None else None


@register_check
class ParamAuditCheck(Check):
    id = "param_audit"
    title = "Parameter audit"
    category = "config"
    requires = set()
    description = "Flags self-contradictory, disabled or implausible parameter settings."

    def run(self, log: FlightLog) -> list[Finding]:
        p = log.params
        multirotor = log.meta.vehicle_kind in _MULTIROTOR
        out: list[Finding] = []
        if multirotor:
            out.extend(self._accel_limits(p))
            if log.meta.vehicle_kind != "heli":
                out.extend(self._rate_gains(p))
        out.extend(self._contradictions(p, multirotor))
        if log.meta.vehicle_kind in ("copter", None):
            out.extend(self._voltage_compensation(log))
        if log.meta.vehicle_kind == "plane":
            out.extend(self._airspeed_envelope(log))
        out.extend(self._height_source(p))
        return out

    def _height_source(self, p: dict[str, float]) -> list[Finding]:
        """EKF3 primary height source other than the barometer, as the EKF sources wiki weighs it."""
        if p.get("AHRS_EKF_TYPE", 3) != 3:
            return []
        src = p.get("EK3_SRC1_POSZ")
        if src == HEIGHT_SRC_RANGEFINDER:
            msg = (
                "EK3_SRC1_POSZ=2 makes the rangefinder the EKF's primary height source. The EKF sources page "
                "says this should almost never be used: it is only appropriate indoors over a flat floor with no "
                "clutter, and surface tracking and terrain following do not need it."
            )
        elif src == HEIGHT_SRC_GPS:
            msg = (
                "EK3_SRC1_POSZ=3 makes GPS the EKF's primary height source. The EKF sources page recommends it "
                "only for long flights where air pressure may change, with a high-quality GPS (e.g. a dual-band "
                "UBlox F9P); a GPS glitch then corrupts the height estimate directly. Read any GPS finding in "
                "this log with that in mind."
            )
        else:
            return []
        return [
            make_finding(
                self.id,
                Severity.INFO,
                "EKF height source is not the barometer",
                msg,
                recommendation="Unless the vehicle needs it, use the barometer (EK3_SRC1_POSZ=1), the default.",
                samples={"ek3_src1_posz": float(src)},
            )
        ]

    def _airspeed_envelope(self, log: FlightLog) -> list[Finding]:
        """Plane: an airspeed range that contradicts itself, or that the flight ran far above.

        AIRSPEED_MIN/MAX (ARSPD_FBW_MIN/MAX before 4.5) bound the speeds TECS flies, and the
        surfaces are scaled by SCALING_SPEED / airspeed, held between
        min(0.5, SCALING_SPEED / (2 AIRSPEED_MAX)) and max(2, SCALING_SPEED / (0.7 AIRSPEED_MIN))
        (Plane::calc_speed_scaler, ArduPlane/Attitude.cpp). Flying well above AIRSPEED_MAX means
        the parameters do not describe the airframe, and the surfaces got a fraction of the
        deflection they were tuned at.
        """
        p = log.params
        vmax = _p(p, "AIRSPEED_MAX") if "AIRSPEED_MAX" in p else _p(p, "ARSPD_FBW_MAX")
        vmin = _p(p, "AIRSPEED_MIN") if "AIRSPEED_MIN" in p else _p(p, "ARSPD_FBW_MIN")
        max_name = "AIRSPEED_MAX" if "AIRSPEED_MAX" in p else "ARSPD_FBW_MAX"
        min_name = "AIRSPEED_MIN" if "AIRSPEED_MIN" in p else "ARSPD_FBW_MIN"
        out: list[Finding] = []
        if vmax and vmin and vmax > 0 and vmin > 0 and vmax <= vmin:
            out.append(
                make_finding(
                    self.id,
                    Severity.WARN,
                    "Airspeed maximum not above the minimum",
                    f"{max_name}={vmax:.1f} m/s is at or below {min_name}={vmin:.1f} m/s, so the speed range the "
                    "autopilot flies (TECS, the speed scaling of the surfaces) is empty or inverted.",
                    recommendation=f"Set {min_name} a margin above the stall speed and {max_name} to the airframe's maximum.",
                    samples={max_name.lower(): round(vmax, 1), min_name.lower(): round(vmin, 1)},
                )
            )
            return out
        if not vmax or vmax <= 0:
            return out
        times, aspd = log.series("CTUN", "As")
        flying = log.flying_mask(times)
        if flying is not None:
            times, aspd = times[flying], aspd[flying]
        ok = np.isfinite(aspd)
        times, aspd = times[ok], aspd[ok]
        if aspd.size < 20:
            return out
        over = aspd > vmax
        frac = float(np.mean(over))
        if frac < AIRSPEED_OVER_MAX_FRACTION:
            return out
        scaling = _p(p, "SCALING_SPEED")
        scaler_txt = ""
        samples: dict[str, float] = {
            max_name.lower(): round(vmax, 1),
            "frac_above_max": round(frac, 2),
            "median_airspeed_above_max": round(float(np.median(aspd[over])), 1),
        }
        if scaling and scaling > 0:
            lo = min(0.5, scaling / (2.0 * vmax))
            hi = max(2.0, scaling / (0.7 * max(vmin or 0.0, 1.0)))
            scaler = float(np.median(np.clip(scaling / aspd[over], lo, hi)))
            scaler_txt = (
                f" The surfaces are scaled by SCALING_SPEED ({scaling:.1f} m/s) / airspeed, so while above "
                f"{max_name} they got a median {scaler:.2f}x of the deflection they were tuned at."
            )
            samples["median_speed_scaler"] = round(scaler, 2)
        return out + [
            make_finding(
                self.id,
                Severity.WARN,
                f"Flew far above {max_name}",
                f"In flight the airspeed estimate (CTUN.As) was above {max_name} ({vmax:.1f} m/s) for "
                f"{frac * 100:.0f}% of the time, at a median {samples['median_airspeed_above_max']:.1f} m/s there. "
                "The airspeed parameters do not describe this airframe's real speeds." + scaler_txt,
                recommendation=(
                    f"Set {min_name}, AIRSPEED_CRUISE and {max_name} to the airframe's measured speeds and "
                    "SCALING_SPEED to the cruise speed, then re-tune (changing SCALING_SPEED scales every PID)."
                ),
                time_start_s=float(times[over][0]),
                message_types=["CTUN"],
                samples=samples,
            )
        ]

    def _voltage_compensation(self, log: FlightLog) -> list[Finding]:
        """MOT_BAT_VOLT_MAX / _MIN that the pack's voltage never enters.

        The firmware clamps the pack's resting voltage into [MOT_BAT_VOLT_MIN,
        MOT_BAT_VOLT_MAX] (MIN raised to 60% of MAX) and scales the thrust gains by
        1 / lift_max from it (lift_max = r (1 - MOT_THST_EXPO) + MOT_THST_EXPO r^2, r = voltage / MAX);
        MAX <= 0, MIN >= MAX or a voltage below 0.25 x MIN disables it
        (Thrust_Linearization::update_lift_max_from_batt_voltage, AP_Motors). A pack that
        stays above MAX all flight gets no compensation; one that stays below MIN gets its
        gains raised by a constant factor all flight.
        """
        vmax, vmin = _p(log.params, "MOT_BAT_VOLT_MAX"), _p(log.params, "MOT_BAT_VOLT_MIN")
        if not vmax or vmax <= 0 or vmin is None or vmin >= vmax:
            return []
        eff_min = max(vmin, 0.6 * vmax)
        src = "BAT" if log.has("BAT") else ("CURR" if log.has("CURR") else None)
        if src is None:
            return []
        insts = log.instances(src)
        idx = int(log.params.get("MOT_BAT_IDX", 0))
        inst = idx if idx in insts else (insts[0] if insts else None)
        field = "VoltR" if "VoltR" in log.fields(src) else "Volt"
        times, volts = log.series(src, field, inst)
        armed = log.armed_mask(times)
        if armed is not None:
            volts = volts[armed]
        volts = volts[np.isfinite(volts) & (volts > 0)]
        if volts.size < 20:
            return []
        lo, hi = float(np.percentile(volts, 5)), float(np.percentile(volts, 95))
        if lo > vmax * (1 + VOLT_COMP_MARGIN):
            title = "Voltage compensation never active (MOT_BAT_VOLT_MAX below the pack)"
            msg = (
                f"While armed the pack's {'resting ' if field == 'VoltR' else ''}voltage stayed above "
                f"{lo:.1f} V ({src}.{field}, 5th percentile), but MOT_BAT_VOLT_MAX is {vmax:.1f} V. The firmware "
                "clamps the voltage to MOT_BAT_VOLT_MAX before scaling thrust, so the compensation did nothing "
                "for the whole flight: the controller's gains did not follow the pack as it discharged."
            )
            rec = "Set MOT_BAT_VOLT_MAX to 4.2 V x cells and MOT_BAT_VOLT_MIN to 3.3 V x cells for this pack."
        elif hi < eff_min * (1 - VOLT_COMP_MARGIN) and lo >= 0.25 * vmin:
            # Below 0.25 x MOT_BAT_VOLT_MIN the firmware turns compensation off instead.
            title = "Voltage compensation pinned at its minimum (MOT_BAT_VOLT_MIN above the pack)"
            # gain = 1 / lift_max, lift_max = r (1 - expo) + expo r^2, r = voltage / MAX
            # (Thrust_Linearization::update_lift_max_from_batt_voltage; MOT_THST_EXPO default 0.65).
            expo = min(1.0, max(-1.0, float(log.params.get("MOT_THST_EXPO", THST_EXPO_DEFAULT))))
            r = eff_min / vmax
            gain = 1.0 / (r * (1 - expo) + expo * r * r)
            msg = (
                f"While armed the pack's {'resting ' if field == 'VoltR' else ''}voltage stayed below "
                f"{hi:.1f} V ({src}.{field}, 95th percentile), but the compensation's floor is {eff_min:.1f} V "
                f"(MOT_BAT_VOLT_MIN, at least 60% of MOT_BAT_VOLT_MAX {vmax:.1f} V). The firmware clamps the "
                f"voltage there, so it raised the thrust gains by about {gain:.2f}x for the whole flight as if "
                "the pack were nearly empty, which makes the controller over-respond."
            )
            rec = "Set MOT_BAT_VOLT_MAX to 4.2 V x cells and MOT_BAT_VOLT_MIN to 3.3 V x cells for this pack."
        else:
            return []
        return [
            make_finding(
                self.id,
                Severity.WARN,
                title,
                msg,
                recommendation=rec,
                message_types=[src],
                samples={
                    "mot_bat_volt_max": round(vmax, 2),
                    "mot_bat_volt_min": round(vmin, 2),
                    "pack_volt_p5": round(lo, 2),
                    "pack_volt_p95": round(hi, 2),
                },
            )
        ]

    def _accel_limits(self, p: dict[str, float]) -> list[Finding]:
        out: list[Finding] = []
        for param, axis in (
            ("ATC_ACCEL_R_MAX", "roll"),
            ("ATC_ACCEL_P_MAX", "pitch"),
            ("ATC_ACCEL_Y_MAX", "yaw"),
        ):
            v = _p(p, param)
            if v is not None and v == 0.0:
                out.append(
                    make_finding(
                        self.id,
                        Severity.WARN,
                        f"{axis.capitalize()} acceleration limit disabled",
                        f"{param}=0 disables the {axis} angular-acceleration limit, so the attitude "
                        "controller can demand unlimited rate-of-change — a known cause of violent, "
                        "oscillation-prone response (a real oscillation crash was traced to exactly this).",
                        recommendation=f"Set {param} to a non-zero limit (defaults are ~110000 cdeg/s/s for roll/pitch).",
                        samples={param: 0.0},
                    )
                )
        return out

    def _rate_gains(self, p: dict[str, float]) -> list[Finding]:
        out: list[Finding] = []
        for param, axis in (("ATC_RAT_RLL_P", "roll"), ("ATC_RAT_PIT_P", "pitch")):
            v = _p(p, param)
            if v is None:
                continue
            if v == 0.0:
                out.append(
                    make_finding(
                        self.id,
                        Severity.WARN,
                        f"{axis.capitalize()} rate gain is zero",
                        f"{param}=0 means there is no {axis} rate-loop P term — the autopilot has no "
                        "control authority on this axis, which makes stable flight impossible.",
                        recommendation="Restore a sane rate P gain (copter default ~0.135) and re-tune.",
                        samples={param: 0.0},
                    )
                )
            elif v < RATE_P_LOW:
                out.append(
                    make_finding(
                        self.id,
                        Severity.INFO,
                        f"Unusually low {axis} rate gain",
                        f"{param}={v:.4f} is well below the ~{RATE_P_DEFAULT} default; very low rate gains give "
                        "sluggish, mushy control and can let the craft oscillate or fail to hold attitude.",
                        recommendation="Verify tuning (AutoTune or methodic tuning); confirm this low gain was intended.",
                        samples={param: round(v, 4), "default": RATE_P_DEFAULT},
                    )
                )
            elif v > RATE_P_HIGH:
                out.append(
                    make_finding(
                        self.id,
                        Severity.INFO,
                        f"Unusually high {axis} rate gain",
                        f"{param}={v:.4f} is above the usual range (default ~{RATE_P_DEFAULT}); excessive rate gain "
                        "is prone to high-frequency oscillation.",
                        recommendation="Re-check tuning; an overly high rate P often shows up as oscillation/hot motors.",
                        samples={param: round(v, 4), "default": RATE_P_DEFAULT},
                    )
                )
        return out

    def _contradictions(self, p: dict[str, float], multirotor: bool) -> list[Finding]:
        out: list[Finding] = []
        crt, low = _p(p, "BATT_CRT_VOLT"), _p(p, "BATT_LOW_VOLT")
        if crt and low and crt > 0 and low > 0 and crt >= low:
            out.append(
                make_finding(
                    self.id,
                    Severity.WARN,
                    "Battery critical voltage not below low voltage",
                    f"BATT_CRT_VOLT={crt:.2f} V is at or above BATT_LOW_VOLT={low:.2f} V. The critical failsafe "
                    "is meant to fire after the low-battery warning; set this way the staged response is defeated "
                    "(critical triggers at the same time as, or before, low).",
                    recommendation="Set BATT_CRT_VOLT below BATT_LOW_VOLT (e.g. ~0.2-0.3 V/cell lower).",
                    samples={"batt_crt_volt": round(crt, 2), "batt_low_volt": round(low, 2)},
                )
            )
        if multirotor:
            arm, mn = _p(p, "MOT_SPIN_ARM"), _p(p, "MOT_SPIN_MIN")
            if arm and mn and arm > 0 and mn > 0 and arm > mn:
                out.append(
                    make_finding(
                        self.id,
                        Severity.WARN,
                        "Motor spin-arm above spin-min",
                        f"MOT_SPIN_ARM={arm:.3f} is greater than MOT_SPIN_MIN={mn:.3f} — motors idle faster when "
                        "armed than the minimum flight throttle, which is backwards and hurts low-throttle control.",
                        recommendation="Set MOT_SPIN_ARM at or below MOT_SPIN_MIN.",
                        samples={"mot_spin_arm": round(arm, 3), "mot_spin_min": round(mn, 3)},
                    )
                )
            pmin, pmax = _p(p, "MOT_PWM_MIN"), _p(p, "MOT_PWM_MAX")
            if pmin and pmax and pmin > 0 and pmax > 0 and pmin >= pmax:
                out.append(
                    make_finding(
                        self.id,
                        Severity.WARN,
                        "Motor PWM range inverted",
                        f"MOT_PWM_MIN={pmin:.0f} >= MOT_PWM_MAX={pmax:.0f}: the motor output range is empty/inverted.",
                        recommendation="Set MOT_PWM_MIN below MOT_PWM_MAX (typically ~1000 and ~2000), or leave both 0 to use the RC range.",
                        samples={"mot_pwm_min": round(pmin, 0), "mot_pwm_max": round(pmax, 0)},
                    )
                )
        if _p(p, "INS_HNTCH_ENABLE") and (_p(p, "INS_HNTCH_FREQ") or 0.0) <= 0.0:
            out.append(
                make_finding(
                    self.id,
                    Severity.WARN,
                    "Harmonic notch enabled but unconfigured",
                    "INS_HNTCH_ENABLE=1 but INS_HNTCH_FREQ=0 — the harmonic notch filter is switched on but has no "
                    "centre frequency, so it does nothing to attenuate motor noise.",
                    recommendation="Set the notch centre frequency (see recommend_tuning / the harmonic-notch docs).",
                    samples={"ins_hntch_freq": 0.0},
                )
            )
        if _p(p, "FENCE_ENABLE"):
            amax = _p(p, "FENCE_ALT_MAX")
            rad = _p(p, "FENCE_RADIUS")
            if amax is not None and amax == 0.0 and (rad is None or rad == 0.0):
                out.append(
                    make_finding(
                        self.id,
                        Severity.WARN,
                        "Geofence enabled with no boundary",
                        "FENCE_ENABLE=1 but the altitude limit and radius are both 0 — the geofence has no actual "
                        "boundary configured.",
                        recommendation="Set FENCE_ALT_MAX and/or FENCE_RADIUS, or disable the fence.",
                        samples={"fence_alt_max": 0.0},
                    )
                )
        return out
