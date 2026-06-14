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

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# Copter rate-controller P gain reference (firmware default ~0.135).
RATE_P_DEFAULT = 0.135
RATE_P_LOW = 0.04  # well below default -> unusually low
RATE_P_HIGH = 0.5  # above the usual valid range -> unusually high
_MULTIROTOR = {"copter", "heli", None}  # None = unknown kind, still evaluate


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
            out.extend(self._rate_gains(p))
        out.extend(self._contradictions(p, multirotor))
        return out

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
