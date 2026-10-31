# SPDX-License-Identifier: AGPL-3.0-only
"""ESC power and temperature check (multirotor).

Two things the forum's crash threads keep finding and the other checks do not:

* **The motors lost their power while the autopilot still commanded them.** The battery
  current collapses to almost nothing in flight, the motor outputs (RCOU) still ask for
  thrust, the pack voltage holds (it even rises once the load is gone) and the log keeps
  running, so the battery and the flight controller stayed powered. Only the ESCs stopped
  drawing current: an ESC shut down (thermal or other protection, a desync it did not
  recover from) or the ESCs' power feed failed. A battery or FC power loss looks different:
  the voltage collapses or the log stops (the power and integrity checks).
* **ESC temperature** from ESC telemetry (``ESC.Temp``). BLHeli_S limits motor power once
  the MCU passes its thermal-protection threshold, which is programmable between 80 and
  140 degC, in four steps down to 0% (BLHeli_S manual Rev16.x, "Thermal protection");
  BLHeli_32 limits it from its programmable threshold down to 25% (BLHeli_32 manual
  Rev32.x). An ESC held above 80 degC may therefore have been limiting power, one above
  140 degC was beyond every threshold BLHeli_S allows.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import longest_run_above, make_finding

# --- power cut -----------------------------------------------------------------
# Design choice: the current before the cut is the median of this window...
BASELINE_WINDOW_S = 3.0
# ...and must show the motors were really drawing power (a few amps for the smallest craft).
MIN_FLIGHT_CURRENT_A = 3.0
# The cut: current at or below this share of the baseline, and below CUT_FLOOR_A...
CUT_SHARE = 0.15
CUT_FLOOR_A = 1.0
# ...for at least this long (a single low sample is a sensor glitch).
CUT_MIN_DURATION_S = 1.0
# The motors were commanded to spin: mean output in the cut window at least MOT_SPIN_MIN
# plus this share of the output range (MOT_SPIN_MIN default 0.15, AP_MotorsMulticopter).
COMMAND_MARGIN = 0.10
MOT_SPIN_MIN_DEFAULT = 0.15
# The battery stayed connected: the voltage in the cut window at least this share of the
# voltage before it (a disconnected pack falls to nothing, a loaded one only sags).
VOLTAGE_HELD_SHARE = 0.90

# --- temperature -----------------------------------------------------------------
# BLHeli_S thermal protection threshold range (BLHeli_S manual Rev16.x).
ESC_TEMP_PROTECT_MIN_C = 80.0
ESC_TEMP_PROTECT_MAX_C = 140.0
# Design choice: a temperature counts when it is held this long (telemetry can glitch).
TEMP_MIN_DURATION_S = 5.0


@register_check
class EscCheck(Check):
    id = "esc"
    title = "ESC power & temperature"
    category = "motors"
    requires: set[str] = set()  # BAT+RCOU for the power cut, ESC for temperature
    vehicles = {"copter"}
    description = (
        "Motors losing power while still commanded (battery current collapses, voltage holds, the log "
        "continues) and ESC temperatures against the ESC firmware's thermal-protection range."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        cut = self._power_cut(log)
        if cut is not None:
            findings.append(cut)
        findings.extend(self._temperatures(log))
        return findings

    # -- the motors lost power while commanded ---------------------------------

    def _power_cut(self, log: FlightLog) -> Finding | None:
        src = "BAT" if log.has("BAT") else ("CURR" if log.has("CURR") else None)
        if src is None or not log.has("RCOU"):
            return None
        insts = log.instances(src)
        times, (volt, curr) = log.series_many(src, ["Volt", "Curr"], insts[0] if insts else None)
        ok = np.isfinite(times) & np.isfinite(volt) & np.isfinite(curr)
        times, volt, curr = times[ok], volt[ok], curr[ok]
        if times.size < 10 or float(np.max(curr)) < MIN_FLIGHT_CURRENT_A:
            return None  # no current sensor, or nothing ever drew current
        armed = log.armed_mask(times)
        if armed is None:
            armed = np.ones(times.shape, dtype=bool)

        from .motors import MotorsCheck
        from .thrust_loss import pwm_range

        channels, _ = MotorsCheck._motor_channels(log)
        pmin, pmax = pwm_range(log)
        spin_min = float(log.params.get("MOT_SPIN_MIN", MOT_SPIN_MIN_DEFAULT))
        commanded_pwm = pmin + (spin_min + COMMAND_MARGIN) * (pmax - pmin)

        low = np.flatnonzero(armed & (curr <= CUT_FLOOR_A))
        checked_until = -np.inf
        for i in low:
            t = float(times[i])
            if t < checked_until:
                continue
            before = (times >= t - BASELINE_WINDOW_S) & (times < t) & armed
            if not np.any(before):
                continue
            base_i = float(np.median(curr[before]))
            if base_i < MIN_FLIGHT_CURRENT_A or float(curr[i]) > CUT_SHARE * base_i:
                continue
            # How long it stayed cut, armed throughout.
            j = i
            while j + 1 < times.size and armed[j + 1] and curr[j + 1] <= max(CUT_FLOOR_A, CUT_SHARE * base_i):
                j += 1
            t_end = float(times[j])
            checked_until = t_end
            if t_end - t < CUT_MIN_DURATION_S:
                continue
            win = (times >= t) & (times <= t_end)
            v_before, v_after = float(np.median(volt[before])), float(np.median(volt[win]))
            if v_before <= 0 or v_after < VOLTAGE_HELD_SHARE * v_before:
                continue  # the pack itself went away: the power check's territory
            outs = []
            for ch in channels:
                rt, rv = log.series("RCOU", ch)
                sel = (rt >= t) & (rt <= t_end)
                if np.any(sel):
                    outs.append(float(np.mean(rv[sel])))
            if not outs or float(np.mean(outs)) < commanded_pwm:
                continue  # the motors were not asked for thrust (landed, throttle cut)
            return self._cut_finding(
                log,
                src,
                t,
                t_end,
                base_i,
                float(np.median(curr[win])),
                v_before,
                v_after,
                outs,
                commanded_pwm,
            )
        return None

    def _cut_finding(
        self,
        log: FlightLog,
        src: str,
        t0: float,
        t1: float,
        base_i: float,
        cut_i: float,
        v_before: float,
        v_after: float,
        outs: list[float],
        commanded_pwm: float,
    ) -> Finding:
        temps = self._max_temps(log, t0)
        temp_txt = (
            f" ESC telemetry just before it read up to {max(temps.values()):.0f} degC "
            f"(BLHeli_S thermal protection starts limiting power at a threshold set between "
            f"{ESC_TEMP_PROTECT_MIN_C:.0f} and {ESC_TEMP_PROTECT_MAX_C:.0f} degC)."
            if temps
            else ""
        )
        samples: dict[str, float] = {
            "current_before_a": round(base_i, 1),
            "current_after_a": round(cut_i, 2),
            "volt_before": round(v_before, 2),
            "volt_after": round(v_after, 2),
            "mean_motor_output_pwm": round(float(np.mean(outs)), 0),
            "commanded_threshold_pwm": round(commanded_pwm, 0),
            "cut_duration_s": round(t1 - t0, 1),
        }
        if temps:
            samples["max_esc_temp_c"] = max(temps.values())
        return make_finding(
            self.id,
            Severity.CRITICAL,
            "Motors lost power while still commanded",
            f"At t={t0:.1f}s the battery current fell from {base_i:.1f} A to {cut_i:.1f} A and stayed there "
            f"for {t1 - t0:.1f} s while the motor outputs still asked for thrust (mean "
            f"{np.mean(outs):.0f} us). The pack voltage held ({v_before:.1f} V before, {v_after:.1f} V after) "
            "and the log kept running, so the battery and the flight controller stayed powered: the ESCs "
            "stopped driving the motors. That is an ESC shutting down (thermal or another protection, a "
            "desync it did not recover from) or the ESCs' power feed failing." + temp_txt,
            recommendation=(
                "Check the ESC power leads and connectors, the ESC protection settings and temperatures, "
                "and whether the ESCs are rated for the current this craft draws."
            ),
            time_start_s=t0,
            time_end_s=t1,
            message_types=[src, "RCOU"] + (["ESC"] if temps else []),
            samples=samples,
        )

    @staticmethod
    def _max_temps(log: FlightLog, before_t: float) -> dict[int, float]:
        """Highest ESC.Temp per ESC in the 30 s before ``before_t`` (0 = not reported)."""
        out: dict[int, float] = {}
        if not log.has("ESC") or "Temp" not in log.fields("ESC"):
            return out
        for inst in log.instances("ESC"):
            t, temp = log.series("ESC", "Temp", inst)
            sel = (t >= before_t - 30.0) & (t <= before_t) & np.isfinite(temp)
            if np.any(sel) and float(np.max(temp[sel])) > 0:
                out[inst] = float(np.max(temp[sel]))
        return out

    # -- ESC temperature ----------------------------------------------------------

    def _temperatures(self, log: FlightLog) -> list[Finding]:
        if not log.has("ESC") or "Temp" not in log.fields("ESC"):
            return []
        hot: list[tuple[int, float, float, float]] = []  # (esc, held max, over-80 s, over-140 s)
        start_hot: list[tuple[int, float]] = []
        t_first: float | None = None
        for inst in log.instances("ESC"):
            t, (temp, rpm) = log.series_many("ESC", ["Temp", "RPM"], inst)
            ok = np.isfinite(temp)
            if not np.any(ok):
                continue
            # A reading already at the protection range on the log's first sample, with the
            # motor stopped, is no temperature the flight produced: an ESC heats up under load.
            # Such a reading (one log reads 140-160 degC from power-on and never rises) is
            # left unjudged and named.
            first = int(np.argmax(ok))
            if temp[first] >= ESC_TEMP_PROTECT_MIN_C and not (np.isfinite(rpm[first]) and rpm[first] > 0):
                start_hot.append((inst, float(temp[first])))
                continue
            armed = log.armed_mask(t)
            if armed is not None:
                t, temp = t[armed], temp[armed]
            ok = np.isfinite(temp)
            t, temp = t[ok], temp[ok]
            if temp.size == 0:
                continue
            over_lo = longest_run_above(t, temp, ESC_TEMP_PROTECT_MIN_C - 1e-9)
            if over_lo < TEMP_MIN_DURATION_S:
                continue
            over_hi = longest_run_above(t, temp, ESC_TEMP_PROTECT_MAX_C - 1e-9)
            # The highest temperature held for TEMP_MIN_DURATION_S, not a one-sample spike.
            held = _held_max(t, temp, TEMP_MIN_DURATION_S)
            hot.append((inst, held, over_lo, over_hi))
            first = t[np.argmax(temp >= ESC_TEMP_PROTECT_MIN_C)]
            t_first = float(first) if t_first is None else min(t_first, float(first))
        out: list[Finding] = []
        if start_hot:
            listing = ", ".join(f"ESC{inst} {temp:.0f} degC" for inst, temp in sorted(start_hot))
            out.append(
                make_finding(
                    self.id,
                    Severity.INFO,
                    "ESC temperature telemetry not judged",
                    f"{len(start_hot)} ESC(s) already read {ESC_TEMP_PROTECT_MIN_C:.0f} degC or more on the "
                    f"log's first sample with the motor stopped ({listing}). An ESC heats up under load, so "
                    "this reading is probably not the ESC's temperature (or the log began right after a "
                    "flight); those ESCs are left out of the temperature check.",
                    message_types=["ESC"],
                    samples={"escs_not_judged": float(len(start_hot))},
                )
            )
        if not hot:
            return out
        hottest = max(hot, key=lambda h: h[1])
        beyond = [h for h in hot if h[3] >= TEMP_MIN_DURATION_S]
        sev = Severity.WARN if beyond else Severity.INFO
        listing = ", ".join(f"ESC{inst} {held:.0f} degC" for inst, held, _, _ in sorted(hot))
        msg = (
            f"While armed, {len(hot)} ESC(s) stayed at or above {ESC_TEMP_PROTECT_MIN_C:.0f} degC for "
            f"{TEMP_MIN_DURATION_S:.0f} s or more ({listing}). BLHeli_S starts limiting motor power at a "
            f"thermal-protection threshold set between {ESC_TEMP_PROTECT_MIN_C:.0f} and "
            f"{ESC_TEMP_PROTECT_MAX_C:.0f} degC, so these ESCs may have been limiting power"
        )
        msg += (
            f"; {len(beyond)} stayed above {ESC_TEMP_PROTECT_MAX_C:.0f} degC, beyond every threshold "
            "BLHeli_S allows."
            if beyond
            else "."
        )
        return out + [
            make_finding(
                self.id,
                sev,
                "Hot ESCs",
                msg,
                recommendation=(
                    "Check the ESCs' rating and cooling against the current drawn, and their thermal-"
                    "protection setting; a hot ESC that limits or cuts power drops the motor it drives."
                ),
                time_start_s=t_first,
                message_types=["ESC"],
                samples={
                    "hottest_esc": float(hottest[0]),
                    "hottest_held_temp_c": round(hottest[1], 0),
                    "escs_over_80c": float(len(hot)),
                    "escs_over_140c": float(len(beyond)),
                },
            )
        ]


def _held_max(times: np.ndarray, vals: np.ndarray, hold_s: float) -> float:
    """Highest level ``vals`` stayed at or above for ``hold_s`` (the max of rolling minima)."""
    best = float("-inf")
    j = 0
    for i in range(times.size):
        while times[i] - times[j] > hold_s:
            j += 1
        if times[i] - times[j] >= hold_s * 0.9:
            best = max(best, float(np.min(vals[j : i + 1])))
    return best if np.isfinite(best) else float(np.max(vals))
