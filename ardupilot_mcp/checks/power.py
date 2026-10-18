# SPDX-License-Identifier: AGPL-3.0-only
"""Battery & power check.

Reads the battery message: pack voltage (``Volt``, V) and current draw
(``Curr``, A). Newer firmware logs this as ``BAT``; older firmware (Copter 3.x)
logs it as ``CURR`` with the same ``Volt``/``Curr`` fields — this check accepts
either so it works across firmware generations. ArduPilot protects the battery
with two pack-level failsafe thresholds stored as parameters:
``BATT_LOW_VOLT`` (low-battery action) and ``BATT_CRT_VOLT`` (critical/land-now
action). This check compares the measured minimum voltage against those
thresholds, looks for a sudden fast voltage collapse (a connector dropout or
brown-out signature), and reports voltage sag under heavy current as an
informational note.

All conclusions cite the concrete numbers (vmin, configured thresholds, drop
magnitude, dt) so a human can verify them against the log.
"""

from __future__ import annotations

import numpy as np

from .. import ardupilot_meta as meta
from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import intervals_above, make_finding, safe_max, safe_min

# Per-cell voltage thresholds used to derive failsafe levels when the vehicle's
# BATT_*_VOLT parameters aren't set (LiPo: ~3.5 V/cell low, ~3.3 V/cell critical).
PER_CELL_LOW_V = 3.5
PER_CELL_CRIT_V = 3.3

# Battery message names across firmware generations, in preference order.
_BATTERY_SOURCES = ("BAT", "CURR")

# Sudden-drop detector: a voltage decrease larger than this, between two
# consecutive samples spaced less than DROP_MAX_DT_S apart, looks like a
# connector dropout / brown-out rather than ordinary discharge sag.
DROP_WARN_V = 2.0  # volts lost in one fast step -> WARN ("sudden voltage drop")
DROP_CRIT_V = 4.0  # an even larger fast collapse -> CRITICAL
DROP_MAX_DT_S = 0.5  # only consider deltas across gaps shorter than this (s)

# Current (A) at or above which we treat the voltage span as sag "under load".
HIGH_CURRENT_A = 20.0
# Only surface a sag INFO if the spread under load is at least this many volts.
SAG_MIN_V = 1.0

# Flight-controller 5 V rail (POWR.Vcc). ArduPilot's LogAnalyzer TestVCC flagged a
# minimum below 4.6 V (FAIL) and a spread above 0.3 V (WARN); the wiki says 0.10-0.15 V
# variation is normal. Evaluated over the armed window only (USB/bench power before
# arming is irrelevant).
# https://github.com/ArduPilot/ardupilot/blob/Copter-4.0.7/Tools/LogAnalyzer/tests/TestVCC.py
# https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html
VCC_MIN_V = 4.6
VCC_MAX_SPREAD_V = 0.3
# Design choice: a median outside this is not a 5 V rail reading, so skip.
VCC_PLAUSIBLE_MIN_V = 3.0
VCC_PLAUSIBLE_MAX_V = 6.0

# Plausible pack-voltage range (V). Outside this the log has no real battery
# monitor or an odd scale (e.g. raw ADC reading thousands), and any "drop" is
# noise — skip power analysis rather than emit nonsense.
PLAUSIBLE_MIN_V = 4.0
PLAUSIBLE_MAX_V = 70.0


@register_check
class PowerCheck(Check):
    id = "power"
    title = "Battery & power"
    category = "power"
    requires = set()  # resolved dynamically in applicable() (BAT/CURR and/or POWR)
    description = (
        "Per-battery voltage vs failsafe thresholds, sudden brown-out drops and sag under load (BAT/CURR), "
        "plus flight-controller board voltage (POWR.Vcc)."
    )

    def _source(self, log: FlightLog) -> str | None:
        """The battery message present in this log (BAT preferred, else CURR)."""
        for name in _BATTERY_SOURCES:
            if log.has(name):
                return name
        return None

    def applicable(self, log: FlightLog) -> tuple[bool, str | None]:
        if self._source(log) is None and not log.has("POWR"):
            return False, "no battery (BAT/CURR) or board power (POWR) message logged"
        return True, None

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        src = self._source(log)
        if src is not None:
            # Every battery monitor is its own pack with its own failsafe params
            # (BATT_* for the first, BATT2_* for the second, ...): judging a 2S
            # payload battery against the 4S main pack's thresholds is meaningless.
            insts = log.instances(src)
            for inst in insts:
                # Guard against monitors with no real sensor / an odd voltage scale.
                _, volts = log.series(src, "Volt", inst)
                if volts.size:
                    med = float(np.median(volts))
                    if med < PLAUSIBLE_MIN_V or med > PLAUSIBLE_MAX_V:
                        continue
                for f in (
                    self._min_voltage_findings(log, src, inst)
                    + self._sudden_drop_findings(log, src, inst)
                    + self._sag_findings(log, src, inst)
                ):
                    if len(insts) > 1:
                        f.title += f" (battery {inst + 1})"
                        f.evidence.samples["battery_instance"] = inst
                    findings.append(f)
        findings.extend(self._vcc_findings(log))
        return findings

    # -- flight-controller board voltage (POWR.Vcc) ---------------------------

    def _vcc_findings(self, log: FlightLog) -> list[Finding]:
        times, vcc = log.series("POWR", "Vcc")
        armed = log.armed_mask(times)
        if armed is not None:
            times, vcc = times[armed], vcc[armed]
        if vcc.size < 2:
            return []
        med = float(np.median(vcc))
        if not VCC_PLAUSIBLE_MIN_V <= med <= VCC_PLAUSIBLE_MAX_V:
            return []  # not a 5 V rail reading (unpowered sensor / other scale)
        vmin, vmax = float(np.min(vcc)), float(np.max(vcc))
        spread = vmax - vmin
        t_min = float(times[int(np.argmin(vcc))])
        samples = {"min_vcc": round(vmin, 3), "max_vcc": round(vmax, 3), "spread_v": round(spread, 3)}
        if vmin < VCC_MIN_V:
            return [
                make_finding(
                    self.id,
                    Severity.CRITICAL,
                    "Flight-controller board voltage (Vcc) brown-out",
                    f"The flight controller's board supply (POWR.Vcc) fell to {vmin:.2f} V while armed, "
                    f"below the {VCC_MIN_V:.1f} V minimum. A 5 V rail this low can brown out or reset the "
                    "autopilot and its sensors mid-flight.",
                    recommendation=(
                        "Check the power module / BEC feeding the autopilot and the servo rail; avoid powering "
                        "servos or peripherals from the same regulator."
                    ),
                    time_start_s=t_min,
                    time_end_s=t_min,
                    message_types=["POWR"],
                    samples={**samples, "min_threshold_v": VCC_MIN_V},
                )
            ]
        if spread > VCC_MAX_SPREAD_V:
            return [
                make_finding(
                    self.id,
                    Severity.WARN,
                    "Flight-controller board voltage (Vcc) unstable",
                    f"The board supply (POWR.Vcc) varied by {spread:.2f} V while armed ({vmin:.2f}-{vmax:.2f} V), "
                    f"more than {VCC_MAX_SPREAD_V:.1f} V. Variations of 0.10-0.15 V are normal; larger ripple "
                    "suggests other devices sharing the autopilot's supply are dragging it down.",
                    recommendation="Give the autopilot a dedicated, adequately rated 5 V supply (power module/BEC).",
                    time_start_s=float(times[0]),
                    time_end_s=float(times[-1]),
                    message_types=["POWR"],
                    samples={**samples, "spread_threshold_v": VCC_MAX_SPREAD_V},
                )
            ]
        return []

    # -- minimum voltage vs configured failsafe thresholds -------------------

    def _min_voltage_findings(self, log: FlightLog, src: str, inst: int = 0) -> list[Finding]:
        times, volts = log.series(src, "Volt", inst)
        if volts.size == 0:
            return []
        vmin = safe_min(volts)
        if vmin is None:
            return []
        # Time of the minimum-voltage sample (first occurrence).
        idx = int(np.argmin(volts))
        t_at_min = float(times[idx]) if idx < times.size else None

        crt, low, source = self._thresholds(log, volts, inst)

        timer = float(log.params.get("BATT_LOW_TIMER" if inst == 0 else f"BATT{inst + 1}_LOW_TIMER", 10.0))

        def first_at_or_below(level: float) -> float | None:
            """Start of the run below ``level`` that the failsafe timer counts.

            AP_BattMonitor fires only when the voltage stays below the threshold continuously
            for BATT_LOW_TIMER seconds, so a momentary sag under load does not start it: the
            first run that lasts that long, else the run holding the minimum.
            """
            runs = intervals_above(times, -volts, -level - 1e-9)
            if not runs:
                return t_at_min
            held = next((r for r in runs if r.duration_s >= timer), None)
            if held is None:
                held = next(
                    (r for r in runs if t_at_min is not None and r.start_s <= t_at_min <= r.end_s), runs[-1]
                )
            return held.start_s

        # Critical takes precedence over low.
        if crt is not None and vmin <= crt:
            return [
                make_finding(
                    self.id,
                    Severity.CRITICAL,
                    "Battery reached critical voltage",
                    f"Pack voltage fell to {vmin:.2f} V, at or below the critical-voltage level "
                    f"({crt:.2f} V, {source}). At this point ArduPilot's critical battery action "
                    "(typically land/RTL) is warranted; flying below it risks an in-air power loss.",
                    recommendation="Land/recover immediately on this threshold; recharge and inspect the pack before flying again.",
                    # From the first sample at the threshold (what the failsafe timer counts
                    # from) to the lowest voltage.
                    time_start_s=first_at_or_below(float(crt)),
                    time_end_s=t_at_min,
                    message_types=[src],
                    samples={"min_volt": round(vmin, 2), "crit_volt": round(float(crt), 2)},
                )
            ]
        if low is not None and vmin <= low:
            return [
                make_finding(
                    self.id,
                    Severity.WARN,
                    "Battery reached low-voltage threshold",
                    f"Pack voltage fell to {vmin:.2f} V, at or below the low-voltage level "
                    f"({low:.2f} V, {source}). This is the low-battery warning point; the pack was "
                    "nearing depletion.",
                    recommendation="Shorten flights or land sooner; verify pack capacity/health and the BATT_LOW_VOLT setting.",
                    time_start_s=first_at_or_below(float(low)),
                    time_end_s=t_at_min,
                    message_types=[src],
                    samples={"min_volt": round(vmin, 2), "low_volt": round(float(low), 2)},
                )
            ]
        return []

    def _thresholds(self, log: FlightLog, volts, inst: int = 0) -> tuple[float | None, float | None, str]:
        """Resolve (critical, low) voltage thresholds and a human description.

        Prefers the vehicle's configured BATT_CRT_VOLT / BATT_LOW_VOLT; when those
        aren't set, derives them from the estimated battery cell count so the check
        still works on logs without a configured failsafe.
        """
        prefix = "BATT_" if inst == 0 else f"BATT{inst + 1}_"
        crt = log.params.get(prefix + "CRT_VOLT")
        low = log.params.get(prefix + "LOW_VOLT")
        crt = crt if (crt and crt > 0.0) else None
        low = low if (low and low > 0.0) else None
        if crt is not None or low is not None:
            return crt, low, f"configured {prefix}*_VOLT failsafe"
        cells = meta.estimate_cells(safe_max(volts))
        if cells:
            return (
                cells * PER_CELL_CRIT_V,
                cells * PER_CELL_LOW_V,
                f"derived from an estimated {cells}S battery at {PER_CELL_CRIT_V}/{PER_CELL_LOW_V} V per cell",
            )
        return None, None, "no threshold available"

    # -- sudden fast voltage drop (connector dropout / brown-out) ------------

    def _sudden_drop_findings(self, log: FlightLog, src: str, inst: int = 0) -> list[Finding]:
        times, volts = log.series(src, "Volt", inst)
        if volts.size < 2:
            return []
        # Largest fast drop: most-negative delta over a short dt.
        worst_drop = 0.0  # magnitude (volts lost), positive number
        worst_i = -1
        worst_t0 = None
        worst_t1 = None
        worst_dt = None
        for i in range(1, volts.size):
            dt = float(times[i] - times[i - 1])
            if dt <= 0.0 or dt >= DROP_MAX_DT_S:
                continue
            drop = float(volts[i - 1] - volts[i])  # >0 means voltage fell
            if drop > worst_drop:
                worst_drop = drop
                worst_i = i
                worst_t0 = float(times[i - 1])
                worst_t1 = float(times[i])
                worst_dt = dt

        if worst_drop <= DROP_WARN_V or worst_i < 1:
            return []

        # Require the drop to PERSIST: a real brown-out stays depressed, whereas a
        # single-sample sensor glitch snaps back. If voltage recovers to near the
        # pre-drop level within ~1 s, treat it as noise, not a power event.
        assert worst_t1 is not None  # guaranteed once worst_i >= 1 (set together)
        pre_drop = float(volts[worst_i - 1])
        post_mask = (times > worst_t1) & (times <= worst_t1 + 1.0)
        if np.any(post_mask):
            if float(np.max(volts[post_mask])) >= pre_drop - worst_drop / 2.0:
                return []

        sev = Severity.CRITICAL if worst_drop > DROP_CRIT_V else Severity.WARN
        dt_txt = f"{worst_dt * 1000:.0f} ms" if worst_dt is not None else "one sample"
        return [
            make_finding(
                self.id,
                sev,
                "Sudden voltage drop",
                f"Pack voltage dropped {worst_drop:.2f} V in {dt_txt} (a step larger than the "
                f"{DROP_WARN_V:.1f} V/{DROP_MAX_DT_S:.1f} s threshold). A collapse this fast is not "
                "normal discharge sag; it points to a loose battery connector, a failing cell, or a "
                "momentary brown-out of the power system.",
                recommendation="Inspect the battery connector/leads and pack health; a brown-out can reset peripherals or the flight controller.",
                time_start_s=worst_t0,
                time_end_s=worst_t1,
                message_types=[src],
                samples={
                    "drop_volt": round(worst_drop, 2),
                    "drop_dt_s": round(float(worst_dt), 4) if worst_dt is not None else 0.0,
                    "drop_threshold_volt": DROP_WARN_V,
                },
            )
        ]

    # -- voltage sag under load (informational) ------------------------------

    def _sag_findings(self, log: FlightLog, src: str, inst: int = 0) -> list[Finding]:
        # Need both voltage and current to talk about sag "under load".
        if not log.has(src):
            return []
        v_times, volts = log.series(src, "Volt", inst)
        c_times, curr = log.series(src, "Curr", inst)
        if volts.size == 0 or curr.size == 0:
            return []
        # Volt and Curr come from the same BAT records, so align by the shorter
        # length (both are produced in record order).
        n = min(volts.size, curr.size)
        if n == 0:
            return []
        v = volts[:n]
        c = curr[:n]
        t = v_times[:n] if v_times.size >= n else v_times

        high = c >= HIGH_CURRENT_A
        if not bool(np.any(high)):
            return []
        v_under_load = v[high]
        vmax_load = safe_max(v_under_load)
        vmin_load = safe_min(v_under_load)
        if vmax_load is None or vmin_load is None:
            return []
        sag = float(vmax_load - vmin_load)
        if sag < SAG_MIN_V:
            return []

        peak_current = safe_max(c) or 0.0
        # Time window: from first to last high-current sample.
        hi_idx = np.nonzero(high)[0]
        t0 = float(t[hi_idx[0]]) if hi_idx.size and hi_idx[0] < t.size else None
        t1 = float(t[hi_idx[-1]]) if hi_idx.size and hi_idx[-1] < t.size else None
        return [
            make_finding(
                self.id,
                Severity.INFO,
                "Voltage sag under load",
                f"Under high current (>= {HIGH_CURRENT_A:.0f} A, peak {peak_current:.1f} A) the pack "
                f"voltage spanned {sag:.2f} V (from {vmax_load:.2f} V down to {vmin_load:.2f} V). Some "
                "sag under load is expected; a large spread can indicate an aging pack or high internal "
                "resistance worth watching.",
                recommendation=None,
                time_start_s=t0,
                time_end_s=t1,
                message_types=[src],
                samples={
                    "sag_volt": round(sag, 2),
                    "peak_current_a": round(float(peak_current), 1),
                    "high_current_a": HIGH_CURRENT_A,
                },
            )
        ]
