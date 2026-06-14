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
from .util import make_finding, safe_max, safe_min

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
    requires = set()  # resolved dynamically in applicable() (BAT or CURR)
    description = (
        "Pack voltage vs failsafe thresholds, sudden brown-out drops, and sag under load from BAT/CURR."
    )

    def _source(self, log: FlightLog) -> str | None:
        """The battery message present in this log (BAT preferred, else CURR)."""
        for name in _BATTERY_SOURCES:
            if log.has(name):
                return name
        return None

    def applicable(self, log: FlightLog) -> tuple[bool, str | None]:
        if self._source(log) is None:
            return False, "no battery message (BAT/CURR) logged"
        return True, None

    def run(self, log: FlightLog) -> list[Finding]:
        src = self._source(log)
        if src is None:
            return []
        # Guard against logs with no real battery monitor / an odd voltage scale.
        _, volts = log.series(src, "Volt")
        if volts.size:
            med = float(np.median(volts))
            if med < PLAUSIBLE_MIN_V or med > PLAUSIBLE_MAX_V:
                return []
        findings: list[Finding] = []
        findings.extend(self._min_voltage_findings(log, src))
        findings.extend(self._sudden_drop_findings(log, src))
        findings.extend(self._sag_findings(log, src))
        return findings

    # -- minimum voltage vs configured failsafe thresholds -------------------

    def _min_voltage_findings(self, log: FlightLog, src: str) -> list[Finding]:
        times, volts = log.series(src, "Volt")
        if volts.size == 0:
            return []
        vmin = safe_min(volts)
        if vmin is None:
            return []
        # Time of the minimum-voltage sample (first occurrence).
        idx = int(np.argmin(volts))
        t_at_min = float(times[idx]) if idx < times.size else None

        crt, low, source = self._thresholds(log, volts)

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
                    time_start_s=t_at_min,
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
                    time_start_s=t_at_min,
                    time_end_s=t_at_min,
                    message_types=[src],
                    samples={"min_volt": round(vmin, 2), "low_volt": round(float(low), 2)},
                )
            ]
        return []

    def _thresholds(self, log: FlightLog, volts) -> tuple[float | None, float | None, str]:
        """Resolve (critical, low) voltage thresholds and a human description.

        Prefers the vehicle's configured BATT_CRT_VOLT / BATT_LOW_VOLT; when those
        aren't set, derives them from the estimated battery cell count so the check
        still works on logs without a configured failsafe.
        """
        crt = log.params.get("BATT_CRT_VOLT")
        low = log.params.get("BATT_LOW_VOLT")
        crt = crt if (crt and crt > 0.0) else None
        low = low if (low and low > 0.0) else None
        if crt is not None or low is not None:
            return crt, low, "configured BATT_*_VOLT failsafe"
        cells = meta.estimate_cells(safe_max(volts))
        if cells:
            return (
                cells * PER_CELL_CRIT_V,
                cells * PER_CELL_LOW_V,
                f"derived from an estimated {cells}S battery at {PER_CELL_CRIT_V}/{PER_CELL_LOW_V} V per cell",
            )
        return None, None, "no threshold available"

    # -- sudden fast voltage drop (connector dropout / brown-out) ------------

    def _sudden_drop_findings(self, log: FlightLog, src: str) -> list[Finding]:
        times, volts = log.series(src, "Volt")
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

    def _sag_findings(self, log: FlightLog, src: str) -> list[Finding]:
        # Need both voltage and current to talk about sag "under load".
        if not log.has(src):
            return []
        v_times, volts = log.series(src, "Volt")
        c_times, curr = log.series(src, "Curr")
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
