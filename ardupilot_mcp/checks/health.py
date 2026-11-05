# SPDX-License-Identifier: AGPL-3.0-only
"""Flight controller health: watchdog resets, a stalled main loop, internal errors.

The autopilot records its own software and processor failures:

* ``WDOG`` is written once a second, for the whole boot, when *this* boot followed a
  watchdog reset (libraries/AP_HAL_ChibiOS/Scheduler.cpp, ``was_watchdog_reset()``), so
  its presence means the previous boot ended with the processor being reset. The
  vehicle also sends a ``WDG: ...`` text (AP_Vehicle::send_watchdog_reset_statustext),
  the only trace in a telemetry log. ``FT`` is the fault type (1 Reset, 2 NMI, 3 Hard
  Fault, 4 MemManage, 5 BusFault, 6 UsageFault; hwdef/common/stm32_util.h). The wiki:
  "A 'HARD FAULT' is extremely serious ... The vehicle is probably not safe to fly until
  the cause is resolved" (Independent Watchdog and Crash Dump).
* ``MON`` is written while the main loop has been stuck for at least 200 ms (``Dly``,
  ms); at 500 ms, outside an expected delay, the firmware raises the ``main_loop_stuck``
  internal error (Scheduler.cpp, the monitor thread).
* ``PM.InE`` (``IntE`` before 4.5) is the internal-error bitmask; any bit set stops
  arming ("PreArm: Internal errors 0x.. l:.. <names>", AP_Arming::system_checks). Bit
  names from libraries/AP_InternalError/AP_InternalError.h.

Sources: Copter-4.6.3.
"""

from __future__ import annotations

import re

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# AP_InternalError::error_t bit -> name (AP_InternalError.h, Copter-4.6.3).
INTERNAL_ERRORS: dict[int, str] = {
    0: "logger_mapfailure",
    1: "logger_missing_logstructure",
    2: "logger_logwrite_missingfmt",
    3: "logger_too_many_deletions",
    4: "logger_bad_getfilename",
    5: "panic",
    6: "logger_flushing_without_sem",
    7: "logger_bad_current_block",
    8: "logger_blockcount_mismatch",
    9: "logger_dequeue_failure",
    10: "constraining_nan",
    11: "watchdog_reset",
    12: "iomcu_reset",
    13: "iomcu_fail",
    14: "spi_fail",
    15: "main_loop_stuck",
    16: "gcs_bad_missionprotocol_link",
    17: "bitmask_range",
    18: "gcs_offset",
    19: "i2c_isr",
    20: "flow_of_control",
    21: "switch_full_sector_recursion",
    22: "bad_rotation",
    23: "stack_overflow",
    24: "imu_reset",
    25: "gpio_isr",
    26: "mem_guard",
    27: "dma_fail",
    28: "params_restored",
    29: "invalid_arg_or_result",
}
WATCHDOG_BIT = 11

# FaultType (hwdef/common/stm32_util.h); 0 = no fault recorded (a plain hang).
FAULT_TYPES: dict[int, str] = {
    1: "Reset",
    2: "NMI",
    3: "Hard Fault",
    4: "MemManage fault",
    5: "Bus fault",
    6: "Usage fault",
}
# The wiki's "extremely serious" class: the processor faulted, not only stalled.
PROCESSOR_FAULTS = {2, 3, 4, 5, 6}

# Texts the firmware sends on the boot after a watchdog reset when, at the reset, the
# vehicle was armed (AP_Logger.cpp, was_watchdog_armed) or its safety switch was off
# (AP_BoardConfig/board_drivers.cpp, was_watchdog_safety_off).
ARMED_AT_RESET = "Forcing logging for watchdog reset"
SAFETY_OFF_AT_RESET = "Forcing safety off for watchdog"

# MON.Dly (ms): the firmware logs MON from 200 ms and declares main_loop_stuck at 500 ms.
MAIN_LOOP_STUCK_MS = 500.0

_WDG_TEXT = re.compile(r"WDG: T(-?\d+).*?\bFL(\d+) FT(\d+)")
_INTERNAL_TEXT = re.compile(r"Internal errors? \(?0x([0-9a-fA-F]+)\)?")


def health_owns(text: str) -> bool:
    """True for a MSG line this check reports (so the startup-message check skips it)."""
    t = text.strip()
    return bool(_WDG_TEXT.match(t)) or "Internal error" in t


@register_check
class HealthCheck(Check):
    id = "health"
    title = "Flight controller health"
    category = "integrity"
    requires: set[str] = set()  # WDOG, MON, PM and MSG are each guarded inside run()
    description = (
        "The autopilot's own record of a watchdog reset (WDOG / WDG:), a stalled main loop (MON) and "
        "internal errors (PM.InE, 'Internal errors 0x..')."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        texts: list[tuple[str, float | None]] = [
            (
                str(r["Message"]).strip(),
                float(ts) if isinstance(ts := r.get("timestamp"), (int, float)) else None,
            )
            for r in log.get("MSG")
            if isinstance(r.get("Message"), str)
        ]
        watchdog = self._watchdog(log, texts)
        out = [watchdog] if watchdog is not None else []
        out += self._main_loop(log)
        internal = self._internal_errors(log, texts, skip_watchdog=watchdog is not None)
        if internal is not None:
            out.append(internal)
        return out

    def _watchdog(self, log: FlightLog, texts: list[tuple[str, float | None]]) -> Finding | None:
        wdog = log.get("WDOG")
        wdg_texts = [(m, ts) for m, ts in texts if _WDG_TEXT.match(m)]
        if not wdog and not wdg_texts:
            return None
        task = line = ftype = None
        name = ""
        if wdog:
            rec = wdog[0]
            task, line, ftype = rec.get("Tsk"), rec.get("FL"), rec.get("FT")
            name = str(rec.get("TN") or "").strip("\x00 ")
        else:
            m = _WDG_TEXT.match(wdg_texts[0][0])
            if m:
                task, line, ftype = int(m.group(1)), int(m.group(2)), int(m.group(3))
        ft = int(ftype) if isinstance(ftype, (int, float)) else 0
        messages = {m for m, _ in texts}
        armed = any(m.startswith(ARMED_AT_RESET) for m in messages)
        safety_off = any(m.startswith(SAFETY_OFF_AT_RESET) for m in messages)
        fault = FAULT_TYPES.get(ft)
        state = (
            "The vehicle was armed when it reset (the firmware forced logging on for that reason)."
            if armed
            else "The safety switch was off when it reset (the firmware forced it off again)."
            if safety_off
            else "Nothing in this log says whether it was armed at the time."
        )
        what = (
            f"the processor hit a {fault} (FT={ft}) at source line {line}"
            if fault
            else f"the main loop hung until the watchdog reset the processor (no fault recorded, FT={ft})"
        )
        where = f" in scheduler task {task}" + (f", thread '{name}'" if name else "")
        severity = Severity.CRITICAL if ft in PROCESSOR_FAULTS or armed or safety_off else Severity.WARN
        # The reset happened before this log began; anchor the finding at the first trace of it
        # (the recovery texts come out at boot, before the first WDOG record).
        times = [float(r["timestamp"]) for r in wdog if isinstance(r.get("timestamp"), (int, float))]
        times += [
            float(ts)
            for m, ts in texts
            if isinstance(ts, (int, float)) and (_WDG_TEXT.match(m) or "watchdog" in m.lower())
        ]
        samples: dict[str, float] = {"fault_type": ft, "armed_at_reset": int(armed)}
        if isinstance(task, (int, float)):
            samples["scheduler_task"] = int(task)
        if isinstance(line, (int, float)):
            samples["fault_line"] = int(line)
        return make_finding(
            self.id,
            severity,
            "Flight controller was reset by its watchdog",
            f"This boot followed a watchdog reset: {what}{where}. The firmware writes WDOG only on the boot "
            f"after the processor was reset for hanging or faulting, so the previous flight or session ended "
            f"with the autopilot restarting. {state}"
            + (
                " The wiki calls a hard fault extremely serious: the vehicle is probably not safe to fly until the "
                "cause is resolved."
                if ft in PROCESSOR_FAULTS
                else ""
            ),
            recommendation=(
                "Report it on the ArduPilot forum with 'watchdog' in the title: attach this log, the log of the "
                "session that ended in the reset and the crash_dump.bin file from the @SYS folder, and give the "
                "firmware's git hash. Do not fly until the cause is resolved."
            ),
            time_start_s=min(times) if times else None,
            message_types=["WDOG"] if wdog else ["MSG"],
            samples=samples,
        )

    def _main_loop(self, log: FlightLog) -> list[Finding]:
        if not log.has("MON"):
            return []
        t, delay = log.series("MON", "Dly")
        ok = np.isfinite(t) & np.isfinite(delay)
        armed = log.armed_mask(t)
        if armed is not None:
            ok &= armed
        if not ok.any():
            return []
        t, delay = t[ok], delay[ok]
        worst = float(np.max(delay))
        stuck = worst >= MAIN_LOOP_STUCK_MS
        return [
            make_finding(
                self.id,
                Severity.WARN if stuck else Severity.INFO,
                "Main loop stalled while armed",
                f"While armed the main loop stopped for up to {worst:.0f} ms (MON.Dly, {t.size} record(s), first at "
                f"t={t[0]:.1f}s). The firmware logs MON once the loop has been stuck for 200 ms"
                + (
                    "; at 500 ms it declares the main_loop_stuck internal error. For that long the main loop, "
                    "which runs the attitude controllers, did not run."
                    if stuck
                    else "."
                ),
                recommendation=(
                    "MON.Tsk names the scheduler task that was running and MON.SmLn the semaphore it waited on; "
                    "report them with the log if the stall repeats."
                ),
                time_start_s=float(t[0]),
                time_end_s=float(t[-1]),
                message_types=["MON"],
                samples={"max_delay_ms": round(worst), "records": int(t.size)},
            )
        ]

    def _internal_errors(
        self, log: FlightLog, texts: list[tuple[str, float | None]], *, skip_watchdog: bool
    ) -> Finding | None:
        mask, first = 0, None
        for field in ("InE", "IntE"):
            if log.has("PM") and field in log.fields("PM"):
                t, vals = log.series("PM", field)
                ok = np.isfinite(vals) & (vals != 0)
                for v in vals[ok]:
                    mask |= int(v)
                if ok.any():
                    first = float(t[ok][0])
                break
        source = "PM.InE"
        if mask == 0:
            for m, ts in texts:
                hit = _INTERNAL_TEXT.search(m)
                if hit:
                    mask |= int(hit.group(1), 16)
                    if first is None and isinstance(ts, (int, float)):
                        first = float(ts)
            source = "the firmware's 'Internal errors' message"
        if skip_watchdog:
            mask &= ~(1 << WATCHDOG_BIT)  # already the watchdog finding
        if mask == 0:
            return None
        names = [INTERNAL_ERRORS.get(b, f"bit {b}") for b in range(32) if mask >> b & 1]
        return make_finding(
            self.id,
            Severity.WARN,
            "Autopilot raised internal errors",
            f"The internal-error mask was 0x{mask:x} ({', '.join(names)}), from {source}"
            + (f", first at t={first:.1f}s" if first is not None else "")
            + ". These are faults the firmware detected in itself (a bus, a driver, the scheduler, the logger), "
            "and the pre-arm checks refuse to arm while any is set.",
            recommendation=(
                "Look up each name in AP_InternalError.h; a reboot clears the mask, so find what raised it (a "
                "sensor on a failing bus, a recent firmware or peripheral change) before flying."
            ),
            time_start_s=first,
            message_types=["PM"] if source == "PM.InE" else ["MSG"],
            samples={"internal_error_mask": mask},
        )
