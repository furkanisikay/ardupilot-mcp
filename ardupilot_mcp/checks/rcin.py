# SPDX-License-Identifier: AGPL-3.0-only
"""RC input check.

Reads ``RCIN`` (the RC channels the autopilot *received*). The clearest failure
signature is loss of the RC link: the receiver drops or holds all channels at an
abnormally low/failsafe value (~900-1000 us) at the same time. In normal flight
the sticks are independent — roll/pitch/yaw sit near centre (~1500 us) while only
throttle is low — so *all* primary channels being low together does not happen
unless the link failed. (This catches the classic "RC stuck at ~990 us, failsafe
didn't trigger" crash.)

Most receivers do NOT output low values on link loss — ArduPilot logs
``rc().get_radio_in()``, which holds the last good values — so the PWM heuristic
alone misses most real losses. Firmware 4.x also logs ``RCI2.Flags`` right after
``RCIN``: bit 0 HAS_VALID_INPUT ("receiving good RC values") and bit 1
IN_RC_FAILSAFE (Copter-4.6.3 libraries/AP_Logger/AP_Logger.h RCLoggingFlags,
LogFile.cpp Write_RCIN). When RCI2 is present it is the authoritative signal; the
PWM heuristic is the fallback for older logs. Only armed time is judged: a
receiver with no transmitter on the bench before arming is not a link loss.
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import intervals_above, make_finding

# Below this PWM a primary channel is outside its normal range (min stick ~1000).
NO_SIGNAL_US = 1000.0
# All primary channels under NO_SIGNAL_US for at least this long = link loss.
SUSTAINED_S = 0.5
# A channel pinned at/near zero is a hard no-pulse.
DEAD_US = 900.0

_PRIMARY = ("C1", "C2", "C3", "C4")  # roll, pitch, throttle, yaw

# RCI2.Flags bits (AP_Logger::RCLoggingFlags).
FLAG_HAS_VALID_INPUT = 1
FLAG_IN_RC_FAILSAFE = 2


@register_check
class RCInputCheck(Check):
    id = "rcin"
    title = "RC input"
    category = "rc"
    requires = {"RCIN"}
    description = (
        "Detects RC link loss while armed: RCI2.Flags 'no valid input' (4.x), else all primary channels "
        "dropping to a low/no-pulse value together."
    )

    def run(self, log: FlightLog) -> list[Finding]:
        if log.has("RCI2"):
            times, flags = log.series("RCI2", "Flags")
            if flags.size:
                return self._flag_findings(log, times, flags.astype(int))
        return self._pwm_findings(log)

    def _flag_findings(self, log: FlightLog, times: np.ndarray, flags: np.ndarray) -> list[Finding]:
        lost = (flags & FLAG_HAS_VALID_INPUT) == 0
        armed = log.armed_mask(times)
        if armed is not None:
            lost &= armed
        intervals = [
            iv
            for iv in intervals_above(times, lost.astype(float), 0.5, min_duration_s=SUSTAINED_S)
            if iv.peak >= 1.0
        ]
        if not intervals:
            return []
        worst = max(intervals, key=lambda iv: iv.duration_s)
        lost_to_end = (float(times[-1]) - worst.end_s) < 1.0
        win = (times >= worst.start_s) & (times <= worst.end_s)
        failsafe = bool(np.any((flags[win] & FLAG_IN_RC_FAILSAFE) != 0))
        sev = Severity.CRITICAL if (lost_to_end or worst.duration_s >= 2.0) else Severity.WARN
        fs_txt = (
            " The firmware entered RC failsafe during the loss."
            if failsafe
            else " The firmware did NOT report RC failsafe during the loss — check the failsafe configuration."
        )
        tail = " The link did not recover before the log ended." if lost_to_end else ""
        return [
            make_finding(
                self.id,
                sev,
                "RC signal loss",
                f"While armed, the autopilot had no valid RC input (RCI2.Flags HAS_VALID_INPUT clear) for "
                f"{worst.duration_s:.1f} s starting at {worst.start_s:.0f} s ({len(intervals)} loss window(s))."
                + fs_txt
                + tail,
                recommendation=(
                    "Check the RC link (receiver power, antenna placement, range, binding) and confirm the "
                    "radio failsafe (FS_THR_ENABLE / RC_FS_TIMEOUT) behaves as intended."
                ),
                time_start_s=worst.start_s,
                time_end_s=worst.end_s,
                message_types=["RCI2"],
                samples={
                    "lost_duration_s": round(worst.duration_s, 1),
                    "loss_windows": len(intervals),
                    "rc_failsafe_flag": 1.0 if failsafe else 0.0,
                    "lost_to_end": 1.0 if lost_to_end else 0.0,
                },
            )
        ]

    def _pwm_findings(self, log: FlightLog) -> list[Finding]:
        # Gather aligned primary channels by index.
        chans: dict[str, np.ndarray] = {}
        times: np.ndarray | None = None
        for ch in _PRIMARY:
            t, v = log.series("RCIN", ch)
            if v.size == 0:
                return []  # not enough channel data to reason about
            chans[ch] = v
            times = t if times is None else times
        if times is None:
            return []
        n = min(times.size, *[v.size for v in chans.values()])
        if n == 0:
            return []
        times = times[:n]
        stack = np.vstack([chans[ch][:n] for ch in _PRIMARY])  # shape (4, n)

        # All four primary channels simultaneously below the no-signal threshold.
        all_low_b = np.all(stack <= NO_SIGNAL_US, axis=0)
        armed = log.armed_mask(times)
        if armed is not None:
            all_low_b &= armed
        all_low = all_low_b.astype(float)
        intervals = intervals_above(times, all_low + 0.0, 0.5, min_duration_s=SUSTAINED_S)
        # intervals_above with threshold 0.5 on a 0/1 mask finds windows where mask==1.
        intervals = [iv for iv in intervals if iv.peak >= 1.0]
        if not intervals:
            return []

        worst = max(intervals, key=lambda iv: iv.duration_s)
        # Did the link stay lost to the end of the log? (most damning)
        log_end = float(times[-1])
        lost_to_end = (log_end - worst.end_s) < 1.0

        min_val = float(stack.min())
        sev = Severity.CRITICAL if (lost_to_end or worst.duration_s >= 2.0) else Severity.WARN
        tail = (
            " The link did not recover before the log ended, consistent with an RC-loss-induced crash."
            if lost_to_end
            else ""
        )
        return [
            make_finding(
                self.id,
                sev,
                "RC signal loss",
                f"All primary RC channels (C1-C4) dropped together to ~{min_val:.0f} us "
                f"(below the {NO_SIGNAL_US:.0f} us valid floor) for {worst.duration_s:.1f} s starting at "
                f"{worst.start_s:.0f} s. Normal flight never drives roll, pitch, throttle and yaw all low "
                f"at once, so this is the receiver losing signal or outputting a failsafe value.{tail}",
                recommendation=(
                    "Check the RC link (receiver power, antenna, range, binding) and verify the throttle "
                    "failsafe (FS_THR_ENABLE/FS_THR_VALUE) actually triggers at this value."
                ),
                time_start_s=worst.start_s,
                time_end_s=worst.end_s,
                message_types=["RCIN"],
                samples={
                    "min_pwm_us": round(min_val, 0),
                    "lost_duration_s": round(worst.duration_s, 1),
                    "lost_to_end": 1.0 if lost_to_end else 0.0,
                },
            )
        ]
