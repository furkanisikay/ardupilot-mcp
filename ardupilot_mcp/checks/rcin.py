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
PWM heuristic is the fallback for older logs. Only flight is judged
(``FlightLog.flying_mask``: armed, and airborne where the log says so): a receiver
with no transmitter on the bench, or a failsafe tested on the ground, is not an
in-flight link loss; the firmware's failsafe mode change is still reported.
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
                found = self._flag_findings(log, times, flags.astype(int))
                if found:
                    return found
                # The receiver kept reporting valid input; one that outputs its own failsafe
                # values on a dropout (a preset, not "no pulses") is only visible in the PWM.
                return self._pwm_findings(log, receiver_valid=True)
        return self._pwm_findings(log)

    def _flag_findings(self, log: FlightLog, times: np.ndarray, flags: np.ndarray) -> list[Finding]:
        if not np.any((flags & FLAG_HAS_VALID_INPUT) != 0):
            # Never a valid RC input in the whole log: no link was lost. The vehicle was driven
            # without an RC receiver (ArduSub logs RCIN while piloted by a MAVLink joystick,
            # ArduSub.cpp; a companion computer or GCS can fly any vehicle), or none was connected.
            return [
                make_finding(
                    self.id,
                    Severity.INFO,
                    "No RC input in this log",
                    "The autopilot never received a valid RC input in this log (RCI2.Flags HAS_VALID_INPUT "
                    "never set). There was no RC link to lose: the vehicle was controlled another way (a "
                    "MAVLink joystick, as ArduSub normally is, a ground station or a companion computer), or "
                    "the receiver was not connected.",
                    time_start_s=float(times[0]),
                    time_end_s=float(times[-1]),
                    message_types=["RCI2"],
                    samples={"valid_input_samples": 0.0},
                )
            ]
        lost = (flags & FLAG_HAS_VALID_INPUT) == 0
        armed = log.flying_mask(times)
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

    def _pwm_findings(self, log: FlightLog, *, receiver_valid: bool = False) -> list[Finding]:
        # The primary channels of each RCIN record, kept together.
        times, cols = log.series_many("RCIN", list(_PRIMARY))
        if times.size == 0:
            return []  # not enough channel data to reason about
        stack = np.vstack(cols)  # shape (4, n)
        if not np.any(stack > NO_SIGNAL_US):
            if receiver_valid:
                return []
            # No primary channel ever carried a stick value: no receiver, so no link to lose.
            return [
                make_finding(
                    self.id,
                    Severity.INFO,
                    "No RC input in this log",
                    f"No primary RC channel ever rose above {NO_SIGNAL_US:.0f} us in this log. There was no "
                    "RC link to lose: the vehicle was controlled another way (a MAVLink joystick, a ground "
                    "station or a companion computer), or the receiver was not connected.",
                    time_start_s=float(times[0]),
                    time_end_s=float(times[-1]),
                    message_types=["RCIN"],
                    samples={"max_primary_us": float(np.nanmax(stack)) if np.isfinite(stack).any() else 0.0},
                )
            ]

        # All four primary channels simultaneously below the no-signal threshold.
        all_low_b = np.all(stack <= NO_SIGNAL_US, axis=0)
        armed = log.flying_mask(times)
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

        in_loss = (times >= worst.start_s) & (times <= worst.end_s)
        min_val = float(stack[:, in_loss].min()) if np.any(in_loss) else float(stack.min())
        # With FS_THR_ENABLE 0 nothing catches a throttle that low: it goes to the motors.
        fs_off = log.params.get("FS_THR_ENABLE") == 0 and log.meta.vehicle_kind in ("copter", "heli")
        sev = Severity.CRITICAL if (lost_to_end or worst.duration_s >= 2.0 or fs_off) else Severity.WARN
        tail = (
            " The link did not recover before the log ended, consistent with an RC-loss-induced crash."
            if lost_to_end
            else ""
        )
        if receiver_valid:
            tail += (
                " The receiver kept reporting valid input (RCI2.Flags) throughout, so these are values it "
                "sent itself: a failsafe preset or a glitch, which the autopilot flies as real stick inputs."
            )
        if fs_off:
            tail += " FS_THR_ENABLE is 0, so the radio failsafe could not catch the low throttle."
        modes = [
            float(r["timestamp"])
            for r in log.get("MODE")
            if isinstance(r.get("timestamp"), (int, float))
            and any(iv.start_s - 0.2 <= float(r["timestamp"]) <= iv.end_s + 0.5 for iv in intervals)
        ]
        if modes:
            tail += f" The flight mode changed during a drop ({len(modes)} time(s)): the mode channel fell with it."
        return [
            make_finding(
                self.id,
                sev,
                "RC signal loss",
                f"All primary RC channels (C1-C4) dropped together to ~{min_val:.0f} us "
                f"(below the {NO_SIGNAL_US:.0f} us valid floor) for {worst.duration_s:.1f} s starting at "
                f"{worst.start_s:.0f} s ({len(intervals)} drop(s) of {SUSTAINED_S:.1f} s or more). Normal flight "
                f"never drives roll, pitch, throttle and yaw all low at once, so this is the receiver losing "
                f"signal or outputting a failsafe value.{tail}",
                recommendation=(
                    "Check the RC link (receiver power, antenna, range, binding) and verify the throttle "
                    "failsafe (FS_THR_ENABLE/FS_THR_VALUE) actually triggers at this value."
                ),
                time_start_s=worst.start_s,
                time_end_s=worst.end_s,
                message_types=["RCIN", "RCI2"] if receiver_valid else ["RCIN"],
                samples={
                    "min_pwm_us": round(min_val, 0),
                    "lost_duration_s": round(worst.duration_s, 1),
                    "lost_to_end": 1.0 if lost_to_end else 0.0,
                    "drops": len(intervals),
                },
            )
        ]
