"""RC input check.

Reads ``RCIN`` (the RC channels the autopilot *received*). The clearest failure
signature is loss of the RC link: the receiver drops or holds all channels at an
abnormally low/failsafe value (~900-1000 us) at the same time. In normal flight
the sticks are independent — roll/pitch/yaw sit near centre (~1500 us) while only
throttle is low — so *all* primary channels being low together does not happen
unless the link failed. (This catches the classic "RC stuck at ~990 us, failsafe
didn't trigger" crash.)
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


@register_check
class RCInputCheck(Check):
    id = "rcin"
    title = "RC input"
    category = "rc"
    requires = {"RCIN"}
    description = "Detects RC link loss (all received channels drop to a low/failsafe value together)."

    def run(self, log: FlightLog) -> list[Finding]:
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
        all_low = np.all(stack <= NO_SIGNAL_US, axis=0).astype(float)
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
