# SPDX-License-Identifier: AGPL-3.0-only
"""Vibration check.

Reads the ``VIBE`` message: per-axis vibration (VibeX/Y/Z, m/s^2) and the
cumulative accelerometer-clip counter(s). Modern firmware (4.x) logs one VIBE
stream per IMU, distinguished by the ``IMU`` instance field, each with a single
``Clip`` counter ("TimeUS,IMU,VibeX,VibeY,VibeZ,Clip" in Copter-4.6.3
libraries/AP_InertialSensor/LogStructure.h). Older firmware logged one stream with
``Clip0/1/2`` (one counter per IMU). Both layouts are handled, per IMU. Per ArduPilot's vibration
guidance, sustained vibration below ~30 m/s^2 is generally acceptable, 30-60 may
cause problems, and >60 nearly always degrades position/altitude hold. Ideal
clipping is zero; a small count (<~100) is likely ok (especially around hard
landings), but a steadily increasing count means the IMU is railing — a serious
vibration problem. Accordingly a count under 100 is noted, a count of 100 or more is a
warning, and critical once it keeps rising (in separate episodes, or for longer than a crash
lasts). Vibration levels are
judged in flight only, and as sustained levels, not single peaks.

Source: https://ardupilot.org/copter/docs/common-measuring-vibration.html
"""

from __future__ import annotations

import numpy as np

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import fraction_above, intervals_above, longest_run_above, make_finding

WARN_MS2 = 30.0
CRIT_MS2 = 60.0
# Fraction of the flight above WARN that turns a marginal note into a warning, and above
# CRIT that makes it critical.
SUSTAINED_FRACTION = 0.10
# Design choice: the wiki's levels are sustained levels ("levels above 60m/s/s nearly
# always have problems"), not peaks. A run above CRIT this long, while flying, is a level;
# a shorter one is a spike (a hard manoeuvre, touchdown or the impact itself). On the 48
# benchmark crash logs no run above 60 m/s^2 before the impact lasted longer than 0.6 s,
# and none of those crashes was caused by vibration.
CRIT_MIN_DURATION_S = 2.0
# Clip count at which clipping is serious: the wiki's "less than ~100 is likely ok".
CLIP_CRIT_COUNT = 100
# Design choice: the wiki calls a *steadily increasing* clip count serious and a few clips
# around a hard landing normal. Clips closer together than this belong to one episode
# (one impact, one violent manoeuvre)...
CLIP_EPISODE_GAP_S = 5.0
# ...and clipping in at least this many separate episodes, or one episode lasting at least
# CLIP_STEADY_SPAN_S, is the steady rise of a vibration problem. Anything shorter, however
# many clips it holds, is an event. On the 48 benchmark crash logs every count of 100+ came
# from one or two bursts of at most 4.6 s, at the loss of control or a hard landing (a crash
# is detected after 2 s of lost attitude control, crash_check.cpp).
CLIP_STEADY_EPISODES = 3
CLIP_STEADY_SPAN_S = 8.0


@register_check
class VibrationCheck(Check):
    id = "vibration"
    title = "Vibration levels"
    category = "vibration"
    requires = {"VIBE"}
    description = "Per-axis vibration magnitude and accelerometer clipping from VIBE."

    def run(self, log: FlightLog) -> list[Finding]:
        findings: list[Finding] = []
        findings.extend(self._axis_findings(log))
        findings.extend(self._clip_findings(log))
        return findings

    def _axis_findings(self, log: FlightLog) -> list[Finding]:
        out: list[Finding] = []
        imus = log.instances("VIBE")
        primary = primary_imu(log, imus)
        for axis in ("VibeX", "VibeY", "VibeZ"):
            # Judge each IMU on its own samples: interleaving IMUs would mix a quiet,
            # isolated IMU with a noisy one.
            main = self._axis_finding(log, axis, primary, multi=len(imus) > 1)
            # Worst secondary IMU, one severity lower (see primary_imu): reported
            # only when it adds something beyond the primary's own finding.
            side: Finding | None = None
            for imu in imus:
                if imu == primary:
                    continue
                f = self._axis_finding(log, axis, imu, multi=True)
                if f is not None and (
                    side is None or f.evidence.samples[f"max_{axis}"] > side.evidence.samples[f"max_{axis}"]
                ):
                    side = f
            if main is not None:
                out.append(main)
            if (
                side is not None
                and demote(side)
                and (main is None or side.severity.rank > main.severity.rank)
            ):
                out.append(side)
        return out

    def _axis_finding(self, log: FlightLog, axis: str, imu: int, *, multi: bool) -> Finding | None:
        times, vals = log.series("VIBE", axis, imu)
        ok = np.isfinite(vals) & np.isfinite(times)
        times, vals = times[ok], vals[ok]
        # Judge flight only: on the ground (before arming, after landing, the airframe lying
        # after a crash) the readings are handling and impacts, not flight vibration.
        mask = log.flying_mask(times)
        if mask is not None:
            times, vals = times[mask], vals[mask]
        if vals.size == 0:
            return None
        vmax = float(np.max(vals))
        if vmax <= WARN_MS2:
            return None
        frac_warn = fraction_above(vals, WARN_MS2)
        frac_crit = fraction_above(vals, CRIT_MS2)
        # Runs break where time jumps (the ground time between two flights is masked out),
        # so a spike before one landing and one after the next take-off are not one run.
        longest_crit = longest_run_above(times, vals, CRIT_MS2)
        where = f" on IMU{imu}" if multi else ""
        if longest_crit >= CRIT_MIN_DURATION_S or frac_crit >= SUSTAINED_FRACTION:
            sev = Severity.CRITICAL
            msg = (
                f"{axis} vibration{where} stayed above {CRIT_MS2:.0f} m/s^2 for up to {longest_crit:.1f} s "
                f"(peak {vmax:.0f} m/s^2; {frac_crit * 100:.0f}% of the flight above {CRIT_MS2:.0f}). "
                "Vibration this high corrupts the accelerometers and can trigger EKF failures."
            )
            rec: str | None = (
                "Improve flight-controller isolation/balance; check prop/motor condition before flying again."
            )
        elif frac_warn >= SUSTAINED_FRACTION:
            sev = Severity.WARN
            msg = (
                f"{axis} vibration{where} is elevated: peak {vmax:.0f} m/s^2, "
                f"{frac_warn * 100:.0f}% of the flight above {WARN_MS2:.0f} m/s^2."
            )
            rec = "Inspect mounting/balance; consider the harmonic notch filter (see recommend_tuning)."
        else:
            sev = Severity.INFO
            spike = (
                f"; the longest run above {CRIT_MS2:.0f} m/s^2 lasted {longest_crit:.1f} s, a spike "
                "(a hard manoeuvre, touchdown or impact) rather than a vibration level"
                if vmax > CRIT_MS2
                else ""
            )
            msg = (
                f"{axis} vibration{where} is mostly fine but peaks at {vmax:.0f} m/s^2 "
                f"(brief excursions above {WARN_MS2:.0f} m/s^2{spike})."
            )
            rec = None
        ivs = intervals_above(times, vals, WARN_MS2)
        return make_finding(
            self.id,
            sev,
            f"Elevated {axis} vibration" + (f" (IMU{imu})" if multi else ""),
            msg,
            recommendation=rec,
            time_start_s=ivs[0].start_s if ivs else None,
            time_end_s=ivs[-1].end_s if ivs else None,
            message_types=["VIBE"],
            samples={
                f"max_{axis}": round(vmax, 1),
                f"frac_above_{int(WARN_MS2)}": round(frac_warn, 3),
                f"longest_above_{int(CRIT_MS2)}_s": round(longest_crit, 2),
                "imu": imu,
            },
        )

    @staticmethod
    def _clip_streams(log: FlightLog) -> list[tuple[int, np.ndarray, np.ndarray]]:
        """(imu, times, cumulative clip counter) for every IMU, in either log layout."""
        streams: list[tuple[int, np.ndarray, np.ndarray]] = []
        for imu in log.instances("VIBE"):
            times, vals = log.series("VIBE", "Clip", imu)
            if vals.size:
                streams.append((imu, times, vals))
        if streams:
            return streams
        for imu, clip in enumerate(("Clip0", "Clip1", "Clip2")):  # pre-4.0 layout
            times, vals = log.series("VIBE", clip)
            if vals.size:
                streams.append((imu, times, vals))
        return streams

    def _clip_findings(self, log: FlightLog) -> list[Finding]:
        out: list[Finding] = []
        streams = self._clip_streams(log)
        primary = primary_imu(log, [imu for imu, _, _ in streams])
        for imu, times, vals in streams:
            # Counters are cumulative. Sum only the increments so a counter reset
            # (reboot mid-log) is not read as a negative or a huge jump.
            steps = np.diff(vals)
            total = float(np.sum(steps[steps > 0])) if steps.size else 0.0
            if total <= 0:
                continue
            first = int(np.flatnonzero(steps > 0)[0]) + 1
            onset = float(times[first]) if first < times.size else None
            clip_times = times[1:][steps > 0]
            breaks = np.flatnonzero(np.diff(clip_times) > CLIP_EPISODE_GAP_S)
            episodes = 1 + int(breaks.size)
            bounds = np.concatenate(([0], breaks + 1, [clip_times.size]))
            longest_span = max(
                (
                    float(clip_times[b - 1] - clip_times[a])
                    for a, b in zip(bounds[:-1], bounds[1:], strict=True)
                ),
                default=0.0,
            )
            # The wiki: "a small count (less than ~100) is likely ok, especially around a
            # hard landing"; a large, steadily rising count is a serious problem.
            if total < CLIP_CRIT_COUNT:
                sev = Severity.INFO
                meaning = (
                    f"Fewer than {CLIP_CRIT_COUNT} clips is usually fine (a hard landing or touchdown "
                    "produces a few); watch whether the count grows from flight to flight."
                )
            elif episodes >= CLIP_STEADY_EPISODES or longest_span >= CLIP_STEADY_SPAN_S:
                sev = Severity.CRITICAL
                meaning = (
                    f"The count kept rising ({episodes} episode(s), the longest {longest_span:.1f} s). Clipping "
                    "means the IMU saturated (railed); its data is unusable while clipped, which is a serious "
                    "vibration problem."
                )
            else:
                sev = Severity.WARN
                meaning = (
                    f"They came in {episodes} short burst(s) (the longest {longest_span:.1f} s), not as a steady "
                    "rise: the IMU railed during a violent event, which is usually the loss of control, a hard "
                    "landing or the impact itself rather than its cause. Check what happened just before."
                )
            out.append(
                make_finding(
                    self.id,
                    sev,
                    f"Accelerometer clipping (IMU{imu})",
                    f"IMU{imu} recorded {int(total)} accelerometer clipping event(s). " + meaning,
                    recommendation="Reduce vibration at the source (balance props, soft-mount the FC) before the next flight.",
                    time_start_s=onset,
                    time_end_s=float(times[-1]) if times.size else None,
                    message_types=["VIBE"],
                    samples={
                        "total_clips": int(total),
                        "clip_episodes": episodes,
                        "longest_clip_episode_s": round(longest_span, 1),
                        "imu": imu,
                        "crit_threshold": CLIP_CRIT_COUNT,
                    },
                )
            )
            if imu != primary and len(streams) > 1:
                demote(out[-1])
        return out


def primary_imu(log: FlightLog, imus: list[int]) -> int:
    """The IMU whose readings decide severity.

    Design choice: ArduPilot does not log which IMU is primary, so we take the first
    IMU that is in use (INS_USE / INS_USE2 / INS_USE3 != 0, default in use), i.e.
    normally IMU0. Secondary IMUs are still reported, one severity lower: some boards
    deliberately leave one IMU unisolated (the Cube's third IMU), so its higher
    vibration is expected and must not drive a CRITICAL on its own.
    """
    for imu in sorted(imus):
        use = log.params.get("INS_USE" if imu == 0 else f"INS_USE{imu + 1}")
        if use is None or use != 0:
            return imu
    return min(imus) if imus else 0


def demote(f: Finding) -> bool:
    """Lower a secondary IMU's finding one severity level; False if it drops below INFO."""
    if f.severity == Severity.INFO:
        return False
    f.severity = Severity.WARN if f.severity == Severity.CRITICAL else Severity.INFO
    f.explanation += " (Secondary IMU: reported one level lower; the primary IMU decides severity.)"
    return True
