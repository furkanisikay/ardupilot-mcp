"""A Markdown report to paste into a forum thread (discuss.ardupilot.org).

Most help requests on the forum start with "here is my log, what happened?". This builds
the reply's starting point from the deterministic diagnosis: the vehicle, what went wrong
first, the findings with their evidence, and a small text plot of the data behind each
one, so a helper can see the shape of the problem without opening the log.

It is read-only and returns text; nothing is written anywhere. It never reads or prints
a position: no GPS latitude/longitude, home or waypoint location, only counts such as
satellites and HDOP. The file path is reduced to its name.
"""

from __future__ import annotations

import os
import re
from collections.abc import Callable

import numpy as np

from . import __version__
from .checks.thrust_loss import motor_channel
from .flight_log import FlightLog
from .model import DiagnosisReport, Finding, IncidentTimeline, Severity
from .profile import build_profile

# Design choices: how much of the report a forum post can carry.
MAX_FINDINGS = 6
PLOT_WIDTH = 48
PLOT_PAD_S = 10.0  # seconds of context either side of a finding's window
_BLOCKS = "▁▂▃▄▅▆▇█"

_INSTANCE = re.compile(r"\((?:IMU|core |GPS)(\d)\)")
_AXIS = re.compile(r"\((roll|pitch|yaw)\)")
_EKF_FIELD = re.compile(r"\((SV|SP|SH|SM|SVT)\)")

Agg = Callable[[np.ndarray], float]


def _instance(f: Finding) -> int | None:
    s = f.evidence.samples
    for key in ("imu", "ekf_core", "gps_instance"):
        if key in s:
            return int(s[key])
    m = _INSTANCE.search(f.title)
    return int(m.group(1)) if m else None


def _series_for(log: FlightLog, f: Finding) -> list[tuple[str, str, int | None, Agg]]:
    """(message, field, instance, bin aggregate) to plot for a finding; [] when nothing fits."""
    inst = _instance(f)
    cid, title = f.check_id, f.title
    if cid == "vibration":
        if "clipping" in title.lower():
            return [("VIBE", "Clip", inst, np.max)]
        axis = next((a for a in ("VibeX", "VibeY", "VibeZ") if a in title), "VibeZ")
        return [("VIBE", axis, inst, np.max)]
    if cid == "attitude":
        m = _AXIS.search(title)
        axis = (m.group(1) if m else "roll").capitalize()
        return [("ATT", f"Des{axis}", None, np.mean), ("ATT", axis, None, np.mean)]
    if cid == "power":
        if "vcc" in title.lower() or "board" in title.lower():
            return [("POWR", "Vcc", None, np.min)]
        return [("BAT", "Volt", inst, np.min), ("BAT", "Curr", inst, np.max)]
    if cid == "rcin":
        return [("RCIN", "C3", None, np.min)]
    if cid == "gps":
        return [("GPS", "NSats", inst, np.min), ("GPS", "HDop", inst, np.max)]
    if cid == "ekf":
        m = _EKF_FIELD.search(title)
        if m:
            return [("XKF4" if log.has("XKF4") else "NKF4", m.group(1), inst, np.max)]
        return []
    if cid == "thrust_loss":
        motor = f.evidence.samples.get("signature_motor", f.evidence.samples.get("motor"))
        out: list[tuple[str, str, int | None, Agg]] = [("CTUN", "ThO", None, np.max)]
        if motor is not None:
            out.insert(0, ("RCOU", motor_channel(log, int(motor)), None, np.max))
        return out
    if cid == "motors":
        m = re.search(r"\((C\d+)\)", title)
        return [("RCOU", m.group(1), None, np.max)] if m else []
    return []


def sparkline(values: np.ndarray, width: int = PLOT_WIDTH, agg: Agg = np.max) -> str:
    """Values squeezed into ``width`` bins and drawn with block characters (lowest to highest)."""
    v = values[np.isfinite(values)]
    if v.size == 0:
        return ""
    bins = np.array_split(v, min(width, v.size))
    pts = np.array([agg(b) for b in bins], dtype=float)
    lo, hi = float(np.min(pts)), float(np.max(pts))
    if hi - lo < 1e-12:
        return _BLOCKS[0] * pts.size
    idx = np.clip(((pts - lo) / (hi - lo) * (len(_BLOCKS) - 1)).round().astype(int), 0, len(_BLOCKS) - 1)
    return "".join(_BLOCKS[i] for i in idx)


def _fmt(x: float) -> str:
    return f"{x:.0f}" if abs(x) >= 100 else f"{x:.2f}".rstrip("0").rstrip(".")


def _plot(log: FlightLog, f: Finding) -> list[str]:
    t0, t1 = f.evidence.time_start_s, f.evidence.time_end_s
    if t0 is None:
        return []
    t1 = t1 if t1 is not None and t1 >= t0 else t0
    lo, hi = t0 - PLOT_PAD_S, t1 + PLOT_PAD_S
    lines = []
    for mtype, name, inst, agg in _series_for(log, f):
        if inst is not None and inst not in (log.instances(mtype) or [0]):
            inst = None
        times, vals = log.series(mtype, name, inst)
        win = (times >= lo) & (times <= hi)
        if int(win.sum()) < 2:
            continue
        v = vals[win]
        if name in ("Yaw", "DesYaw"):
            # Heading wraps at 0/360 deg; unwrapped, a turn through north is not a spike.
            v = np.degrees(np.unwrap(np.radians(v)))
        label = f"{mtype}.{name}" + (f"[{inst}]" if inst is not None and log.instance_field(mtype) else "")
        lines.append(
            f"{label:<14} {_fmt(float(np.min(v))):>7} {sparkline(v, agg=agg)} {_fmt(float(np.max(v)))}"
        )
    if not lines:
        return []
    return ["```", f"t={lo:.1f}..{hi:.1f}s  (min, then max at the right)", *lines, "```"]


def _vehicle_line(log: FlightLog) -> str:
    p = build_profile(log)
    parts = [p.firmware_version or log.meta.vehicle_type or "unknown firmware"]
    frame = " ".join(x for x in (p.frame_class_name, p.frame_type_name) if x)
    if frame:
        parts.append(frame)
    if p.battery_cells:
        parts.append(f"{p.battery_cells}S (estimated)")
    if log.meta.duration_s is not None:
        parts.append(f"log {log.meta.duration_s / 60:.1f} min")
    if log.meta.max_altitude_m is not None:
        parts.append(f"max {log.meta.max_altitude_m:.0f} m above home")
    return " · ".join(parts)


def _when(f: Finding) -> str:
    t0, t1 = f.evidence.time_start_s, f.evidence.time_end_s
    if t0 is None:
        return ""
    if t1 is not None and t1 - t0 >= 0.5:
        return f" (t={t0:.1f}–{t1:.1f}s)"
    return f" (t={t0:.1f}s)"


def build_forum_report(log: FlightLog, report: DiagnosisReport, timeline: IncidentTimeline | None) -> str:
    """The Markdown text of a forum reply for this log."""
    name = os.path.basename(log.path) if log.path else "log"
    out = [f"### Log review: `{name}`", "", f"**Vehicle:** {_vehicle_line(log)}"]
    if log.meta.flight_modes:
        out.append(f"**Modes:** {' → '.join(log.meta.flight_modes)}")
    out.append("")

    out.append("#### What went wrong first")
    if timeline is not None and (timeline.onset_s is not None or timeline.impact_s is not None):
        if timeline.onset_s is not None:
            out.append(f"- **First anomaly** at t={timeline.onset_s:.1f}s: {timeline.onset_basis}")
        if timeline.impact_s is not None:
            out.append(f"- **Impact** at t={timeline.impact_s:.1f}s: {timeline.impact_basis}")
        out.append("- _Order in time is not proof of cause._")
    else:
        out.append("- No in-flight anomaly or crash was found in this log.")
    out.append("")

    ranked = [f for f in report.findings if f.severity != Severity.INFO and not f.after_impact]
    if timeline is not None:
        flight_long = set(timeline.conditions)
        ranked = [f for f in ranked if f.title not in flight_long] + [
            f for f in ranked if f.title in flight_long
        ]
    out.append("#### Findings")
    if not ranked:
        out.append("No warnings. Every check that had data passed.")
    for i, f in enumerate(ranked[:MAX_FINDINGS], 1):
        out.append(f"{i}. **{f.severity.value.upper()}: {f.title}**{_when(f)}")
        out.append(f"   {f.explanation}")
        if f.related:
            out.append(f"   _Also in the log:_ {'; '.join(f.related)}")
        out.extend("   " + line for line in _plot(log, f))
    if len(ranked) > MAX_FINDINGS:
        rest = ", ".join(f.title for f in ranked[MAX_FINDINGS:])
        out.append(f"\n_{len(ranked) - MAX_FINDINGS} more:_ {rest}")
    after = [f for f in report.findings if f.after_impact and f.severity != Severity.INFO]
    if after:
        out.append("")
        out.append(
            f"_After the impact ({len(after)}, most likely consequences of the crash):_ "
            + ", ".join(dict.fromkeys(f.title for f in after))
        )
    if timeline is not None and timeline.conditions:
        out.append("")
        out.append("#### Present for the whole flight (context, not the cause)")
        out.extend(f"- {c}" for c in timeline.conditions)

    missing = [s for s in report.checks_skipped if "not logged" in s.reason]
    if missing:
        out.append("")
        out.append("#### Not checked (data not in the log)")
        out.extend(f"- {s.title}: {s.reason}" for s in missing)
    if report.logging_advice:
        out.append("")
        out.append("#### To log next time")
        out.extend(f"- {a.missing} (for {a.needed_for}): {a.setting}" for a in report.logging_advice)

    out.append("")
    out.append(
        f"<sub>Deterministic analysis by ardupilot-mcp {__version__}, offline; this text contains no "
        "GPS positions. Findings come from the log alone: add what you saw, the frame, props and weight.</sub>"
    )
    return "\n".join(out)


__all__ = ["build_forum_report", "sparkline"]
