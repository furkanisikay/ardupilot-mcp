"""Startup & pre-arm message check.

ArduPilot writes human-readable status lines into the ``MSG`` message (field
``Message``) and, on some firmware/GCS combinations, ``STATUSTEXT`` (field
``Text``). Most of these are routine boot chatter (firmware/board banners, frame
detection, sensor initialisation, GPS auto-detect) that says nothing about the
health of the vehicle. A handful, however, are the firmware *itself* reporting a
configuration or health problem: pre-arm refusals, "not calibrated",
inconsistent sensors, mag anomalies, glitches, resets and failsafes.

This check surfaces the *notable* lines and filters out the routine ones, so the
diagnosis quotes exactly what the autopilot said about its own state. It is a
pure text classifier — no thresholds on numeric telemetry — so it always runs
(``requires`` is empty) and simply returns ``[]`` when nothing notable was said.
"""

from __future__ import annotations

import re

from ..flight_log import FlightLog
from ..model import Finding, Severity
from .base import Check, register_check
from .util import make_finding

# Substrings (case-insensitive) that mark a line as worth surfacing. Order does
# not matter for detection; severity is decided separately below.
NOTABLE_TOKENS: tuple[str, ...] = (
    "prearm",
    "arm:",
    "not calibrated",
    "inconsistent",
    "unhealthy",
    "ground mag anomaly",
    "error",
    "bad ",
    "failed",
    "failsafe",
    " reset",
    "glitch",
    "denied",
)

# Of the notable lines, these signals make it a WARN (a real config/health
# problem the firmware refused to ignore). Everything else notable is INFO
# (anomaly auto-corrected, re-alignment, reset, informational failsafe).
WARN_TOKENS: tuple[str, ...] = (
    "prearm",
    "error",
    "failed",
    "bad ",
    "denied",
    "not calibrated",
)

# Substrings (case-insensitive) that mark a line as ROUTINE boot chatter to be
# ignored even if it happened to brush a notable token.
ROUTINE_TOKENS: tuple[str, ...] = (
    "initialis",
    "initializ",
    "alignment complete",
    " ready",
    "detected as",
    "rcout",
    "rcin",
    "frame:",
    "param space",
    "u-blox",
    "gps 1:",
    "gps 2:",
)

# Maximum number of findings emitted; if more distinct notable lines exist we
# emit this many and note the overflow on the last finding.
MAX_FINDINGS = 6

# Matches an 8-hex-digit token (board id / silicon serial words).
_HEX8 = re.compile(r"\b[0-9a-fA-F]{8}\b")


def _is_routine(low: str) -> bool:
    """True if a (lowercased) line is routine boot chatter to ignore."""
    for tok in ROUTINE_TOKENS:
        if tok in low:
            return True
    # "calibrated" alone (e.g. "Compass calibrated") is routine, but
    # "not calibrated" is a real problem -> only routine WITHOUT "not ".
    if "calibrated" in low and "not " not in low:
        return True
    # "EKF ... IMU ... initial ..." boot lines (e.g. "initial yaw alignment").
    if "ekf" in low and "imu" in low and "initial" in low:
        return True
    # Board-id / silicon-serial lines: two or more 8-hex-digit tokens, or a
    # line that is mostly hex.
    if len(_HEX8.findall(low)) >= 2:
        return True
    return False


def _is_notable(low: str) -> bool:
    return any(tok in low for tok in NOTABLE_TOKENS)


def _severity(low: str) -> Severity:
    return Severity.WARN if any(tok in low for tok in WARN_TOKENS) else Severity.INFO


@register_check
class StartupMessagesCheck(Check):
    id = "prearm"
    title = "Startup & pre-arm messages"
    category = "config"
    requires: set[str] = set()  # always runs; reads MSG/STATUSTEXT text directly
    description = "Notable startup / pre-arm status lines the firmware logged in MSG/STATUSTEXT."

    def run(self, log: FlightLog) -> list[Finding]:
        notable = self._collect_notable(log)
        if not notable:
            return []

        total = len(notable)
        capped = notable[:MAX_FINDINGS]
        findings: list[Finding] = []
        for idx, (text, ts, sev) in enumerate(capped):
            label = self._label(text)
            kind = "warning" if sev is Severity.WARN else "note"
            explanation = f'The firmware logged: "{text}". ' + (
                "This is a pre-arm / configuration / health message the autopilot "
                "emitted itself; review it before flying."
                if sev is Severity.WARN
                else "This is a status note the autopilot emitted; usually informational "
                "(an anomaly it auto-corrected, a reset, or an info-level event)."
            )
            samples: dict[str, float] = {"notable_messages": total}
            is_last = idx == len(capped) - 1
            if is_last and total > MAX_FINDINGS:
                explanation += (
                    f" ({total} notable startup/pre-arm message(s) found; showing the first {MAX_FINDINGS}.)"
                )
                samples["shown"] = MAX_FINDINGS
            findings.append(
                make_finding(
                    self.id,
                    sev,
                    f"Startup {kind}: {label}",
                    explanation,
                    time_start_s=ts,
                    message_types=["MSG"],
                    samples=samples,
                )
            )
        return findings

    def _collect_notable(self, log: FlightLog) -> list[tuple[str, float | None, Severity]]:
        """Distinct notable lines, earliest timestamp, in first-seen order."""
        # Map normalised text -> (original text, earliest timestamp, severity).
        seen: dict[str, list] = {}
        order: list[str] = []
        for text, ts in self._iter_text(log):
            stripped = text.strip()
            if not stripped:
                continue
            low = stripped.lower()
            if not _is_notable(low):
                continue
            if _is_routine(low):
                continue
            key = stripped
            if key not in seen:
                seen[key] = [stripped, ts, _severity(low)]
                order.append(key)
            else:
                # Keep the earliest timestamp for a repeated line.
                prev_ts = seen[key][1]
                if ts is not None and (prev_ts is None or ts < prev_ts):
                    seen[key][1] = ts
        return [tuple(seen[k]) for k in order]

    @staticmethod
    def _iter_text(log: FlightLog):
        """Yield (text, timestamp) from MSG (Message) and STATUSTEXT (Text)."""
        for rec in log.get("MSG"):
            val = rec.get("Message")
            if isinstance(val, str):
                ts = float(rec["timestamp"]) if "timestamp" in rec else None
                yield val, ts
        for rec in log.get("STATUSTEXT"):
            val = rec.get("Text")
            if isinstance(val, str):
                ts = float(rec["timestamp"]) if "timestamp" in rec else None
                yield val, ts

    @staticmethod
    def _label(text: str) -> str:
        """Short label for the finding title (the message, trimmed)."""
        clean = text.strip().replace("\r", " ").replace("\n", " ")
        clean = " ".join(clean.split())
        return clean if len(clean) <= 80 else clean[:77] + "..."
