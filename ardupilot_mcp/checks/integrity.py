"""Log integrity check.

This is a meta-check: it does not look at any DataFlash message stream, it
reports on the *trustworthiness of the log file itself* as determined by the
parser. ``log.meta.integrity`` is a :class:`LogIntegrity` enum:

* ``OK``        - the log was parsed cleanly and ends normally; nothing to flag.
* ``TRUNCATED`` - the byte stream ends abruptly (no clean shutdown record). This
  is the classic signature of a crash, brown-out or sudden power loss: the
  autopilot stopped writing mid-flight. Any diagnosis is then based on partial
  data, so downstream checks may be missing the most interesting final seconds.
* ``PARTIAL``   - the file was readable end-to-end, but the parser hit message(s)
  it could not decode, so some records were dropped. Results may be incomplete.

Both abnormal states are reported as WARN: they do not by themselves prove a
fault, but they tell the reader to treat the rest of the report as provisional.
The parser's own ``integrity_detail`` string (when present) is surfaced verbatim
in the evidence ``detail`` so a human can see exactly what the parser observed.
"""

from __future__ import annotations

from ..flight_log import FlightLog
from ..model import Finding, LogIntegrity, Severity
from .base import Check, register_check
from .util import make_finding

# Evidence.samples is dict[str, float], so the integrity state is encoded as a
# numeric flag (the human-readable state name lives in the explanation/detail).
_TRUNCATED_FLAG = 1.0
_PARTIAL_FLAG = 2.0
_UNKNOWN_FLAG = 9.0


@register_check
class IntegrityCheck(Check):
    id = "integrity"
    title = "Log integrity"
    category = "integrity"
    # Meta-check on the parse result itself - no message stream is required, so
    # it always runs (even on an otherwise empty/garbage log).
    requires: set[str] = set()
    description = "Reports whether the log parsed cleanly or was truncated/partial."

    def run(self, log: FlightLog) -> list[Finding]:
        integrity = log.meta.integrity
        detail = log.meta.integrity_detail

        # Clean parse: nothing to report.
        if integrity == LogIntegrity.OK:
            return []

        if integrity == LogIntegrity.TRUNCATED:
            return [
                make_finding(
                    self.id,
                    Severity.WARN,
                    "Log is truncated",
                    (
                        "The log ends abruptly without a clean shutdown, so the recording was "
                        "cut short. This commonly follows a crash, brown-out or power loss, and "
                        "it means the diagnosis is based on partial data - the final moments of "
                        "the flight may be missing."
                    ),
                    recommendation=(
                        "Treat the rest of this report as provisional; the most relevant evidence "
                        "may be in the unrecorded final seconds. Check power wiring/battery and the "
                        "SD card for the cause of the abrupt stop."
                    ),
                    message_types=[],
                    samples={"truncated": _TRUNCATED_FLAG},
                    detail=detail,
                )
            ]

        if integrity == LogIntegrity.PARTIAL:
            return [
                make_finding(
                    self.id,
                    Severity.WARN,
                    "Log parsed with errors",
                    (
                        "Some messages in the log could not be parsed and were dropped, so the "
                        "data set is incomplete. Findings (and their absence) may be unreliable "
                        "because the affected records were never seen by the checks."
                    ),
                    recommendation=(
                        "Treat the rest of this report as provisional. Re-download the log if "
                        "possible; a corrupted transfer or a firmware/parser mismatch can cause "
                        "undecodable messages."
                    ),
                    message_types=[],
                    samples={"partial": _PARTIAL_FLAG},
                    detail=detail,
                )
            ]

        # Defensive: an unknown integrity state should never silently pass.
        return [
            make_finding(
                self.id,
                Severity.WARN,
                "Log integrity unknown",
                (
                    f"The parser reported an unrecognised integrity state ({integrity!r}); "
                    "the log may not be fully trustworthy."
                ),
                message_types=[],
                samples={"unknown": _UNKNOWN_FLAG},
                detail=detail,
            )
        ]
