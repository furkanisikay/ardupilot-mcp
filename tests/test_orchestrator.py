"""Tests for the diagnosis orchestrator, using purpose-built fake checks."""

from __future__ import annotations

from ardupilot_mcp.checks.base import Check
from ardupilot_mcp.checks.util import make_finding
from ardupilot_mcp.model import CheckStatus, Severity
from ardupilot_mcp.orchestrator import diagnose
from tests.helpers import make_flight_log


class _Emit(Check):
    id = "emit"
    title = "Emit"
    requires = set()

    def run(self, log):
        return [
            make_finding("emit", Severity.WARN, "warn-late", "w", time_start_s=10.0),
            make_finding("emit", Severity.CRITICAL, "crit-late", "c", time_start_s=20.0),
            make_finding("emit", Severity.CRITICAL, "crit-early", "c", time_start_s=5.0),
            make_finding("emit", Severity.INFO, "info", "i"),
        ]


class _NeedsVibe(Check):
    id = "needs_vibe"
    title = "Needs VIBE"
    requires = {"VIBE"}

    def run(self, log):
        return []


class _Boom(Check):
    id = "boom"
    title = "Boom"
    requires = set()

    def run(self, log):
        raise RuntimeError("kaboom")


def test_severity_and_time_sorting():
    log = make_flight_log({})
    rep = diagnose(log, checks=[_Emit()])
    titles = [f.title for f in rep.findings]
    # critical first (earliest critical before later critical), then warn, then info
    assert titles == ["crit-early", "crit-late", "warn-late", "info"]
    assert rep.critical_count == 2 and rep.warn_count == 1 and rep.info_count == 1


def test_skipped_check_recorded_not_fatal():
    log = make_flight_log({})  # no VIBE
    rep = diagnose(log, checks=[_NeedsVibe()])
    statuses = {r.check_id: r.status for r in rep.results}
    assert statuses["needs_vibe"] == CheckStatus.SKIPPED
    assert any(s.check_id == "needs_vibe" for s in rep.checks_skipped)


def test_erroring_check_is_isolated():
    log = make_flight_log({})
    rep = diagnose(log, checks=[_Boom(), _Emit()])
    boom = next(r for r in rep.results if r.check_id == "boom")
    assert boom.status == CheckStatus.ERROR
    assert "kaboom" in (boom.error or "")
    # The other check still ran and produced findings.
    assert rep.findings, "a misbehaving check must not suppress others"


def test_summary_text_mentions_counts():
    log = make_flight_log({})
    rep = diagnose(log, checks=[_Emit()])
    assert "critical" in rep.summary_text
    assert "ArduCopter" in rep.summary_text


def test_no_findings_clean_summary():
    log = make_flight_log({})
    rep = diagnose(log, checks=[_NeedsVibe()])
    assert rep.findings == []
    assert "No issues detected" in rep.summary_text
