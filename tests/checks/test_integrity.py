"""Tests for the log integrity check."""

from __future__ import annotations

from ardupilot_mcp.checks.integrity import IntegrityCheck
from ardupilot_mcp.flight_log import FlightLog, LogMeta
from ardupilot_mcp.model import CheckStatus, LogIntegrity, Severity
from tests.helpers import make_flight_log


def test_always_runs_even_on_empty_log():
    # requires=set() -> the check is never skipped, even with no messages.
    log = make_flight_log({})
    result = IntegrityCheck().execute(log)
    assert result.status == CheckStatus.RAN


def test_ok_integrity_no_findings():
    # Default integrity is OK -> no finding.
    log = make_flight_log({})
    findings = IntegrityCheck().run(log)
    assert findings == []


def test_ok_integrity_with_data_no_findings():
    # A clean log with real messages still produces nothing from this check.
    log = make_flight_log(
        {"ATT": [{"TimeUS": 0, "Roll": 1.0}, {"TimeUS": 1_000_000, "Roll": 2.0}]},
        integrity=LogIntegrity.OK,
    )
    findings = IntegrityCheck().run(log)
    assert findings == []


def test_truncated_emits_single_warn():
    log = make_flight_log({}, integrity=LogIntegrity.TRUNCATED)
    findings = IntegrityCheck().run(log)
    assert len(findings) == 1
    f = findings[0]
    assert f.severity == Severity.WARN
    assert f.title == "Log is truncated"
    assert f.evidence.samples["truncated"] == 1.0
    assert f.recommendation is not None


def test_truncated_surfaces_integrity_detail():
    # The parser's detail string must be passed through to evidence.detail.
    meta = LogMeta(
        integrity=LogIntegrity.TRUNCATED,
        integrity_detail="stream ended at offset 12345 with no FMT terminator",
    )
    log = FlightLog(messages={}, meta=meta, path="synthetic.bin")
    findings = IntegrityCheck().run(log)
    assert len(findings) == 1
    assert findings[0].evidence.detail == "stream ended at offset 12345 with no FMT terminator"


def test_partial_emits_single_warn():
    log = make_flight_log({}, integrity=LogIntegrity.PARTIAL)
    findings = IntegrityCheck().run(log)
    assert len(findings) == 1
    f = findings[0]
    assert f.severity == Severity.WARN
    assert f.title == "Log parsed with errors"
    assert f.evidence.samples["partial"] == 2.0


def test_partial_surfaces_integrity_detail():
    meta = LogMeta(
        integrity=LogIntegrity.PARTIAL,
        integrity_detail="3 undecodable messages dropped",
    )
    log = FlightLog(messages={}, meta=meta, path="synthetic.bin")
    findings = IntegrityCheck().run(log)
    assert findings[0].evidence.detail == "3 undecodable messages dropped"


def test_findings_carry_check_id():
    log = make_flight_log({}, integrity=LogIntegrity.TRUNCATED)
    findings = IntegrityCheck().run(log)
    assert findings[0].check_id == "integrity"


def test_check_is_registered():
    from ardupilot_mcp.checks import get_check

    check = get_check("integrity")
    assert check is not None
    assert check.title == "Log integrity"
    assert check.category == "integrity"
    assert check.requires == set()
