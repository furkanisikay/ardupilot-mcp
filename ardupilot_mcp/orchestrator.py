"""Diagnosis orchestrator: run the check registry over a FlightLog.

This is the reliable backbone. It runs every registered check (with skip/error
isolation), aggregates findings, ranks them by severity then time, and produces
a deterministic ``summary_text`` skeleton. The LLM enriches that summary; it
never invents findings the engine did not produce.
"""

from __future__ import annotations

from . import ardupilot_docs as docs
from .checks.base import Check, get_checks
from .dedupe import merge_duplicates
from .flight_log import FlightLog
from .logging_advice import logging_advice
from .model import (
    CheckStatus,
    DiagnosisReport,
    Finding,
    IncidentTimeline,
    Reference,
    Severity,
    SkippedCheck,
)
from .timeline import build_timeline

_GUIDANCE = (
    "Ground your explanation in the cited references and the firmware version in log_summary "
    "(behaviour and parameter meanings can differ by version). Call the vehicle_profile tool for "
    "the airframe's physical configuration (frame, motor count, battery cells, hover throttle, power "
    "margin) and weave in any real-world details the user gives (weight, prop/motor size, wind). "
    "Report only what the deterministic findings show; cite the doc links rather than inventing causes. "
    "When `incident` is set, start from its onset (what went wrong first) and treat findings flagged "
    "after_impact as consequences of the crash; say what preceded what, not what caused what. "
    "A finding's `related` list names the other log records of the same event (ERR, failsafe mode "
    "change, firmware message): they corroborate it, they are not separate problems."
)


def _finding_sort_key(f: Finding) -> tuple[bool, int, float]:
    # Consequences of a crash last; then most severe first; then earliest evidence first.
    t = f.evidence.time_start_s if f.evidence.time_start_s is not None else float("inf")
    return (f.after_impact, -f.severity.rank, t)


def diagnose(log: FlightLog, checks: list[Check] | None = None) -> DiagnosisReport:
    """Run all checks and assemble a DiagnosisReport."""
    checks = checks if checks is not None else get_checks()

    results = [c.execute(log) for c in checks]

    findings: list[Finding] = []
    for r in results:
        r.finding_count = len(r.findings)
        findings.extend(r.findings)
        # Each finding is reported once, in the top-level list: duplicating it in
        # results[] doubled the payload (and the LLM's token bill) for no information.
        r.findings = []
    # One event, one finding: ERR/MODE/firmware echoes join the finding that measured it.
    findings = merge_duplicates(findings, log.params)
    # Flags after-impact findings (it must run before sorting).
    timeline = build_timeline(log, findings)
    findings.sort(key=_finding_sort_key)
    incident = None
    if timeline.onset_s is not None or timeline.impact_s is not None or timeline.conditions:
        incident = timeline.model_copy(update={"events": []})

    # Attach authoritative ArduPilot doc references to each finding (vehicle-aware).
    kind = log.meta.vehicle_kind
    for f in findings:
        if not f.references:
            f.references = docs.references_for(f.check_id, kind)
    version_ref = docs.version_reference(log.meta.firmware_version, kind)
    report_refs = _dedupe_refs(
        [ref for f in findings for ref in f.references]
        + docs.baseline_references(kind)
        + ([version_ref] if version_ref else [])
    )

    skipped = [
        SkippedCheck(check_id=r.check_id, title=r.title, reason=r.skipped_reason or "")
        for r in results
        if r.status == CheckStatus.SKIPPED
    ]

    crit = sum(1 for f in findings if f.severity == Severity.CRITICAL)
    warn = sum(1 for f in findings if f.severity == Severity.WARN)
    info = sum(1 for f in findings if f.severity == Severity.INFO)

    return DiagnosisReport(
        log_summary=log.summary(),
        findings=findings,
        results=results,
        checks_skipped=skipped,
        critical_count=crit,
        warn_count=warn,
        info_count=info,
        summary_text=_build_summary(log, findings, results, crit, warn, info, len(skipped), incident),
        references=report_refs,
        guidance=_GUIDANCE,
        incident=incident,
        logging_advice=logging_advice(log),
    )


def _dedupe_refs(refs: list[Reference]) -> list[Reference]:
    seen: set[str] = set()
    out: list[Reference] = []
    for r in refs:
        if r.url not in seen:
            seen.add(r.url)
            out.append(r)
    return out


def _build_summary(
    log: FlightLog,
    findings: list[Finding],
    results: list,
    crit: int,
    warn: int,
    info: int,
    n_skipped: int,
    incident: IncidentTimeline | None = None,
) -> str:
    s = log.summary()
    lines: list[str] = []
    head = s.vehicle_type or "Vehicle"
    if s.firmware_version:
        head += f" ({s.firmware_version})"
    dur = f"{s.duration_s:.0f}s" if s.duration_s is not None else "unknown duration"
    lines.append(f"{head}, {dur} log.")

    if s.integrity.value != "ok":
        lines.append(
            f"Log integrity: {s.integrity.value} ({s.integrity_detail or 'see log_integrity finding'})."
        )

    if incident is not None:
        if incident.onset_s is not None:
            lines.append(f"First anomaly while armed: {incident.onset_basis} @ {incident.onset_s:.0f}s.")
        if incident.conditions:
            more = f" (+{len(incident.conditions) - 3} more)" if len(incident.conditions) > 3 else ""
            lines.append(
                "Present throughout (context, not the onset): "
                + "; ".join(incident.conditions[:3])
                + more
                + "."
            )
        if incident.impact_s is not None:
            n_after = sum(1 for f in findings if f.after_impact)
            tail = f"; {n_after} finding(s) start after it and are listed last" if n_after else ""
            lines.append(f"Impact: {incident.impact_basis} @ {incident.impact_s:.0f}s{tail}.")

    if not findings:
        lines.append("No issues detected by the deterministic checks.")
    else:
        lines.append(f"{crit} critical, {warn} warning, {info} info finding(s).")
        top = findings[: min(3, len(findings))]
        for f in top:
            when = f" @ {f.evidence.time_start_s:.0f}s" if f.evidence.time_start_s is not None else ""
            lines.append(f"- [{f.severity.value}] {f.title}{when}")

    ran = sum(1 for r in results if r.status == CheckStatus.RAN)
    errored = sum(1 for r in results if r.status == CheckStatus.ERROR)
    tail = f"{ran} checks ran"
    if n_skipped:
        tail += f", {n_skipped} skipped (data not logged)"
    if errored:
        tail += f", {errored} errored"
    lines.append(tail + ".")
    return "\n".join(lines)
