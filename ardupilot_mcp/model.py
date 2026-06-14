"""Core data model for the ArduPilot log-diagnosis MCP server.

All types are Pydantic models so MCP tools can emit validated ``structuredContent``
and auto-generate output schemas. The deterministic check engine is the single
source of truth; the LLM only narrates these structures.
"""

from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, Field


class Severity(str, Enum):
    """Finding severity, ordered INFO < WARN < CRITICAL."""

    INFO = "info"
    WARN = "warn"
    CRITICAL = "critical"

    @property
    def rank(self) -> int:
        return {"info": 0, "warn": 1, "critical": 2}[self.value]


class Evidence(BaseModel):
    """Deterministic, quotable evidence backing a finding.

    Everything here comes straight from the log so a human can verify the claim.
    """

    time_start_s: float | None = Field(
        default=None, description="Flight-relative start time of the evidence window (s)."
    )
    time_end_s: float | None = Field(
        default=None, description="Flight-relative end time of the evidence window (s)."
    )
    message_types: list[str] = Field(
        default_factory=list, description="DataFlash message types the evidence was drawn from."
    )
    samples: dict[str, float] = Field(
        default_factory=dict,
        description="Named numeric evidence, e.g. {'max_VibeX': 62.0, 'clip0': 38}.",
    )
    detail: str | None = Field(default=None, description="Optional extra free-text context for the evidence.")


class Reference(BaseModel):
    """A pointer to authoritative ArduPilot documentation for a finding."""

    title: str
    url: str


class Finding(BaseModel):
    """A single deterministic diagnostic conclusion."""

    check_id: str = Field(description="Id of the check that produced this finding.")
    severity: Severity
    title: str = Field(description="Short human-readable headline.")
    explanation: str = Field(
        description="Deterministic, plain-language explanation of what was detected and why it matters."
    )
    evidence: Evidence = Field(default_factory=Evidence)
    recommendation: str | None = Field(
        default=None,
        description="Optional advisory next step (e.g. a parameter to inspect). Never an applied change.",
    )
    references: list[Reference] = Field(
        default_factory=list,
        description="Official ArduPilot docs for this issue — the LLM should read/cite them, "
        "consulting the firmware version in log_summary for version-specific behaviour.",
    )


class CheckStatus(str, Enum):
    RAN = "ran"
    SKIPPED = "skipped"  # required data not present in the log
    ERROR = "error"  # the check raised; never aborts the whole diagnosis


class CheckResult(BaseModel):
    """Outcome of running one check, including the "couldn't run" cases."""

    check_id: str
    title: str
    status: CheckStatus
    skipped_reason: str | None = Field(
        default=None, description="Why the check was skipped, e.g. 'VIBE not logged'."
    )
    error: str | None = Field(default=None, description="Error message if the check raised.")
    findings: list[Finding] = Field(default_factory=list)


class LogIntegrity(str, Enum):
    OK = "ok"
    TRUNCATED = "truncated"  # log ends abruptly (common after a crash)
    PARTIAL = "partial"  # parsed but errors were encountered mid-stream


class LogSummary(BaseModel):
    """Cheap orientation metadata about a parsed log."""

    path: str
    vehicle_type: str | None = None
    firmware_version: str | None = None
    board: str | None = None
    duration_s: float | None = None
    start_time_s: float | None = None
    available_messages: list[str] = Field(default_factory=list)
    message_counts: dict[str, int] = Field(default_factory=dict)
    flight_modes: list[str] = Field(default_factory=list)
    max_altitude_m: float | None = None
    integrity: LogIntegrity = LogIntegrity.OK
    integrity_detail: str | None = None


class SkippedCheck(BaseModel):
    check_id: str
    title: str
    reason: str


class DiagnosisReport(BaseModel):
    """Top-level structured output of ``analyze_log`` — the reliable backbone."""

    log_summary: LogSummary
    findings: list[Finding] = Field(
        default_factory=list, description="All findings, sorted by severity (most severe first)."
    )
    results: list[CheckResult] = Field(
        default_factory=list, description="Per-check outcomes, including skipped/errored checks."
    )
    checks_skipped: list[SkippedCheck] = Field(default_factory=list)
    critical_count: int = 0
    warn_count: int = 0
    info_count: int = 0
    summary_text: str = Field(
        default="",
        description="Deterministic summary skeleton. The LLM enriches this; it never invents findings.",
    )
    references: list[Reference] = Field(
        default_factory=list, description="De-duplicated documentation links across all findings."
    )
    guidance: str = Field(
        default="",
        description="How the LLM should use this report: read the cited docs for the log's firmware "
        "version, factor in the vehicle_profile (frame, motors, battery, power margin) and any "
        "physical details (weight, prop size) the user provides, then explain and cite sources.",
    )


# ---- Auxiliary tool output models -------------------------------------------------


class EventRecord(BaseModel):
    time_s: float | None = None
    kind: str = Field(description="Event family: ERR, MODE, EV, MSG, FAILSAFE.")
    name: str = Field(description="Decoded human-readable event, e.g. 'GPS glitch cleared'.")
    raw: dict[str, float | int | str] = Field(default_factory=dict)


class EventTimeline(BaseModel):
    path: str
    events: list[EventRecord] = Field(default_factory=list)
    total: int = 0


class TimeseriesResult(BaseModel):
    path: str
    message_type: str
    fields: list[str]
    times_s: list[float] = Field(default_factory=list)
    series: dict[str, list[float]] = Field(default_factory=dict)
    sample_count: int = 0
    downsampled: bool = False
    note: str | None = None


class ParamResult(BaseModel):
    path: str
    params: dict[str, float] = Field(default_factory=dict)
    matched: int = Field(default=0, description="Params returned after the name filter.")
    total: int = Field(default=0, description="Total params available in the source (before filtering).")
    source: str = Field(default="log", description="'log' (PARM dump) or 'file' (exported .param).")
    metadata_url: str | None = Field(
        default=None,
        description="Version-specific parameter definitions (apm.pdef.xml) — meanings, defaults, ranges. "
        "The LLM should consult this to interpret values.",
    )
    note: str | None = Field(
        default=None, description="Completeness caveat (e.g. truncated log -> snapshot may be partial)."
    )


class TuningArea(str, Enum):
    NOTCH = "notch"
    PID = "pid"
    AUTOTUNE = "autotune"


class TuningRecommendation(BaseModel):
    area: TuningArea
    title: str
    explanation: str
    suggested_params: dict[str, float] = Field(
        default_factory=dict, description="Advisory parameter values. NOT applied — recommendation only."
    )
    evidence: Evidence = Field(default_factory=Evidence)
    confidence: str = Field(default="medium", description="low | medium | high")
    references: list[Reference] = Field(default_factory=list)


class TuningReport(BaseModel):
    path: str
    recommendations: list[TuningRecommendation] = Field(default_factory=list)
    notes: list[str] = Field(default_factory=list)


class CheckInfo(BaseModel):
    check_id: str
    title: str
    requires: list[str]
    category: str
    description: str


class CheckCatalog(BaseModel):
    checks: list[CheckInfo] = Field(default_factory=list)
    total: int = 0


class VehicleProfile(BaseModel):
    """Physical / architecture profile of the aircraft, inferred from the log.

    Gives the LLM the concrete configuration to reason about (and to combine with
    real-world dimensions the user supplies: weight, prop size, motor KV, ...).
    """

    path: str
    vehicle_type: str | None = None
    vehicle_kind: str | None = None
    firmware_version: str | None = None
    frame_class: int | None = None
    frame_class_name: str | None = None
    frame_type: int | None = None
    frame_type_name: str | None = None
    motor_count: int | None = Field(default=None, description="Number of lift motors implied by the frame.")
    battery_cells: int | None = Field(default=None, description="Estimated LiPo cell count (S).")
    battery_capacity_mah: float | None = None
    nominal_voltage_v: float | None = None
    max_voltage_v: float | None = None
    hover_throttle: float | None = Field(
        default=None, description="Throttle (0..1) needed to hover; from MOT_THST_HOVER or measured."
    )
    power_margin_pct: float | None = Field(
        default=None, description="Spare throttle above hover (100*(1-hover)); low = underpowered."
    )
    thrust_to_weight_estimate: float | None = Field(
        default=None, description="Rough max-thrust/weight (~1/hover_throttle)."
    )
    assessment: list[str] = Field(
        default_factory=list, description="Plain-language physical observations (e.g. 'underpowered')."
    )
    notes: list[str] = Field(default_factory=list)
    references: list[Reference] = Field(default_factory=list)
