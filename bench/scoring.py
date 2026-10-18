"""Score diagnosis reports against labelled real flights.

Each label category maps to the checks whose findings would point a reader at that
cause. A report *hits* when its first WARN/CRITICAL finding (report order: before the
impact, most severe first) comes from one of those checks; *top-3* counts a hit in the
first three. A CRITICAL finding from any other check is a *false critical*: on a flight
whose cause was something else, it would send the reader down the wrong path. For
``not_a_fault`` flights every CRITICAL is false. ``pilot_error`` and ``firmware_bug``
have no check that could name them; they only count towards false criticals.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field

from ardupilot_mcp.model import DiagnosisReport, Severity

# Label category -> checks whose findings name that cause.
CATEGORY_CHECKS: dict[str, set[str]] = {
    "vibration": {"vibration"},
    "power_brownout": {"power"},
    "thrust_loss_motor_esc": {"thrust_loss", "motors"},
    "compass_interference": {"compass", "calibration"},
    "gps": {"gps", "sensors"},
    "ekf": {"ekf", "firmware"},
    "rc_failsafe_link": {"rcin", "firmware"},
    "tuning_oscillation": {"attitude"},
    "mechanical_frame": {"motors", "firmware", "attitude"},
    "configuration": {"config", "param_audit", "calibration", "prearm", "sensors"},
    "not_a_fault": set(),
    "pilot_error": set(),
    "firmware_bug": set(),
}
# Categories where no check can name the cause: scored for false criticals only.
UNSCORABLE = {"not_a_fault", "pilot_error", "firmware_bug"}

# The events and firmware checks report many subsystems; their title says which one.
# ERR subsystem names: ardupilot_mcp/ardupilot_meta.ERR_SUBSYSTEMS.
_TITLE_CATEGORIES: tuple[tuple[str, str], ...] = (
    ("EKF", "ekf"),
    ("FAILSAFE_RADIO", "rc_failsafe_link"),
    ("Error: RADIO", "rc_failsafe_link"),
    ("radio failsafe", "rc_failsafe_link"),
    ("FAILSAFE_BATT", "power_brownout"),
    ("battery failsafe", "power_brownout"),
    ("GPS", "gps"),
    ("COMPASS", "compass_interference"),
    ("Compass", "compass_interference"),
    ("THRUST_LOSS", "thrust_loss_motor_esc"),
    ("thrust loss", "thrust_loss_motor_esc"),
    ("Yaw imbalance", "mechanical_frame"),
    ("vibration", "vibration"),
    ("VIBE", "vibration"),
)


def finding_categories(check_id: str, title: str) -> set[str]:
    """Label categories a finding points at."""
    if check_id in ("events", "firmware"):
        return {cat for key, cat in _TITLE_CATEGORIES if key.lower() in title.lower()}
    return {cat for cat, checks in CATEGORY_CHECKS.items() if check_id in checks}


@dataclass
class Score:
    id: str
    category: str
    expected: list[str]
    top_check: str | None
    hit: bool | None  # None = category not scorable
    top3: bool | None
    onset_check: str | None
    false_criticals: list[str] = field(default_factory=list)


def score_report(entry: dict, report: DiagnosisReport) -> Score:
    cat = entry["category"]
    accepted = {cat, *entry.get("secondary", [])}
    ranked = [f for f in report.findings if f.severity in (Severity.WARN, Severity.CRITICAL)]
    top = ranked[0].check_id if ranked else None
    scorable = cat not in UNSCORABLE
    hit = (
        (bool(ranked) and cat in finding_categories(ranked[0].check_id, ranked[0].title))
        if scorable
        else None
    )
    top3 = any(cat in finding_categories(f.check_id, f.title) for f in ranked[:3]) if scorable else None
    false_crit = [
        f.check_id
        for f in report.findings
        if f.severity == Severity.CRITICAL and not (finding_categories(f.check_id, f.title) & accepted)
    ]
    primary = CATEGORY_CHECKS.get(cat, set())
    onset = report.incident.first_anomaly_check_id if report.incident else None
    return Score(
        id=entry["id"],
        category=cat,
        expected=sorted(primary),
        top_check=top,
        hit=hit,
        top3=top3,
        onset_check=onset,
        false_criticals=false_crit,
    )


def summarize(scores: list[Score]) -> dict:
    scored = [s for s in scores if s.hit is not None]
    fc = Counter(c for s in scores for c in s.false_criticals)
    return {
        "logs": len(scores),
        "scored": len(scored),
        "top1_hits": sum(1 for s in scored if s.hit),
        "top3_hits": sum(1 for s in scored if s.top3),
        "logs_with_false_critical": sum(1 for s in scores if s.false_criticals),
        "false_criticals_total": sum(len(s.false_criticals) for s in scores),
        "false_criticals_by_check": dict(fc.most_common()),
    }
