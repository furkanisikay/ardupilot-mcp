"""Check protocol and the extensible check registry.

A *check* is a small, independently-testable diagnostic unit. It declares the
DataFlash message types it needs (``requires``); the orchestrator skips it
cleanly when those are absent and isolates any exception it raises so one bad
check never aborts a whole diagnosis.

Adding a check = drop a module in :mod:`ardupilot_mcp.checks` that defines a
``Check`` subclass decorated with ``@register_check``. This registry is the
"framework" backbone that makes the catalogue community-extensible.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..flight_log import FlightLog
from ..model import CheckInfo, CheckResult, CheckStatus, Finding


class Check(ABC):
    #: Stable, unique identifier (snake_case), e.g. ``"vibration"``.
    id: str = ""
    #: Short human-readable title.
    title: str = ""
    #: Grouping for the catalogue, e.g. ``"power"``, ``"estimator"``, ``"tuning"``.
    category: str = "general"
    #: DataFlash message types this check needs to run at all.
    requires: set[str] = set()
    #: Vehicle kinds this check applies to (copter/heli/plane/rover/sub/...).
    #: ``None`` means all vehicles. A check is skipped on a *known* vehicle kind
    #: outside this set (unknown kinds always run, to avoid false negatives).
    vehicles: set[str] | None = None
    #: One-line description for ``list_checks``.
    description: str = ""

    @abstractmethod
    def run(self, log: FlightLog) -> list[Finding]:
        """Inspect ``log`` and return zero or more findings. Pure and deterministic."""

    # -- orchestration helpers (not overridden by checks) --------------------

    def applicable(self, log: FlightLog) -> tuple[bool, str | None]:
        kind = log.meta.vehicle_kind
        if self.vehicles is not None and kind and kind not in self.vehicles:
            return False, f"not applicable to vehicle kind '{kind}'"
        missing = sorted(m for m in self.requires if not log.has(m))
        if missing:
            return False, f"required message(s) not logged: {', '.join(missing)}"
        return True, None

    def execute(self, log: FlightLog) -> CheckResult:
        """Run with skip/error isolation, always returning a CheckResult."""
        ok, reason = self.applicable(log)
        if not ok:
            return CheckResult(
                check_id=self.id, title=self.title, status=CheckStatus.SKIPPED, skipped_reason=reason
            )
        try:
            findings = self.run(log)
        except Exception as exc:  # noqa: BLE001 - deliberately isolate misbehaving checks
            return CheckResult(
                check_id=self.id,
                title=self.title,
                status=CheckStatus.ERROR,
                error=f"{type(exc).__name__}: {exc}",
            )
        return CheckResult(
            check_id=self.id, title=self.title, status=CheckStatus.RAN, findings=list(findings)
        )

    def info(self) -> CheckInfo:
        return CheckInfo(
            check_id=self.id,
            title=self.title,
            requires=sorted(self.requires),
            category=self.category,
            description=self.description,
        )


# ---- registry ---------------------------------------------------------------

_REGISTRY: dict[str, Check] = {}


def register_check(cls: type[Check]) -> type[Check]:
    """Class decorator: instantiate and register a check (no-arg constructor)."""
    inst = cls()
    if not inst.id:
        raise ValueError(f"check {cls.__name__} must define a non-empty id")
    if inst.id in _REGISTRY:
        raise ValueError(f"duplicate check id: {inst.id!r} ({cls.__name__})")
    _REGISTRY[inst.id] = inst
    return cls


def get_checks() -> list[Check]:
    """All registered checks, in registration order."""
    return list(_REGISTRY.values())


def get_check(check_id: str) -> Check | None:
    return _REGISTRY.get(check_id)


def clear_registry() -> None:
    """Test-only: empty the registry."""
    _REGISTRY.clear()
