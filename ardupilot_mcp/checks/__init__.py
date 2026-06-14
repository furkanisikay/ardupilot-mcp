"""Check framework + catalogue.

Importing this package registers every check via ``@register_check`` side effects.
Adding a check = create a module here and add it to ``_CHECK_MODULES`` below.
"""

from __future__ import annotations

from .base import Check, clear_registry, get_check, get_checks, register_check

# Check modules to import for their registration side effects. Order here is the
# order checks appear in the catalogue.
_CHECK_MODULES = [
    "integrity",
    "events",
    "ekf",
    "vibration",
    "power",
    "gps",
    "compass",
    "attitude",
    "motors",
    "rcin",
    "timing",
    # configuration / setup checks (params, calibration, sensors, startup messages)
    "config",
    "calibration",
    "sensors",
    "prearm",
    "param_audit",
]


def load_checks() -> None:
    """Import all check modules so they register. Idempotent and fault-tolerant.

    A check module that fails to import (not yet implemented, or a broken
    third-party check) is skipped with a warning rather than taking down the
    whole server. This also keeps partial/parallel builds importable.
    """
    import importlib
    import sys

    for name in _CHECK_MODULES:
        try:
            importlib.import_module(f"{__name__}.{name}")
        except Exception as exc:  # noqa: BLE001 - never let one check break import
            print(f"ardupilot_mcp: skipping check module {name!r}: {exc}", file=sys.stderr)


load_checks()

__all__ = ["Check", "register_check", "get_checks", "get_check", "clear_registry", "load_checks"]
