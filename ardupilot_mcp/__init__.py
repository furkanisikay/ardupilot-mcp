"""Deterministic-grounded MCP server for diagnosing ArduPilot DataFlash logs."""

from __future__ import annotations

__version__ = "0.1.2"

# Importing the checks package registers every check via decorator side effects.
from . import checks as checks  # noqa: E402,F401
from .flight_log import FlightLog, LogMeta
from .orchestrator import diagnose
from .parser import parse_bin

__all__ = ["FlightLog", "LogMeta", "diagnose", "parse_bin", "checks", "__version__"]
