"""Test helpers for building synthetic in-memory flight logs.

Checks are tested against these — no real ``.bin`` required. ``build_series``
turns parallel field arrays into per-record dicts with monotonically increasing
``TimeUS`` at a chosen rate, which is how time-series checks (vibration,
attitude, notch) get their input.
"""

from __future__ import annotations

from collections.abc import Sequence

from ardupilot_mcp.flight_log import FlightLog, LogMeta
from ardupilot_mcp.model import LogIntegrity


def build_series(
    t0_s: float,
    rate_hz: float,
    **fields: Sequence[float],
) -> list[dict]:
    """Build N records from parallel field arrays at ``rate_hz`` starting at ``t0_s``.

    All arrays must be the same length N. Each record gets ``TimeUS`` in microseconds.
    """
    names = list(fields)
    if not names:
        return []
    n = len(fields[names[0]])
    for k in names:
        if len(fields[k]) != n:
            raise ValueError(f"field {k!r} length {len(fields[k])} != {n}")
    dt_us = 1_000_000.0 / rate_hz
    out: list[dict] = []
    for i in range(n):
        rec: dict = {"TimeUS": int(round((t0_s * 1_000_000.0) + i * dt_us))}
        for k in names:
            rec[k] = fields[k][i]
        out.append(rec)
    return out


def make_flight_log(
    messages: dict[str, list[dict]] | None = None,
    *,
    params: dict[str, float] | None = None,
    vehicle_type: str | None = "ArduCopter",
    vehicle_kind: str | None = None,
    firmware_version: str | None = "ArduCopter V4.5.7",
    flight_modes: list[str] | None = None,
    max_altitude_m: float | None = None,
    integrity: LogIntegrity = LogIntegrity.OK,
    path: str = "synthetic.bin",
) -> FlightLog:
    """Construct a FlightLog directly, bypassing binary parsing."""
    meta = LogMeta(
        vehicle_type=vehicle_type,
        vehicle_kind=vehicle_kind,
        firmware_version=firmware_version,
        flight_modes=flight_modes or [],
        max_altitude_m=max_altitude_m,
        integrity=integrity,
    )
    return FlightLog(messages=messages or {}, params=params, meta=meta, path=path)
