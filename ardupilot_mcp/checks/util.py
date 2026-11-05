# SPDX-License-Identifier: AGPL-3.0-only
"""Shared numeric helpers for checks.

Small, pure, independently tested. Checks compose these instead of re-deriving
threshold/interval logic, which keeps each check short and consistent.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..model import Evidence, Finding, Severity


@dataclass
class Interval:
    start_s: float
    end_s: float
    peak: float

    @property
    def duration_s(self) -> float:
        return max(0.0, self.end_s - self.start_s)


def intervals_above(
    times: np.ndarray,
    values: np.ndarray,
    threshold: float,
    min_duration_s: float = 0.0,
) -> list[Interval]:
    """Contiguous windows where ``values`` exceed ``threshold``.

    Times and values must be aligned and the same length. Returns intervals whose
    duration is at least ``min_duration_s``. A single sample above threshold yields
    a zero-duration interval (kept only if ``min_duration_s == 0``).
    """
    if times.size == 0 or values.size == 0 or times.size != values.size:
        return []
    mask = values > threshold
    out: list[Interval] = []
    i = 0
    n = mask.size
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        peak = values[i]
        while j + 1 < n and mask[j + 1]:
            j += 1
            peak = max(peak, values[j])
        start_t = float(times[i])
        end_t = float(times[j])
        iv = Interval(start_s=start_t, end_s=end_t, peak=float(peak))
        if iv.duration_s >= min_duration_s:
            out.append(iv)
        i = j + 1
    return out


def longest_run_above(times: np.ndarray, vals: np.ndarray, threshold: float) -> float:
    """Longest time ``vals`` stayed above ``threshold``, counting each sample as one period.

    A run of n samples spans n sample periods (10 checks at 10 Hz is the firmware's 1 s), so
    one median sample spacing is added to the first-to-last-sample time. Samples dropped by a
    mask (on the ground, another core primary) end a run rather than join two.
    """
    if times.size == 0:
        return 0.0
    dt = float(np.median(np.diff(times))) if times.size > 1 else 0.0
    idx = np.flatnonzero(vals > threshold)
    if idx.size == 0:
        return 0.0
    t = times[idx]
    # A run breaks where a sample in between was not above, or where time jumps.
    breaks = np.flatnonzero((np.diff(idx) > 1) | (np.diff(t) > 3 * dt)) + 1
    bounds = np.concatenate(([0], breaks, [idx.size]))
    longest = max(float(t[b - 1] - t[a]) + dt for a, b in zip(bounds[:-1], bounds[1:], strict=True))
    return round(longest, 6)  # 5 samples at 5 Hz is 1.0 s, not 0.9999999


def fraction_at_or_above(values: np.ndarray, threshold: float) -> float:
    """Fraction of samples at or above threshold (0..1)."""
    if values.size == 0:
        return 0.0
    return float(np.count_nonzero(values >= threshold)) / float(values.size)


def fraction_above(values: np.ndarray, threshold: float) -> float:
    """Fraction of samples strictly above threshold (0..1)."""
    if values.size == 0:
        return 0.0
    return float(np.count_nonzero(values > threshold)) / float(values.size)


def safe_max(values: np.ndarray) -> float | None:
    return float(np.max(values)) if values.size else None


def safe_min(values: np.ndarray) -> float | None:
    return float(np.min(values)) if values.size else None


def percentile(values: np.ndarray, q: float) -> float | None:
    return float(np.percentile(values, q)) if values.size else None


def make_finding(
    check_id: str,
    severity: Severity,
    title: str,
    explanation: str,
    *,
    recommendation: str | None = None,
    time_start_s: float | None = None,
    time_end_s: float | None = None,
    message_types: list[str] | None = None,
    samples: dict[str, float] | None = None,
    detail: str | None = None,
) -> Finding:
    """Construct a Finding with its Evidence in one call."""
    return Finding(
        check_id=check_id,
        severity=severity,
        title=title,
        explanation=explanation,
        recommendation=recommendation,
        evidence=Evidence(
            time_start_s=time_start_s,
            time_end_s=time_end_s,
            message_types=message_types or [],
            samples=samples or {},
            detail=detail,
        ),
    )
