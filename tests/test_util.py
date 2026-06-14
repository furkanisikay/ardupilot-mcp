"""Tests for the shared check utilities."""

from __future__ import annotations

import numpy as np

from ardupilot_mcp.checks.util import (
    fraction_above,
    intervals_above,
    make_finding,
    percentile,
    safe_max,
    safe_min,
)
from ardupilot_mcp.model import Severity


def test_intervals_above_basic():
    t = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
    v = np.array([0.0, 5.0, 6.0, 0.0, 7.0])
    ivs = intervals_above(t, v, threshold=4.0)
    assert len(ivs) == 2
    assert ivs[0].start_s == 1.0 and ivs[0].end_s == 2.0 and ivs[0].peak == 6.0
    assert ivs[1].start_s == 4.0 and ivs[1].peak == 7.0


def test_intervals_above_min_duration_filters():
    t = np.array([0.0, 1.0, 2.0, 3.0])
    v = np.array([9.0, 0.0, 9.0, 9.0])
    ivs = intervals_above(t, v, threshold=1.0, min_duration_s=0.5)
    # first interval is a single sample (duration 0) -> filtered; second spans 2..3 (duration 1.0) -> kept
    assert len(ivs) == 1
    assert ivs[0].start_s == 2.0 and ivs[0].end_s == 3.0


def test_intervals_above_empty_and_mismatched():
    assert intervals_above(np.array([]), np.array([]), 1.0) == []
    assert intervals_above(np.array([0.0, 1.0]), np.array([1.0]), 1.0) == []


def test_fraction_above():
    v = np.array([1.0, 2.0, 3.0, 4.0])
    assert fraction_above(v, 2.0) == 0.5
    assert fraction_above(np.array([]), 1.0) == 0.0


def test_safe_minmax_percentile():
    v = np.array([3.0, 1.0, 2.0])
    assert safe_max(v) == 3.0
    assert safe_min(v) == 1.0
    assert percentile(v, 50) == 2.0
    assert safe_max(np.array([])) is None
    assert percentile(np.array([]), 50) is None


def test_make_finding_packs_evidence():
    f = make_finding(
        "x",
        Severity.WARN,
        "title",
        "explanation",
        recommendation="do thing",
        time_start_s=1.0,
        time_end_s=2.0,
        message_types=["ATT"],
        samples={"k": 1.5},
        detail="d",
    )
    assert f.check_id == "x" and f.severity == Severity.WARN
    assert f.evidence.time_start_s == 1.0 and f.evidence.samples["k"] == 1.5
    assert f.evidence.message_types == ["ATT"] and f.evidence.detail == "d"
