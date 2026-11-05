"""Tests for the loss-of-control consequence pass."""

from __future__ import annotations

from ardupilot_mcp.checks.util import make_finding
from ardupilot_mcp.consequences import downgrade_rejections_after_loss_of_control
from ardupilot_mcp.model import Severity


def _att(t0, t1):
    return make_finding(
        "attitude", Severity.CRITICAL, "Attitude tracking failure (roll)", "x", time_start_s=t0, time_end_s=t1
    )


def _ekf(t0):
    return make_finding(
        "ekf", Severity.CRITICAL, "EKF velocity variance (SV)", "x.", time_start_s=t0, time_end_s=t0 + 2
    )


def test_an_ekf_rejection_inside_a_loss_of_control_is_a_warning_that_says_so():
    ekf = _ekf(12.0)
    downgrade_rejections_after_loss_of_control([_att(10.0, 15.0), ekf])
    assert ekf.severity == Severity.WARN
    assert ekf.evidence.samples["after_loss_of_control_s"] == 2.0
    assert "cannot have caused it" in ekf.explanation


def test_an_ekf_rejection_before_the_loss_of_control_stays_critical():
    ekf = _ekf(8.0)
    downgrade_rejections_after_loss_of_control([_att(10.0, 15.0), ekf])
    assert ekf.severity == Severity.CRITICAL


def test_an_ekf_rejection_long_after_a_recovered_loss_of_control_stays_critical():
    ekf = _ekf(40.0)
    downgrade_rejections_after_loss_of_control([_att(10.0, 15.0), ekf])
    assert ekf.severity == Severity.CRITICAL


def test_other_checks_are_left_alone():
    gps = make_finding("gps", Severity.CRITICAL, "GPS lost 3D fix", "x", time_start_s=12.0)
    downgrade_rejections_after_loss_of_control([_att(10.0, 15.0), gps])
    assert gps.severity == Severity.CRITICAL
