# SPDX-License-Identifier: AGPL-3.0-only
"""Estimator rejections that began while the aircraft was already out of control.

A tumbling aircraft makes the EKF's innovations jump: the IMU sees rates and accelerations
far outside normal flight and the GPS velocity no longer matches the prediction, so the
velocity, position and compass test ratios cross 1.0. An EKF rejection that begins inside
an attitude-tracking failure (or within ``SLACK_S`` of its end) began after the loss of
control, so it cannot have caused it. It stays in the report as a WARN that says so,
instead of a CRITICAL that reads as a second, independent fault.

Design choice; the attitude failure is the attitude check's CRITICAL (for Copter, the
firmware's own loss-of-control line of 30 deg held for 1 s).
"""

from __future__ import annotations

from .model import Finding, Severity

# Design choice: an EKF rejection up to this long after a loss of control ends still
# belongs to it (the test ratios are filtered and lag the motion).
SLACK_S = 2.0


def downgrade_rejections_after_loss_of_control(findings: list[Finding]) -> list[Finding]:
    """EKF CRITICALs that began during an attitude CRITICAL become WARNs that say so."""
    windows = [
        (f.evidence.time_start_s, f.evidence.time_end_s or f.evidence.time_start_s)
        for f in findings
        if f.check_id == "attitude"
        and f.severity == Severity.CRITICAL
        and f.evidence.time_start_s is not None
    ]
    if not windows:
        return findings
    for f in findings:
        t = f.evidence.time_start_s
        if f.check_id != "ekf" or f.severity != Severity.CRITICAL or t is None:
            continue
        hit = [(a, b) for a, b in windows if a < t <= b + SLACK_S]
        if not hit:
            continue
        onset = min(a for a, _ in hit)
        f.severity = Severity.WARN
        f.explanation += (
            f" It began {t - onset:.1f} s after the loss of attitude control at t={onset:.1f}s, so it cannot have "
            "caused it: a tumbling aircraft makes the estimator's innovations jump."
        )
        f.evidence.samples["after_loss_of_control_s"] = round(t - onset, 1)
    return findings
