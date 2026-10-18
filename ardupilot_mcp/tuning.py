# SPDX-License-Identifier: AGPL-3.0-only
"""Tuning recommendations (advisory only — never writes parameters).

This module is *not* a Check: it does not subclass :class:`Check` and is not
registered with the check engine. It is consumed directly by the
``recommend_tuning`` MCP tool, which wraps the returned list in a
``TuningReport``.

Three independent analyses, each gated by ``area``:

* **notch** — finds the motor-noise frequency in the batch sampler, a high-rate
  IMU, the in-flight FFT or ESC RPM telemetry and suggests harmonic-notch
  (``INS_HNTCH_*``) parameters (see :mod:`ardupilot_mcp.notch`).
* **pid** — measures RMS attitude-tracking error (actual vs desired) on roll and
  pitch from ``ATT`` and, when tracking is poor, advises reviewing the rate PIDs.
* **autotune** — reports the presence of autotune data (``AUTOTUNE_*`` params or
  an ``ATUN`` message) so a human knows what to inspect.

Everything is deterministic and grounded: every recommendation cites the
concrete numbers (frequencies, RMS errors, sample counts) that produced it.
"""

from __future__ import annotations

import numpy as np

from .flight_log import FlightLog
from .model import Evidence, TuningArea, TuningRecommendation
from .notch import analyze_notch

# --- PID (attitude tracking) -------------------------------------------------
# RMS attitude tracking error (deg) above which tracking is considered poor
# enough to advise a rate-PID review. Conservative; this is advisory only.
PID_RMS_WARN_DEG = 5.0


def analyze_tuning(log: FlightLog, area: str | None = None) -> list[TuningRecommendation]:
    """Produce advisory tuning recommendations for ``log``.

    ``area`` filters which analyses run: ``None`` runs all; otherwise one of
    ``"notch"``, ``"pid"``, ``"autotune"``. Returns a (possibly empty) list of
    :class:`TuningRecommendation`. Never mutates the log and never applies any
    parameter — these are recommendations only.
    """
    want = None if area is None else str(area).strip().lower()
    out: list[TuningRecommendation] = []
    if want in (None, "notch"):
        rec = analyze_notch(log)
        if rec is not None:
            out.append(rec)
    if want in (None, "pid"):
        out.extend(_analyze_pid(log))
    if want in (None, "autotune"):
        rec = _analyze_autotune(log)
        if rec is not None:
            out.append(rec)
    return out


def _analyze_pid(log: FlightLog) -> list[TuningRecommendation]:
    """Advise on rate-PID review when attitude tracking error is high."""
    if not log.has("ATT"):
        return []
    out: list[TuningRecommendation] = []
    # (actual field, desired field, axis label, ATC param family)
    axes = (
        ("Roll", "DesRoll", "roll", "ATC_RAT_RLL"),
        ("Pitch", "DesPitch", "pitch", "ATC_RAT_PIT"),
    )
    for actual_f, des_f, label, fam in axes:
        t_act, act = log.series("ATT", actual_f)
        t_des, des = log.series("ATT", des_f)
        n = min(act.size, des.size)
        if n == 0:
            continue
        # Records carry both fields aligned, so equal-length slices are aligned.
        err = act[:n] - des[:n]
        flying = log.flying_mask(t_act[:n])  # ground time is not tracking performance
        if flying is not None:
            err = err[flying]
            if err.size == 0:
                continue
        rms = float(np.sqrt(np.mean(err**2)))
        peak = float(np.max(np.abs(err)))
        if rms <= PID_RMS_WARN_DEG:
            continue
        t0 = float(t_act[0]) if t_act.size else None
        t1 = float(t_act[min(n, t_act.size) - 1]) if t_act.size else None
        explanation = (
            f"The {label} axis tracks its target poorly: RMS error {rms:.1f} deg "
            f"(peak {peak:.1f} deg) between desired and actual over {n} samples. "
            f"Sustained lag like this usually means the rate loop is under-tuned "
            f"(sluggish tracking) — review {fam}_P and {fam}_D, or run autotune. "
            "This is advisory: confirm against vibration, motor saturation (RCOU) "
            "and power before changing gains."
        )
        evidence = Evidence(
            time_start_s=t0,
            time_end_s=t1,
            message_types=["ATT"],
            samples={
                "rms_error_deg": round(rms, 2),
                "peak_error_deg": round(peak, 2),
                "samples": float(n),
            },
            detail=f"{label} desired-vs-actual tracking error.",
        )
        out.append(
            TuningRecommendation(
                area=TuningArea.PID,
                title=f"Poor {label} tracking ({rms:.1f} deg RMS)",
                explanation=explanation,
                suggested_params={},  # qualitative advice only
                evidence=evidence,
                confidence="low",
            )
        )
    return out


def _analyze_autotune(log: FlightLog) -> TuningRecommendation | None:
    """Report presence of autotune data so a human knows what to inspect."""
    autotune_params = sorted(k for k in log.params if k.startswith("AUTOTUNE_"))
    has_atun = log.has("ATUN")
    if not autotune_params and not has_atun:
        return None

    parts: list[str] = []
    samples: dict[str, float] = {}
    if has_atun:
        n = log.count("ATUN")
        parts.append(f"{n} ATUN message(s)")
        samples["atun_messages"] = float(n)
    if autotune_params:
        parts.append(f"{len(autotune_params)} AUTOTUNE_* parameter(s)")
        samples["autotune_params"] = float(len(autotune_params))
    present = " and ".join(parts)

    explanation = (
        f"This log contains autotune data ({present}). Autotune derives rate-PID "
        "gains in flight; inspect the resulting ATC_RAT_RLL/PIT/YAW_P, _I and _D "
        "values for plausibility, confirm the run completed without an aborting "
        "twitch, and verify the new gains were saved. No action is recommended "
        "here — this is an informational pointer to data worth reviewing."
    )
    evidence = Evidence(
        message_types=["ATUN"] if has_atun else [],
        samples=samples,
        detail=(
            "AUTOTUNE_* params: " + ", ".join(autotune_params) if autotune_params else "ATUN message present."
        ),
    )
    return TuningRecommendation(
        area=TuningArea.AUTOTUNE,
        title="Autotune data present",
        explanation=explanation,
        suggested_params={},
        evidence=evidence,
        confidence="medium",
    )
