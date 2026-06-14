"""Tuning recommendations (advisory only — never writes parameters).

This module is *not* a Check: it does not subclass :class:`Check` and is not
registered with the check engine. It is consumed directly by the
``recommend_tuning`` MCP tool, which wraps the returned list in a
``TuningReport``.

Three independent analyses, each gated by ``area``:

* **notch** — estimates the dominant motor-noise frequency from the IMU gyro
  via a real FFT and suggests harmonic-notch (``INS_HNTCH_*``) parameters.
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

# --- NOTCH (harmonic notch filter) -------------------------------------------
# Ignore everything below this when searching for the motor-noise peak: low
# frequencies are dominated by airframe motion / control inputs, not motor hum.
NOTCH_MIN_HZ = 10.0
# Search up to this fraction of the sample rate (a conservative slice of Nyquist).
NOTCH_MAX_FS_FRACTION = 0.45
# Minimum IMU sample rate (Hz) needed to resolve a meaningful motor peak. Below
# this the spectrum is too coarse to trust, so we skip rather than guess.
NOTCH_MIN_FS_HZ = 100.0
# Minimum number of IMU samples needed for a usable FFT.
NOTCH_MIN_SAMPLES = 64
# A peak counts as "clear" only if its power is at least this multiple of the
# median in-band power — i.e. it really sticks out of the noise floor.
NOTCH_PEAK_RATIO = 5.0

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
        rec = _analyze_notch(log)
        if rec is not None:
            out.append(rec)
    if want in (None, "pid"):
        out.extend(_analyze_pid(log))
    if want in (None, "autotune"):
        rec = _analyze_autotune(log)
        if rec is not None:
            out.append(rec)
    return out


def _estimate_rate_hz(times: np.ndarray) -> float | None:
    """Sample rate (Hz) as 1 / median(dt), or None if not derivable."""
    if times.size < 2:
        return None
    dt = np.diff(times)
    dt = dt[dt > 0.0]
    if dt.size == 0:
        return None
    median_dt = float(np.median(dt))
    if median_dt <= 0.0:
        return None
    return 1.0 / median_dt


def _dominant_gyro_axis(log: FlightLog) -> tuple[str, np.ndarray] | None:
    """Return (axis_name, values) for the gyro axis with the most AC energy.

    Energy is measured on the mean-removed signal so a constant offset (e.g. a
    yaw bias) does not masquerade as energy.
    """
    best: tuple[str, np.ndarray] | None = None
    best_energy = -1.0
    for axis in ("GyrX", "GyrY", "GyrZ"):
        vals = log.field("IMU", axis)
        if vals.size == 0:
            continue
        ac = vals - float(np.mean(vals))
        energy = float(np.dot(ac, ac))
        if energy > best_energy:
            best_energy = energy
            best = (axis, vals)
    return best


def _analyze_notch(log: FlightLog) -> TuningRecommendation | None:
    """Estimate the dominant motor-noise frequency and suggest INS_HNTCH_* params."""
    if not log.has("IMU"):
        return None
    times = log.times("IMU")
    fs = _estimate_rate_hz(times)
    if fs is None or fs < NOTCH_MIN_FS_HZ:
        return None  # sample rate too low to resolve motor noise — skip
    picked = _dominant_gyro_axis(log)
    if picked is None:
        return None
    axis, vals = picked
    if vals.size < NOTCH_MIN_SAMPLES:
        return None

    # Real FFT of the mean-removed signal (drop DC so the window below excludes it).
    ac = vals - float(np.mean(vals))
    spectrum = np.abs(np.fft.rfft(ac))
    freqs = np.fft.rfftfreq(ac.size, d=1.0 / fs)
    power = spectrum**2

    hi_hz = NOTCH_MAX_FS_FRACTION * fs
    band = (freqs >= NOTCH_MIN_HZ) & (freqs <= hi_hz)
    if not np.any(band):
        return None
    band_power = power[band]
    band_freqs = freqs[band]

    peak_idx = int(np.argmax(band_power))
    peak_power = float(band_power[peak_idx])
    peak_hz = float(band_freqs[peak_idx])
    median_power = float(np.median(band_power))
    # Ratio of the peak above the in-band noise floor. Guard against a zero/near
    # zero median (flat or near-silent spectrum) so a clean log does not trip.
    if median_power <= 0.0:
        return None
    ratio = peak_power / median_power
    if ratio < NOTCH_PEAK_RATIO:
        return None  # no peak that clearly stands out — nothing to recommend

    freq_hz = round(peak_hz, 0)
    bw_hz = round(peak_hz / 2.0, 0)
    suggested = {
        "INS_HNTCH_ENABLE": 1.0,
        "INS_HNTCH_MODE": 1.0,
        "INS_HNTCH_FREQ": freq_hz,
        "INS_HNTCH_BW": bw_hz,
        "INS_HNTCH_REF": 1.0,
    }
    explanation = (
        f"The {axis} gyro spectrum has a dominant peak at {peak_hz:.0f} Hz "
        f"(power {ratio:.1f}x the in-band median), consistent with motor/propeller "
        f"noise. A harmonic notch centred on {freq_hz:.0f} Hz would attenuate this "
        "before it reaches the rate controllers and the EKF. Suggested INS_HNTCH_* "
        "values configure a throttle-independent notch at the detected frequency "
        f"with a {bw_hz:.0f} Hz bandwidth (recommendation only — verify in flight)."
    )
    evidence = Evidence(
        message_types=["IMU"],
        samples={
            "peak_hz": round(peak_hz, 1),
            "peak_ratio": round(ratio, 2),
            "sample_rate_hz": round(fs, 1),
            "axis_samples": float(vals.size),
        },
        detail=f"Dominant gyro axis: {axis}.",
    )
    return TuningRecommendation(
        area=TuningArea.NOTCH,
        title=f"Harmonic notch at {freq_hz:.0f} Hz",
        explanation=explanation,
        suggested_params=suggested,
        evidence=evidence,
        confidence="medium",
    )


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
