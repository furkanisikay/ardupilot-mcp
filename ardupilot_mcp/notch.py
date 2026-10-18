"""Harmonic-notch advice from the data that can actually show motor noise.

Motor and propeller noise sits at tens to hundreds of Hz. The IMU message is usually
logged at 25-50 Hz, far below that, so a spectrum of it shows nothing (or an alias).
ArduPilot logs three things that can show it, tried here in order of quality:

1. **Batch sampler** (ISBH/ISBD, ``INS_LOG_BAT_MASK``): raw gyro at the full sensor
   rate (1-8 kHz) before the filters. Spectra of every pre-filter gyro batch taken in
   flight are averaged (Welch). With pre- and post-filter batches both logged
   (``INS_LOG_BAT_OPT``), the attenuation at the peak is measured too.
2. **IMU at a high rate** (``IMU_FAST`` in LOG_BITMASK): the primary IMU's gyro, in flight,
   when its rate is at least ``NOTCH_MIN_FS_HZ``.
3. **In-flight FFT** (FTN1, ``FFT_ENABLE``): the firmware's own weighted noise-centre
   frequency, as its median in flight.

The advice follows the harmonic-notch parameter documentation
(libraries/Filter/HarmonicNotchFilter.cpp) and the ArduPilot wiki. For a throttle notch
FREQ is the noise frequency at hover and REF the hover throttle; BW is half of FREQ. ESC
telemetry with RPM lets the notch follow the motors (MODE 3, REF 1); FREQ is then the
lowest motor frequency it follows, and the gyro line is checked against the motors'
harmonics, which tells motor noise from a frame resonance. A plane without VTOL motors
cannot use the throttle mode.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .flight_log import FlightLog, ImuBatch
from .model import Evidence, TuningArea, TuningRecommendation

# Design choice: ignore everything below this when searching for the motor-noise peak.
# Low frequencies are airframe motion, control inputs and crash transients, not motor hum
# (a forum crash log peaked at 13 Hz); a 20 Hz floor is still below the hover noise of
# large, slow propellers.
NOTCH_MIN_HZ = 20.0
# Search up to this fraction of the sample rate (a conservative slice of Nyquist).
NOTCH_MAX_FS_FRACTION = 0.45
# INS_HNTCH_FREQ is limited to 10-495 Hz (parameter range) and must stay below half the
# backend gyro rate; a peak above that is out of the notch's reach.
NOTCH_MAX_HZ = 495.0
# Minimum IMU rate (Hz) that can resolve a motor peak; below it the spectrum is aliased.
NOTCH_MIN_FS_HZ = 100.0
# Minimum samples for a usable spectrum.
NOTCH_MIN_SAMPLES = 64
# Welch segment length for the IMU fallback.
WELCH_SEGMENT = 256
# Design choice: a line counts only if its power is at least this multiple of its local
# floor, the median power within a factor LOCAL_SPAN of its frequency (an octave either
# side: wide enough that a broad motor hump does not raise its own floor).
NOTCH_PEAK_RATIO = 5.0
LOCAL_SPAN = 2.0
# Design choice: the strongest line is often the 2nd or 3rd harmonic. If the spectrum
# also has a clear line (NOTCH_PEAK_RATIO) within this fraction of f/2 or f/3, that lower
# line is the fundamental the notch is centred on.
SUBHARMONIC_TOLERANCE = 0.06
# Design choice: the pre/post-filter comparison and the FFT cross-check use the band
# within this fraction of the peak frequency.
ATTENUATION_BAND = 0.15
# Design choice: with a second pass of unknown kind, a difference below this many dB is
# flagged as possibly raw (a filtered pass is normally tens of dB lower at the peak).
RAW_PASS_DB = 6.0
# Design choice: the in-flight FFT is mentioned as disagreeing when its centre frequency
# differs from the measured peak by more than this fraction.
FFT_AGREE_FRACTION = 0.2
# ESC telemetry counts as RPM tracking when its median RPM in flight exceeds this.
ESC_RPM_MIN = 500.0
# Design choice: a gyro line is a motor harmonic when it lies within k times the motors'
# in-flight speed range (5th-95th percentile of ESC RPM), widened by HARMONIC_SLACK for
# the spectrum's resolution and the averaging over the flight, for k up to MAX_HARMONIC.
MAX_HARMONIC = 4
HARMONIC_SLACK = 0.1
# INS_HNTCH_FREQ parameter minimum (HarmonicNotchFilter.cpp, range 10-495 Hz).
FREQ_PARAM_MIN_HZ = 10.0
# INS_HNTCH_HMNCS default: 1st and 2nd harmonics (HarmonicNotchFilter.cpp).
HMNCS_DEFAULT = 3

# INS_HNTCH_MODE values (HarmonicNotchFilter.cpp): 0 fixed, 1 throttle, 2 RPM sensor,
# 3 ESC telemetry, 4 dynamic FFT.
MODE_FIXED, MODE_THROTTLE, MODE_RPM, MODE_ESC = 0, 1, 2, 3


@dataclass
class NoisePeak:
    hz: float
    source: str  # "batch sampler", "IMU", "in-flight FFT"
    ratio: float | None = None  # line power / its local floor
    fs_hz: float | None = None
    samples: dict[str, float] = field(default_factory=dict)
    detail: str = ""


def _baseline(freqs: np.ndarray, power: np.ndarray) -> np.ndarray:
    """Per-bin local noise floor: the median power within a factor LOCAL_SPAN in frequency."""
    base = np.full(power.shape, np.nan)
    for k, f in enumerate(freqs):
        if f > 0:
            near = (freqs >= f / LOCAL_SPAN) & (freqs <= f * LOCAL_SPAN)
            base[k] = float(np.median(power[near]))
    return base


def _peak(freqs: np.ndarray, power: np.ndarray, fs: float) -> tuple[float, float] | None:
    """(fundamental Hz, prominence) of the strongest in-band line, or None if none stands out.

    A line is a local maximum at least NOTCH_PEAK_RATIO times its local floor. Measuring
    against the local floor (not the band median) keeps the rising low-frequency end of
    the spectrum (airframe motion) from passing for a peak at the band edge.
    """
    hi = min(NOTCH_MAX_FS_FRACTION * fs, NOTCH_MAX_HZ)
    base = _baseline(freqs, power)
    with np.errstate(divide="ignore", invalid="ignore"):
        prom = np.where(base > 0, power / base, 0.0)
    k = np.arange(1, power.size - 1)
    local_max = (power[k] > power[k - 1]) & (power[k] >= power[k + 1])
    lines = k[local_max & (freqs[k] >= NOTCH_MIN_HZ) & (freqs[k] <= hi) & (prom[k] >= NOTCH_PEAK_RATIO)]
    if lines.size == 0:
        return None
    best = int(lines[np.argmax(power[lines])])
    f0 = float(freqs[best])
    for div in (3, 2):  # the lowest clear sub-harmonic wins
        target = f0 / div
        near = lines[np.abs(freqs[lines] - target) <= SUBHARMONIC_TOLERANCE * target]
        if near.size:
            j = int(near[np.argmax(power[near])])
            return float(freqs[j]), float(prom[j])
    return f0, float(prom[best])


def _ac(v: np.ndarray) -> np.ndarray | None:
    """``v`` minus its mean, or None if it does not vary.

    Gyro values are logged as float32, so a spread below float32 resolution of their
    magnitude is representation noise, not a signal (its spectrum is all rounding).
    """
    a = v.astype(float)
    mean = float(np.mean(a))
    if float(np.ptp(a)) <= float(np.finfo(np.float32).eps) * max(1.0, abs(mean)):
        return None
    return a - mean


def _spectrum(batch: ImuBatch) -> np.ndarray:
    win = np.hanning(batch.x.size)
    total = np.zeros(batch.x.size // 2 + 1)
    for axis in (batch.x, batch.y, batch.z):
        if (a := _ac(axis)) is not None:
            total += np.abs(np.fft.rfft(a * win)) ** 2
    return total


def _flying(log: FlightLog, t: np.ndarray) -> np.ndarray:
    mask = log.flying_mask(t)
    return np.ones(t.shape, dtype=bool) if mask is None else mask


def from_batches(log: FlightLog) -> NoisePeak | None:
    gyro = [b for b in log.batches if b.sensor == "gyro" and b.x.size >= NOTCH_MIN_SAMPLES]
    if not gyro:
        return None
    t = np.array([b.time_s for b in gyro])
    gyro = [b for b, ok in zip(gyro, _flying(log, t), strict=True) if ok]
    pre = [b for b in gyro if b.post_filter is False]
    if not pre:
        return None
    from .checks.vibration import primary_imu

    inst = primary_imu(log, sorted({b.instance for b in pre}))
    pre = [b for b in pre if b.instance == inst]
    n = int(np.median([b.x.size for b in pre]))
    pre = [b for b in pre if b.x.size == n]
    fs = float(np.median([b.rate_hz for b in pre]))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    power = np.mean([_spectrum(b) for b in pre], axis=0)
    found = _peak(freqs, power, fs)
    if found is None:
        return None
    hz, ratio = found
    samples = {"batches": float(len(pre)), "imu": float(inst), "sample_rate_hz": round(fs, 1)}
    # The second pass of a pre+post log at sensor rate may be raw too (post_filter None).
    post = [b for b in gyro if b.post_filter is not False and b.instance == inst and b.x.size == n]
    unsure = any(b.post_filter is None for b in post)
    detail = f"Pre-filter gyro batches of IMU{inst}, {len(pre)} in flight, {n} samples each at {fs:.0f} Hz."
    if post:
        # Measured over a band around the peak and only reported: pre- and post-filter
        # batches are captured at different moments, so it is a guide, not a filter test.
        post_power = np.mean([_spectrum(b) for b in post], axis=0)
        band = np.abs(freqs - hz) <= ATTENUATION_BAND * hz
        pre_e, post_e = float(np.sum(power[band])), float(np.sum(post_power[band]))
        if pre_e > 0 and post_e > 0:
            att = 10.0 * float(np.log10(pre_e / post_e))
            samples["attenuation_db"] = round(att, 1)
            change = f"{abs(att):.0f} dB {'less' if att >= 0 else 'more'}"
            kind = "second-pass" if unsure else "post-filter"
            detail += (
                f" The {len(post)} {kind} batches carry {change} energy within "
                f"{ATTENUATION_BAND:.0%} of that frequency than the pre-filter ones."
            )
            if unsure and att < RAW_PASS_DB:
                detail += (
                    " With INS_LOG_BAT_OPT sampling at sensor rate, an IMU that supports it is "
                    "logged raw in both passes, so a difference near 0 dB means that pass is raw too."
                )
    return NoisePeak(hz=hz, source="batch sampler", ratio=ratio, fs_hz=fs, samples=samples, detail=detail)


def from_imu(log: FlightLog) -> NoisePeak | None:
    if not log.has("IMU"):
        return None
    from .checks.vibration import primary_imu

    imus = log.instances("IMU") or [0]
    inst = primary_imu(log, imus) if log.instance_field("IMU") else None
    t, cols = log.aligned("IMU", ["GyrX", "GyrY", "GyrZ"], inst)
    if t.size < NOTCH_MIN_SAMPLES:
        return None
    dt = np.diff(t)
    dt = dt[dt > 0]
    if dt.size == 0:
        return None
    fs = 1.0 / float(np.median(dt))
    if fs < NOTCH_MIN_FS_HZ:
        return None
    keep = _flying(log, t)
    for c in cols.values():
        keep &= np.isfinite(c)
    if np.count_nonzero(keep) < NOTCH_MIN_SAMPLES:
        return None
    seg = min(WELCH_SEGMENT, int(np.count_nonzero(keep)))
    win = np.hanning(seg)
    power = np.zeros(seg // 2 + 1)
    nseg = 0
    for c in cols.values():
        v = c[keep]
        for s in range(0, v.size - seg + 1, seg // 2):
            chunk = _ac(v[s : s + seg])
            if chunk is not None:
                power += np.abs(np.fft.rfft(chunk * win)) ** 2
            nseg += 1
    if nseg == 0:
        return None
    freqs = np.fft.rfftfreq(seg, d=1.0 / fs)
    found = _peak(freqs, power / nseg, fs)
    if found is None:
        return None
    hz, ratio = found
    label = f"IMU{inst}" if inst is not None else "IMU"
    return NoisePeak(
        hz=hz,
        source="IMU",
        ratio=ratio,
        fs_hz=fs,
        samples={"sample_rate_hz": round(fs, 1), "imu": float(inst or 0)},
        detail=f"{label} gyro at {fs:.0f} Hz in flight, {nseg} Welch segments of {seg} samples.",
    )


def from_fft(log: FlightLog) -> NoisePeak | None:
    t, pk = log.series("FTN1", "PkAvg")
    if pk.size == 0:
        return None
    keep = _flying(log, t) & (pk > NOTCH_MIN_HZ)
    if np.count_nonzero(keep) < 10:
        return None
    hz = float(np.median(pk[keep]))
    return NoisePeak(
        hz=hz,
        source="in-flight FFT",
        samples={"fft_samples": float(np.count_nonzero(keep))},
        detail=f"Median of FTN1.PkAvg over {np.count_nonzero(keep)} in-flight samples.",
    )


@dataclass
class MotorSpeed:
    """In-flight motor rotation frequency from ESC telemetry (RPM / 60), in Hz."""

    low: float  # 5th percentile
    median: float
    high: float  # 95th percentile


def _motor_speed(log: FlightLog) -> MotorSpeed | None:
    t, rpm = log.series("ESC", "RPM")
    if rpm.size == 0:
        return None
    turning = rpm[_flying(log, t) & (rpm > 0)]
    if turning.size == 0 or float(np.median(turning)) <= ESC_RPM_MIN:
        return None
    lo, med, hi = np.percentile(turning / 60.0, [5, 50, 95])
    return MotorSpeed(low=float(lo), median=float(med), high=float(hi))


def _harmonic(hz: float, speed: MotorSpeed) -> int | None:
    """The lowest motor harmonic (1 = fundamental) whose in-flight range holds ``hz``."""
    for k in range(1, MAX_HARMONIC + 1):
        if k * speed.low * (1 - HARMONIC_SLACK) <= hz <= k * speed.high * (1 + HARMONIC_SLACK):
            return k
    return None


def _mode(log: FlightLog, speed: MotorSpeed | None) -> tuple[int, float | None, str]:
    """(INS_HNTCH_MODE, INS_HNTCH_REF, why) for this vehicle.

    Throttle tracking exists for Copter/Heli, Plane (its VTOL motors) and Rover
    (AP_Vehicle::update_throttle_notch); the parameter documentation rules it out for a
    fixed-wing-only plane.
    """
    from .profile import _hover_throttle

    if speed is not None:
        return MODE_ESC, 1.0, "ESC telemetry reports motor RPM, so the notch follows the motors directly"
    kind = log.meta.vehicle_kind
    vtol = kind == "plane" and float(log.params.get("Q_ENABLE", 0)) > 0
    if kind in ("copter", "heli", None) or vtol:
        hover, estimated = _hover_throttle(log)
        if vtol:
            q = log.params.get("Q_M_THST_HOVER")
            hover, estimated = (float(q), False) if q is not None else (None, False)
        if hover is not None:
            src = "the median flying throttle" if estimated else "the learned hover throttle"
            return (
                MODE_THROTTLE,
                round(hover, 2),
                f"throttle-based: REF is the throttle the frequency was measured at ({src})",
            )
        return (
            MODE_THROTTLE,
            None,
            "throttle-based: set REF to the hover throttle the frequency was measured at",
        )
    if kind == "rover":
        return MODE_THROTTLE, None, "throttle-based: set REF to the throttle the frequency was measured at"
    if kind == "plane":
        return (
            MODE_FIXED,
            0.0,
            "fixed: a fixed-wing-only plane cannot use the throttle-based notch without ESC RPM",
        )
    return MODE_FIXED, 0.0, f"fixed: throttle tracking is not available on a {kind}"


_ORDINAL = {1: "fundamental", 2: "2nd harmonic", 3: "3rd harmonic", 4: "4th harmonic"}


def _esc_advice(
    log: FlightLog, peak: NoisePeak | None, speed: MotorSpeed
) -> tuple[float, list[str], str, dict]:
    """(FREQ, explanation parts, confidence, extra params) for an ESC-telemetry notch.

    In ESC mode FREQ is the lowest motor frequency the notch tracks; the wiki asks for it
    below the hover frequency and above INS_GYRO_FILTER (ESC Telemetry Based Harmonic
    Notch Setup). The in-flight 5th percentile of motor speed is below hover.
    """
    gyro_filter = log.params.get("INS_GYRO_FILTER")
    floor_hz = math.floor(float(gyro_filter)) + 1.0 if gyro_filter else FREQ_PARAM_MIN_HZ
    freq = min(max(float(math.floor(speed.low)), floor_hz), NOTCH_MAX_HZ)
    parts = [
        f"ESC telemetry puts the motors at {speed.median:.0f} Hz in flight (median; 5th-95th "
        f"percentile {speed.low:.0f}-{speed.high:.0f} Hz). In ESC mode FREQ is the lowest motor "
        "frequency the notch follows, set below the hover frequency and above INS_GYRO_FILTER."
    ]
    if gyro_filter and floor_hz > speed.median:
        parts.append(
            f"INS_GYRO_FILTER ({float(gyro_filter):.0f} Hz) is above the motors' median frequency, so "
            "FREQ cannot be both; it is kept above the gyro filter."
        )
    extra: dict[str, float] = {}
    if peak is None:
        parts.append("No gyro data in the log shows the noise itself (no batch sampler, fast IMU or FFT).")
        return freq, parts, "medium", extra
    k = _harmonic(peak.hz, speed)
    seen = f"The strongest gyro line, {peak.hz:.0f} Hz ({peak.source})"
    if k is None:
        parts.append(
            f"{seen}, matches no motor harmonic up to the {_ORDINAL[MAX_HARMONIC]}, so it is not "
            "motor noise: look for a frame or mount resonance."
        )
        return freq, parts, "medium", extra
    parts.append(f"{seen}, is the motors' {_ORDINAL[k]}. {peak.detail}")
    hmncs = int(log.params.get("INS_HNTCH_HMNCS", HMNCS_DEFAULT))
    if not hmncs & (1 << (k - 1)):
        extra["INS_HNTCH_HMNCS"] = float(hmncs | 1 | (1 << (k - 1)))
        parts.append(f"INS_HNTCH_HMNCS gains the {_ORDINAL[k]} so the notch covers that line.")
    return freq, parts, ("high" if peak.source == "batch sampler" else "medium"), extra


def analyze_notch(log: FlightLog) -> TuningRecommendation | None:
    peak = from_batches(log) or from_imu(log) or from_fft(log)
    speed = _motor_speed(log)
    if peak is None and speed is None:
        return None
    mode, ref, why = _mode(log, speed)
    extra: dict[str, float] = {}
    if speed is not None:
        freq, parts, confidence, extra = _esc_advice(log, peak, speed)
        title = f"Harmonic notch on ESC RPM, from {freq:.0f} Hz"
    else:
        assert peak is not None
        freq = float(round(peak.hz))
        ratio_txt = f", {peak.ratio:.1f}x the surrounding noise floor" if peak.ratio is not None else ""
        parts = [
            f"Motor noise peaks at {peak.hz:.0f} Hz ({peak.source}{ratio_txt}). {peak.detail}",
            "A harmonic notch centred there removes it before the rate controllers and the EKF.",
        ]
        confidence = "high" if peak.source == "batch sampler" else "medium"
        if peak.source != "in-flight FFT" and (fft := from_fft(log)) is not None:
            if abs(fft.hz - peak.hz) <= FFT_AGREE_FRACTION * peak.hz:
                parts.append(f"The in-flight FFT agrees (median centre {fft.hz:.0f} Hz).")
            else:
                confidence = "medium"
                parts.append(
                    f"The in-flight FFT centres on {fft.hz:.0f} Hz instead (FTN1.PkAvg, its energy-weighted "
                    "roll/pitch peak); compare both in a spectrum view before choosing."
                )
        title = f"Harmonic notch at {freq:.0f} Hz"
    bw = float(round(freq / 2.0))
    suggested = {
        "INS_HNTCH_ENABLE": 1.0,
        "INS_HNTCH_MODE": float(mode),
        "INS_HNTCH_FREQ": freq,
        "INS_HNTCH_BW": bw,
    }
    if ref is not None:
        suggested["INS_HNTCH_REF"] = ref
    suggested.update(extra)
    parts.append(f"Mode {mode} ({why}); BW is half of FREQ, as the parameter documentation advises.")
    if (
        float(log.params.get("INS_HNTCH_ENABLE", 0)) > 0
        and (cf := log.params.get("INS_HNTCH_FREQ")) is not None
    ):
        cbw = log.params.get("INS_HNTCH_BW")
        parts.append(
            f"The log's notch: mode {log.params.get('INS_HNTCH_MODE', 0):.0f}, FREQ {cf:.0f} Hz, BW {cbw or 0:.0f} Hz."
        )
    parts.append("Recommendation only: verify with a post-filter batch log after the change.")
    samples: dict[str, float] = {}
    msg: list[str] = []
    if peak is not None:
        samples["peak_hz"] = round(peak.hz, 1)
        samples.update(peak.samples)
        if peak.ratio is not None:
            samples["peak_ratio"] = round(peak.ratio, 2)
        msg += {"batch sampler": ["ISBH", "ISBD"], "IMU": ["IMU"], "in-flight FFT": ["FTN1"]}[peak.source]
    if speed is not None:
        samples["motor_hz_median"] = round(speed.median, 1)
        samples["motor_hz_p5"] = round(speed.low, 1)
        msg.append("ESC")
    return TuningRecommendation(
        area=TuningArea.NOTCH,
        title=title,
        explanation=" ".join(parts),
        suggested_params=suggested,
        evidence=Evidence(message_types=msg, samples=samples, detail=peak.detail if peak else parts[0]),
        confidence=confidence,
    )
