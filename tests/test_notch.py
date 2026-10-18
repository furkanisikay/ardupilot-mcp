"""Harmonic-notch advice (ardupilot_mcp.notch) from batch-sampler, IMU and FFT data."""

from __future__ import annotations

import math

import numpy as np

from ardupilot_mcp.model import TuningArea
from ardupilot_mcp.notch import NOTCH_MIN_HZ, analyze_notch
from ardupilot_mcp.parser import parse_bin
from tests.helpers import build_series, make_flight_log
from tests.synth_bin import build_bin

RATE = 2000.0
BLOCK = 32
MUL = 1000.0  # raw int16 = rad/s * MUL


def _gyro(n: int, tones: dict[float, float], seed: int, *, drift: bool = False) -> np.ndarray:
    t = np.arange(n) / RATE
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(n) * 0.02
    if drift:  # airframe motion: power rising towards low frequencies (brown noise)
        v = np.cumsum(v) * 0.5
    for hz, amp in tones.items():
        v += amp * np.sin(2.0 * math.pi * hz * t)
    return v


def _batch(
    n_seq: int, t_us: int, instance: int, tones: dict[float, float], *, blocks=32, skip=(), drift=False
):
    """One ISBH header and its ISBD blocks (gyro, ``blocks`` x 32 samples)."""
    n = blocks * BLOCK
    axes = [_gyro(n, tones, seed=n_seq * 10 + k, drift=drift) for k in range(3)]
    msgs = [
        (
            "ISBH",
            {
                "TimeUS": t_us,
                "N": n_seq,
                "type": 1,
                "instance": instance,
                "mul": int(MUL),
                "smp_cnt": n,
                "SampleUS": t_us - 1000,
                "smp_rate": RATE,
            },
        )
    ]
    for b in range(blocks):
        if b in skip:
            continue
        sl = slice(b * BLOCK, (b + 1) * BLOCK)
        msgs.append(
            (
                "ISBD",
                {
                    "TimeUS": t_us + b,
                    "N": n_seq,
                    "seqno": b,
                    **{
                        name: list(np.round(a[sl] * MUL).astype(int))
                        for name, a in zip("xyz", axes, strict=True)
                    },
                },
            )
        )
    return msgs


def _bin(tmp_path, batches, params=None):
    msgs = [("ATT", {"TimeUS": i * 100_000, "Roll": 0.0}) for i in range(50)]
    for m in batches:
        msgs.extend(m)
    return parse_bin(build_bin(str(tmp_path / "batch.bin"), msgs, params=params))


# Strongest line at 240 Hz (2nd harmonic), fundamental at 120 Hz.
TONES = {120.0: 0.15, 240.0: 0.4}


def test_batches_are_decoded_and_split_pre_post(tmp_path):
    # INS_LOG_BAT_OPT bit 2 (pre+post): the post-filter batch is logged as instance + IMU count.
    batches = [_batch(1, 1_000_000, 0, TONES), _batch(2, 2_000_000, 1, {120.0: 0.01, 240.0: 0.02})]
    log = _bin(tmp_path, batches, params={"INS_LOG_BAT_OPT": 4})
    assert len(log.batches) == 2
    pre, post = sorted(log.batches, key=lambda b: b.post_filter)
    assert (pre.post_filter, post.post_filter) == (False, True)
    assert pre.instance == post.instance == 0
    assert pre.sensor == "gyro" and pre.rate_hz == RATE
    assert pre.x.size == 32 * BLOCK
    # Values are raw / mul, and the time comes from the header's TimeUS.
    expect = _gyro(32 * BLOCK, TONES, seed=10)
    assert np.allclose(pre.x, expect, atol=1.0 / MUL)
    assert abs(pre.time_s - (1.0 - log.meta.start_time_s)) < 1e-6


def test_post_filter_option_without_pre_post(tmp_path):
    # Bit 1 alone: every batch is post-filter and none carries the instance offset.
    log = _bin(tmp_path, [_batch(1, 1_000_000, 0, TONES, blocks=4)], params={"INS_LOG_BAT_OPT": 2})
    assert [b.post_filter for b in log.batches] == [True]
    both = [_batch(1, 1_000_000, 0, TONES, blocks=4), _batch(2, 2_000_000, 1, TONES, blocks=4)]
    # Bits 0+1 (post-filter at sensor rate): the offset shows that sensor-rate sampling was
    # on, and at sensor rate the backend feeds the sampler raw samples in every pass.
    log = _bin(tmp_path, both, params={"INS_LOG_BAT_OPT": 3})
    assert sorted((b.instance, b.post_filter) for b in log.batches) == [(0, False), (0, False)]
    # Bits 0+2 (pre+post at sensor rate): the offset pass is filtered only if this IMU was not
    # sampled at sensor rate, which the log does not say.
    log = _bin(tmp_path, both, params={"INS_LOG_BAT_OPT": 5})
    assert sorted((b.instance, str(b.post_filter)) for b in log.batches) == [(0, "False"), (0, "None")]
    # Bits 0+1 with no offset batch: that IMU is not sampled at sensor rate, so post-filter.
    log = _bin(tmp_path, [_batch(1, 1_000_000, 0, TONES, blocks=4)], params={"INS_LOG_BAT_OPT": 3})
    assert [b.post_filter for b in log.batches] == [True]


def test_a_dropped_block_ends_the_batch(tmp_path):
    log = _bin(tmp_path, [_batch(1, 1_000_000, 0, TONES, skip=(10,))])
    assert log.batches[0].x.size == 10 * BLOCK


def test_blocks_of_another_batch_are_not_joined(tmp_path):
    # An ISBD whose N does not match the header before it belongs to no batch.
    stray = _batch(7, 1_500_000, 0, TONES, blocks=4)[1:]
    log = _bin(tmp_path, [_batch(1, 1_000_000, 0, TONES, blocks=4) + stray])
    assert len(log.batches) == 1
    assert log.batches[0].x.size == 4 * BLOCK


def test_notch_from_batches_centres_on_the_fundamental(tmp_path):
    batches = [_batch(i, i * 1_000_000, 0, TONES) for i in range(1, 5)]
    batches.append(_batch(9, 6_000_000, 1, {120.0: 0.01, 240.0: 0.02}))
    log = _bin(tmp_path, batches, params={"INS_LOG_BAT_OPT": 4, "MOT_THST_HOVER": 0.32})
    rec = analyze_notch(log)
    assert rec is not None and rec.area == TuningArea.NOTCH
    assert abs(rec.suggested_params["INS_HNTCH_FREQ"] - 120.0) <= 3.0
    assert rec.suggested_params["INS_HNTCH_BW"] == round(rec.suggested_params["INS_HNTCH_FREQ"] / 2)
    assert rec.suggested_params["INS_HNTCH_MODE"] == 1.0
    assert rec.suggested_params["INS_HNTCH_REF"] == 0.32
    assert rec.confidence == "high"
    assert rec.evidence.message_types == ["ISBH", "ISBD"]
    assert rec.evidence.samples["batches"] == 4.0
    assert rec.evidence.samples["attenuation_db"] > 15.0


def test_second_pass_of_unknown_kind_is_compared_with_a_caveat(tmp_path):
    batches = [_batch(i, i * 1_000_000, 0, TONES) for i in range(1, 3)]
    batches.append(_batch(9, 6_000_000, 1, {120.0: 0.01, 240.0: 0.02}))
    rec = analyze_notch(_bin(tmp_path, batches, params={"INS_LOG_BAT_OPT": 5}))
    assert rec is not None
    assert "second-pass batches" in rec.explanation
    # A large difference shows the pass was filtered; the raw-pass caveat is for small ones.
    assert "logged raw in both passes" not in rec.explanation
    assert rec.evidence.samples["attenuation_db"] > 15.0
    raw = [_batch(i, i * 1_000_000, 0, TONES) for i in range(1, 3)] + [_batch(9, 6_000_000, 1, TONES)]
    rec = analyze_notch(_bin(tmp_path, raw, params={"INS_LOG_BAT_OPT": 5}))
    assert rec is not None and "logged raw in both passes" in rec.explanation


def test_peaks_below_the_floor_are_ignored(tmp_path):
    log = _bin(tmp_path, [_batch(1, 1_000_000, 0, {NOTCH_MIN_HZ / 2: 0.5})])
    assert analyze_notch(log) is None


def test_low_frequency_drift_is_not_a_peak(tmp_path):
    # The spectrum rises towards the floor; its first bin is the largest but no line.
    quiet = _bin(tmp_path, [_batch(i, i * 1_000_000, 0, {}, drift=True) for i in range(1, 4)])
    assert analyze_notch(quiet) is None
    tone = _bin(tmp_path, [_batch(i, i * 1_000_000, 0, {150.0: 0.05}, drift=True) for i in range(1, 4)])
    rec = analyze_notch(tone)
    assert rec is not None and abs(rec.suggested_params["INS_HNTCH_FREQ"] - 150.0) <= 3.0


def _ftn1(hz: float, n: int = 40) -> list[dict]:
    return build_series(0.0, 10.0, PkAvg=[hz] * n, BwAvg=[hz / 4] * n)


def test_fft_is_the_fallback_without_raw_gyro():
    log = make_flight_log({"FTN1": _ftn1(150.0)})
    rec = analyze_notch(log)
    assert rec is not None
    assert rec.suggested_params["INS_HNTCH_FREQ"] == 150.0
    assert rec.evidence.message_types == ["FTN1"]
    assert rec.confidence == "medium"


def _imu(tone_hz: float) -> list[dict]:
    t = np.arange(2048) / 1000.0
    rng = np.random.default_rng(3)
    return build_series(
        0.0,
        1000.0,
        GyrX=list(30.0 * np.sin(2.0 * math.pi * tone_hz * t) + rng.standard_normal(t.size) * 0.5),
        GyrY=list(rng.standard_normal(t.size) * 0.2),
        GyrZ=list(rng.standard_normal(t.size) * 0.2),
    )


def test_fft_cross_check_is_reported():
    agree = analyze_notch(make_flight_log({"IMU": _imu(80.0), "FTN1": _ftn1(82.0)}))
    assert agree is not None and "FFT agrees" in agree.explanation
    differ = analyze_notch(make_flight_log({"IMU": _imu(80.0), "FTN1": _ftn1(160.0)}))
    assert differ is not None and "centres on 160 Hz instead" in differ.explanation


def _esc(rpm_mid: float) -> list[dict]:
    # Motor speed spread +-10% around rpm_mid through the flight.
    return build_series(0.0, 10.0, RPM=list(np.linspace(0.9, 1.1, 41) * rpm_mid))


def test_esc_rpm_selects_the_esc_notch_from_the_low_motor_speed():
    # 80 Hz gyro line; motors at 4800 rpm = 80 Hz median, 5th percentile ~73 Hz.
    rec = analyze_notch(
        make_flight_log({"IMU": _imu(80.0), "ESC": _esc(4800.0)}, params={"INS_GYRO_FILTER": 20})
    )
    assert rec is not None
    p = rec.suggested_params
    assert p["INS_HNTCH_MODE"] == 3.0 and p["INS_HNTCH_REF"] == 1.0
    assert p["INS_HNTCH_FREQ"] == 72.0  # floor of the 5th percentile, below hover
    assert p["INS_HNTCH_BW"] == 36.0
    assert "fundamental" in rec.explanation
    assert "ESC" in rec.evidence.message_types


def test_esc_freq_stays_above_the_gyro_filter():
    rec = analyze_notch(make_flight_log({"ESC": _esc(1500.0)}, params={"INS_GYRO_FILTER": 40}))
    assert rec is not None  # ESC telemetry alone is enough for the ESC notch
    assert rec.suggested_params["INS_HNTCH_FREQ"] == 41.0
    assert rec.confidence == "medium"


def test_a_motor_harmonic_outside_hmncs_is_added():
    # The gyro line is the 4th harmonic of 40 Hz motors; HMNCS default covers 1st+2nd only.
    rec = analyze_notch(make_flight_log({"IMU": _imu(160.0), "ESC": _esc(2400.0)}))
    assert rec is not None
    assert "4th harmonic" in rec.explanation
    assert rec.suggested_params["INS_HNTCH_HMNCS"] == float(3 | 8)


def test_a_line_that_is_no_motor_harmonic_is_called_out():
    rec = analyze_notch(make_flight_log({"IMU": _imu(30.0), "ESC": _esc(12000.0)}))
    assert rec is not None
    assert "not motor noise" in rec.explanation
    assert "INS_HNTCH_HMNCS" not in rec.suggested_params


def test_plane_without_vtol_gets_a_fixed_notch():
    log = make_flight_log({"IMU": _imu(80.0)}, vehicle_type="ArduPlane", vehicle_kind="plane")
    rec = analyze_notch(log)
    assert rec is not None
    assert rec.suggested_params["INS_HNTCH_MODE"] == 0.0


def test_quadplane_uses_the_vtol_hover_throttle():
    log = make_flight_log(
        {"IMU": _imu(80.0)},
        params={"Q_ENABLE": 1, "Q_M_THST_HOVER": 0.41},
        vehicle_type="ArduPlane",
        vehicle_kind="plane",
    )
    rec = analyze_notch(log)
    assert rec is not None
    assert rec.suggested_params["INS_HNTCH_MODE"] == 1.0
    assert rec.suggested_params["INS_HNTCH_REF"] == 0.41


def test_rover_gets_a_throttle_notch_without_a_guessed_ref():
    log = make_flight_log({"IMU": _imu(80.0)}, vehicle_type="ArduRover", vehicle_kind="rover")
    rec = analyze_notch(log)
    assert rec is not None
    assert rec.suggested_params["INS_HNTCH_MODE"] == 1.0
    assert "INS_HNTCH_REF" not in rec.suggested_params
