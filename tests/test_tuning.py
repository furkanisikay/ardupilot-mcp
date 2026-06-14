"""Tests for the advisory tuning analyser (ardupilot_mcp.tuning).

Not a check: ``analyze_tuning`` is a plain function returning a list of
``TuningRecommendation``. These tests build synthetic IMU/ATT logs with
``make_flight_log`` / ``build_series`` and assert on the area, confidence and a
key evidence sample for each path.
"""

from __future__ import annotations

import math

import numpy as np

from ardupilot_mcp.model import TuningArea
from ardupilot_mcp.tuning import analyze_tuning
from tests.helpers import build_series, make_flight_log

# --- builders ----------------------------------------------------------------


def _gyro_imu_records(rate_hz: float, n: int, *, tone_hz: float | None, amp: float = 30.0):
    """N IMU records at rate_hz. GyrX carries a tone (if given) on top of noise.

    The tone goes on GyrX; GyrY/GyrZ stay quiet so GyrX is the dominant axis.
    With ``tone_hz=None`` the gyro is a flat constant (no AC content) — a clean,
    peak-free spectrum that must not trigger a notch recommendation.
    """
    t = np.arange(n) / rate_hz
    rng = np.random.default_rng(1234)  # deterministic noise
    if tone_hz is not None:
        noise = rng.standard_normal(n) * 0.5
        gyrx = amp * np.sin(2.0 * math.pi * tone_hz * t) + noise
        gyry = list(rng.standard_normal(n) * 0.2)
        gyrz = list(rng.standard_normal(n) * 0.2)
    else:
        # Flat constant signal -> zero AC energy -> no in-band peak.
        gyrx = np.full(n, 0.5)
        gyry = [0.1] * n
        gyrz = [0.1] * n
    return build_series(
        0.0,
        rate_hz,
        GyrX=list(gyrx),
        GyrY=list(gyry),
        GyrZ=list(gyrz),
    )


def _att_records(rate_hz: float, n: int, *, roll_err_deg: float):
    """ATT records where actual roll lags desired by a constant offset (deg)."""
    des_roll = [10.0] * n
    act_roll = [10.0 - roll_err_deg] * n  # constant tracking error
    return build_series(
        0.0,
        rate_hz,
        DesRoll=des_roll,
        Roll=act_roll,
        DesPitch=[0.0] * n,
        Pitch=[0.0] * n,
        DesYaw=[0.0] * n,
        Yaw=[0.0] * n,
    )


# --- NOTCH -------------------------------------------------------------------


def test_notch_detects_80hz_tone():
    recs = _gyro_imu_records(1000.0, 2048, tone_hz=80.0, amp=30.0)
    log = make_flight_log({"IMU": recs})
    recs_out = analyze_tuning(log, area="notch")
    notch = [r for r in recs_out if r.area == TuningArea.NOTCH]
    assert notch, "expected a NOTCH recommendation for the 80 Hz tone"
    rec = notch[0]
    freq = rec.suggested_params["INS_HNTCH_FREQ"]
    assert abs(freq - 80.0) <= 5.0, f"INS_HNTCH_FREQ {freq} not within 5 Hz of 80"
    # Bandwidth is half the detected frequency.
    assert abs(rec.suggested_params["INS_HNTCH_BW"] - round(freq / 2.0, 0)) < 1e-6
    assert rec.suggested_params["INS_HNTCH_ENABLE"] == 1.0
    assert rec.suggested_params["INS_HNTCH_MODE"] == 1.0
    assert rec.suggested_params["INS_HNTCH_REF"] == 1.0
    assert rec.confidence == "medium"
    assert abs(rec.evidence.samples["peak_hz"] - 80.0) <= 5.0


def test_notch_skipped_without_imu():
    log = make_flight_log({})
    assert analyze_tuning(log, area="notch") == []


def test_notch_skipped_when_sample_rate_too_low():
    # 50 Hz IMU is below NOTCH_MIN_FS_HZ -> skip even with a tone.
    recs = _gyro_imu_records(50.0, 256, tone_hz=15.0, amp=30.0)
    log = make_flight_log({"IMU": recs})
    notch = [r for r in analyze_tuning(log, area="notch") if r.area == TuningArea.NOTCH]
    assert notch == []


def test_notch_no_recommendation_for_quiet_gyro():
    # Broadband low noise, no dominant tone -> no clear peak -> no recommendation.
    recs = _gyro_imu_records(1000.0, 2048, tone_hz=None)
    log = make_flight_log({"IMU": recs})
    notch = [r for r in analyze_tuning(log, area="notch") if r.area == TuningArea.NOTCH]
    assert notch == []


# --- PID ---------------------------------------------------------------------


def test_pid_flags_high_rms_tracking_error():
    # Constant 8 deg roll error -> RMS 8 deg > 5 deg threshold.
    recs = _att_records(50.0, 200, roll_err_deg=8.0)
    log = make_flight_log({"ATT": recs})
    pid = [r for r in analyze_tuning(log, area="pid") if r.area == TuningArea.PID]
    assert pid, "expected a PID recommendation for high RMS tracking error"
    rec = pid[0]
    assert "roll" in rec.title.lower()
    assert rec.confidence == "low"
    assert rec.suggested_params == {}  # qualitative advice only
    assert abs(rec.evidence.samples["rms_error_deg"] - 8.0) < 0.5


def test_pid_clean_tracking_no_recommendation():
    # 1 deg error -> below 5 deg threshold -> no recommendation.
    recs = _att_records(50.0, 200, roll_err_deg=1.0)
    log = make_flight_log({"ATT": recs})
    pid = [r for r in analyze_tuning(log, area="pid") if r.area == TuningArea.PID]
    assert pid == []


def test_pid_skipped_without_att():
    log = make_flight_log({})
    assert analyze_tuning(log, area="pid") == []


# --- AUTOTUNE ----------------------------------------------------------------


def test_autotune_from_params():
    log = make_flight_log({}, params={"AUTOTUNE_AGGR": 0.1, "AUTOTUNE_AXES": 7.0})
    rec = analyze_tuning(log, area="autotune")
    auto = [r for r in rec if r.area == TuningArea.AUTOTUNE]
    assert auto, "expected an AUTOTUNE recommendation when AUTOTUNE_* params exist"
    assert auto[0].evidence.samples["autotune_params"] == 2.0


def test_autotune_from_atun_message():
    recs = build_series(0.0, 1.0, RP=[0.1, 0.12], RD=[0.005, 0.006])
    log = make_flight_log({"ATUN": recs})
    auto = [r for r in analyze_tuning(log, area="autotune") if r.area == TuningArea.AUTOTUNE]
    assert auto, "expected an AUTOTUNE recommendation when an ATUN message exists"
    assert auto[0].evidence.samples["atun_messages"] == 2.0


def test_autotune_skipped_when_absent():
    log = make_flight_log({})
    assert analyze_tuning(log, area="autotune") == []


# --- area filter / combined --------------------------------------------------


def test_area_none_runs_all_analyses():
    imu = _gyro_imu_records(1000.0, 2048, tone_hz=80.0, amp=30.0)
    att = _att_records(50.0, 200, roll_err_deg=8.0)
    log = make_flight_log(
        {"IMU": imu, "ATT": att},
        params={"AUTOTUNE_AGGR": 0.1},
    )
    recs = analyze_tuning(log, area=None)
    areas = {r.area for r in recs}
    assert TuningArea.NOTCH in areas
    assert TuningArea.PID in areas
    assert TuningArea.AUTOTUNE in areas


def test_empty_log_returns_no_recommendations():
    log = make_flight_log({})
    assert analyze_tuning(log) == []


def test_area_filter_isolates_single_analysis():
    imu = _gyro_imu_records(1000.0, 2048, tone_hz=80.0, amp=30.0)
    att = _att_records(50.0, 200, roll_err_deg=8.0)
    log = make_flight_log({"IMU": imu, "ATT": att})
    # Only notch requested -> no PID even though ATT error is high.
    recs = analyze_tuning(log, area="notch")
    assert all(r.area == TuningArea.NOTCH for r in recs)
    assert recs, "expected the notch recommendation"
