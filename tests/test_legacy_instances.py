"""Pre-4.1 per-instance messages (IMU2, MAG2, NKF6, NK11, ...) folded into their base name."""

from __future__ import annotations

import numpy as np

from ardupilot_mcp.checks.ekf import EKFCheck
from ardupilot_mcp.parser import parse_bin
from tests.helpers import build_series, make_flight_log
from tests.synth_bin import build_bin


def _imu(t0: float, gyr: float, n: int = 4) -> list[dict]:
    return build_series(t0, 10.0, GyrX=[gyr] * n, GyrY=[0.0] * n, GyrZ=[0.0] * n)


def test_secondary_imus_become_instances_in_time_order():
    log = make_flight_log({"IMU": _imu(0.0, 1.0), "IMU2": _imu(0.05, 2.0), "IMU3": _imu(0.02, 3.0)})
    assert log.merged_legacy == {"IMU": ["IMU", "IMU2", "IMU3"]}
    assert log.instance_field("IMU") == "I"
    assert log.instances("IMU") == [0, 1, 2]
    assert list(log.series("IMU", "GyrX", instance=1)[1]) == [2.0] * 4
    t, _ = log.series("IMU", "GyrX")
    assert np.all(np.diff(t) >= 0)  # interleaved by time, as 4.1+ logs them
    # The member tables stay available under their own names.
    assert log.count("IMU2") == 4


def test_ekf_cores_fold_from_their_numbered_messages():
    ratios = {"SV": [0.1] * 3, "SP": [0.1] * 3, "SH": [0.1] * 3, "SM": [0.1] * 3, "PI": [0] * 3}
    log = make_flight_log(
        {
            "NKF4": build_series(0.0, 10.0, **ratios),
            "NKF9": build_series(0.0, 10.0, **ratios),  # core 1
            "NK14": build_series(0.0, 10.0, **ratios),  # core 2 (4.0)
        }
    )
    assert log.instances("NKF4") == [0, 1, 2]
    assert log.instance_field("NKF4") == "C"


def test_a_modern_log_is_left_alone():
    # 4.1+ already carries the instance field; an unrelated IMU2 must not be folded in.
    imu = [{**r, "I": 0} for r in _imu(0.0, 1.0)]
    log = make_flight_log({"IMU": imu, "IMU2": _imu(0.0, 2.0)})
    assert log.merged_legacy == {}
    assert log.count("IMU") == 4


def test_a_member_with_other_fields_is_not_merged():
    other = build_series(0.0, 10.0, GyrX=[1.0] * 4, Extra=[0.0] * 4)
    log = make_flight_log({"IMU": _imu(0.0, 1.0), "IMU2": other})
    assert log.merged_legacy == {}
    assert log.instances("IMU") == [0]


def test_a_missing_middle_member_keeps_the_instance_numbers():
    # IMU3 without IMU2: the third IMU is still instance 2.
    log = make_flight_log({"IMU": _imu(0.0, 1.0), "IMU3": _imu(0.0, 3.0)})
    assert log.instances("IMU") == [0, 2]


def test_parsed_legacy_log_feeds_the_checks_per_instance(tmp_path):
    msgs: list[tuple[str, dict]] = []
    for i in range(30):
        t = int(i * 100_000)
        msgs.append(("XKF4", {"TimeUS": t, "SV": 0.1, "SP": 0.1, "SH": 0.1, "SM": 0.1, "SVT": 0.1}))
        msgs.append(("XKF9", {"TimeUS": t, "SV": 1.5, "SP": 0.1, "SH": 0.1, "SM": 0.1, "SVT": 0.1}))
        msgs.append(("IMU", {"TimeUS": t, "GyrX": 0.0, "AccZ": -9.8}))
        msgs.append(("IMU2", {"TimeUS": t, "GyrX": 0.0, "AccZ": -9.8}))
    log = parse_bin(build_bin(str(tmp_path / "legacy.bin"), msgs, firmware="APM:Copter V3.5.7 (synthetic)"))
    assert log.instances("IMU") == [0, 1]
    assert log.instances("XKF4") == [0, 1]
    findings = EKFCheck().run(log)
    # Core 1's velocity ratio was over 1.0; it is reported for that core.
    assert any("(core 1)" in f.title and "velocity" in f.title for f in findings)
