"""Generate a synthetic but realistic multi-fault ArduCopter crash log.

Produces a ``.bin`` that deliberately trips most checks so you can see a full
diagnosis end to end. Uses the project's synthetic DataFlash writer (a dev tool),
so no real hardware or large fixture is needed.

Run:  python examples/generate_demo_log.py [out.bin]
"""

from __future__ import annotations

import math
import os
import sys

# Make the repo root importable when run directly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.synth_bin import build_bin  # noqa: E402


def build_crash_log(path: str) -> str:
    msgs: list[tuple[str, dict]] = []

    def at(t: float) -> int:
        return int(t * 1e6)

    # --- IMU @ 1000 Hz for 2 s with an 80 Hz gyro noise component (notch demo) ---
    fs = 1000.0
    n_imu = 2000
    for i in range(n_imu):
        t = i / fs
        noise = 0.6 * math.sin(2 * math.pi * 80.0 * t)
        msgs.append(("IMU", {"TimeUS": at(t), "GyrX": noise, "GyrY": 0.4 * noise, "GyrZ": 0.0,
                            "AccX": 0.0, "AccY": 0.0, "AccZ": -9.81}))

    # --- 30 s flight streams ---
    # VIBE @ 10 Hz: Z vibration ramps into the danger zone; clipping accumulates late.
    clip = 0
    for i in range(300):
        t = i / 10.0
        vz = 12.0 + (60.0 * (t / 30.0))  # ~12 -> ~72 m/s^2
        if t > 24:
            clip += 8
        msgs.append(("VIBE", {"TimeUS": at(t), "VibeX": 14.0, "VibeY": 13.0, "VibeZ": vz,
                            "Clip0": clip, "Clip1": 0, "Clip2": 0}))

    # ATT @ 50 Hz: clean tracking until t=20s, then roll diverges 35 deg for 3 s.
    for i in range(1500):
        t = i / 50.0
        des_roll = 5.0 * math.sin(2 * math.pi * 0.2 * t)
        roll = des_roll
        if 20.0 <= t <= 23.0:
            roll = des_roll + 35.0  # loss of attitude control
        msgs.append(("ATT", {"TimeUS": at(t), "DesRoll": des_roll, "Roll": roll,
                            "DesPitch": 0.0, "Pitch": 0.0, "DesYaw": 90.0, "Yaw": 90.0}))

    # GPS @ 5 Hz: 3D fix, lost (Status 1) between 22 and 24 s.
    for i in range(150):
        t = i / 5.0
        status = 1 if 22.0 <= t <= 24.0 else 3
        hdop = 3.5 if 21.0 <= t <= 25.0 else 0.8
        msgs.append(("GPS", {"TimeUS": at(t), "Status": status, "NSats": 12 if status == 3 else 4,
                            "HDop": hdop, "Alt": 30.0 + t, "Spd": 2.0, "Yaw": 90.0}))

    # BAT @ 5 Hz: sags from 16.8 V toward a critical 13.x V.
    for i in range(150):
        t = i / 5.0
        volt = 16.8 - (3.6 * (t / 30.0))  # 16.8 -> ~13.2
        msgs.append(("BAT", {"TimeUS": at(t), "Volt": volt, "Curr": 35.0 + 20.0 * math.sin(t),
                            "CurrTot": 1000.0 * (t / 30.0)}))

    # RCOU @ 10 Hz: motors climb toward saturation as it struggles late.
    for i in range(300):
        t = i / 10.0
        base = 1450 + int(450 * (t / 30.0))
        sat = 1990 if t > 24 else base
        msgs.append(("RCOU", {"TimeUS": at(t), "C1": sat, "C2": base, "C3": base, "C4": sat}))

    # XKF4 @ 5 Hz: EKF velocity variance spikes past 1.0 around the upset.
    for i in range(150):
        t = i / 5.0
        sv = 1.3 if 21.5 <= t <= 24.0 else 0.2
        msgs.append(("XKF4", {"TimeUS": at(t), "SV": sv, "SP": 0.3, "SH": 0.2, "SM": 0.2, "SVT": 0.0}))

    # Discrete events.
    msgs.append(("MODE", {"TimeUS": at(0.0), "Mode": 0, "ModeNum": 0, "Rsn": 1}))     # STABILIZE
    msgs.append(("MODE", {"TimeUS": at(2.0), "Mode": 5, "ModeNum": 5, "Rsn": 1}))     # LOITER
    msgs.append(("MODE", {"TimeUS": at(25.0), "Mode": 6, "ModeNum": 6, "Rsn": 4}))    # RTL
    msgs.append(("ERR", {"TimeUS": at(22.0), "Subsys": 16, "ECode": 2}))             # EKFCHECK
    msgs.append(("ERR", {"TimeUS": at(25.0), "Subsys": 6, "ECode": 1}))              # FAILSAFE_BATT
    msgs.append(("EV", {"TimeUS": at(0.5), "Id": 10}))                                # ARMED
    msgs.append(("EV", {"TimeUS": at(29.0), "Id": 11}))                               # DISARMED

    params = {
        "INS_HNTCH_ENABLE": 0.0,   # notch OFF -> recommend enabling
        "BATT_CRT_VOLT": 13.5,
        "BATT_LOW_VOLT": 14.4,
        "ATC_RAT_RLL_P": 0.135,
        "FRAME_CLASS": 1.0,        # Quad
        "FRAME_TYPE": 1.0,         # X
        "MOT_THST_HOVER": 0.55,    # hovers at 55% -> modest power margin
        "BATT_CAPACITY": 5200.0,
    }
    build_bin(path, msgs, params=params, firmware="ArduCopter V4.5.7 (synthetic demo)")
    return path


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "demo_crash.bin")
    build_crash_log(out)
    print(f"wrote {out} ({os.path.getsize(out)} bytes)")
