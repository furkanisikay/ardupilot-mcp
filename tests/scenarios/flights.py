"""Labelled fault-injection scenarios in Copter-4.6.3's exact log format.

Every scenario is the same realistic quad flight — dual GPS, dual compass, three
IMUs, two EKF3 cores, one battery, logging starting 30 s after boot — with one
injected condition (or a benign variation of a clean flight), plus a ground-truth
label: which (check, severity band, title fragment) MUST fire. Anything else at
WARN or above is a false alarm. ``tests/test_scenarios.py`` runs the full
parse -> diagnose pipeline over each one.

Benign variations are the real-world traps that used to raise false alarms: a
receiver with no transmitter on the bench before arming, a 2S payload battery on
the second monitor, a gimbal servo on a spare output, the Cube's deliberately
unisolated third IMU.
"""

from __future__ import annotations

import math
import random
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .fw46 import Writer

BOOT_S = 30.0  # logging starts 30 s after boot (TimeUS origin is boot)
ARM_S, DISARM_S, END_S = 10.0, 100.0, 108.0
LAT0, LNG0, AMSL0 = -35.3632621, 149.1652374, 584.0


@dataclass(frozen=True)
class Expect:
    check: str
    min_severity: str  # info | warn | critical
    title: str = ""  # case-insensitive fragment of the finding title
    max_severity: str = "critical"


@dataclass(frozen=True)
class Scenario:
    name: str
    inject: dict[str, Any] = field(default_factory=dict)
    expect: tuple[Expect, ...] = ()
    duration: float = END_S


def _us(t: float) -> int:
    return int((BOOT_S + t) * 1e6)


BASE_PARAMS: dict[str, float] = {
    "FRAME_CLASS": 1,
    "FRAME_TYPE": 1,
    "BATT_MONITOR": 4,
    "BATT_LOW_VOLT": 14.0,
    "BATT_CRT_VOLT": 13.2,
    "BATT_CAPACITY": 5200,
    "ARMING_CHECK": 1,
    "FS_THR_ENABLE": 1,
    "GPS1_TYPE": 1,
    "GPS2_TYPE": 1,
    "MOT_THST_HOVER": 0.38,
    "MOT_SPIN_ARM": 0.08,
    "MOT_SPIN_MIN": 0.12,
    "ATC_RAT_RLL_P": 0.12,
    "ATC_RAT_PIT_P": 0.12,
    "ATC_ACCEL_R_MAX": 110000,
    "ATC_ACCEL_P_MAX": 110000,
    "ATC_ACCEL_Y_MAX": 27000,
    "COMPASS_OFS_X": 45,
    "COMPASS_OFS_Y": -80,
    "COMPASS_OFS_Z": 120,
    "COMPASS_OFS2_X": 10,
    "COMPASS_OFS2_Y": 22,
    "COMPASS_OFS2_Z": -35,
    "COMPASS_USE": 1,
    "COMPASS_USE2": 1,
    "COMPASS_DEV_ID": 97539,
    "COMPASS_DEV_ID2": 590114,
    "INS_ACCSCAL_X": 1.001,
    "INS_ACCSCAL_Y": 0.998,
    "INS_ACCSCAL_Z": 1.003,
    "INS_ACCOFFS_X": 0.02,
    "INS_ACCOFFS_Y": -0.05,
    "INS_ACCOFFS_Z": 0.11,
    "SERVO1_FUNCTION": 33,
    "SERVO2_FUNCTION": 34,
    "SERVO3_FUNCTION": 35,
    "SERVO4_FUNCTION": 36,
    "SERVO5_FUNCTION": 0,
}


def write_flight(path: str, inject: dict[str, Any], *, duration: float = END_S, seed: int = 1) -> None:
    """Write one ~2-minute LOITER flight with ``inject`` applied."""
    rng = random.Random(seed)
    w = Writer(path)
    w.write("MSG", TimeUS=_us(0), Message="ArduCopter V4.6.3 (3fc7011a)")
    w.write("MSG", TimeUS=_us(0), Message="CubeOrange 00350040 32335105 33363132")
    w.write("MSG", TimeUS=_us(0), Message="Frame: QUAD/X")
    params = {**BASE_PARAMS, **inject.get("params", {})}
    for k, v in params.items():
        w.write("PARM", TimeUS=_us(0.01), Name=k, Value=v, Default=v)
    w.write("MODE", TimeUS=_us(0.1), Mode=0, ModeNum=0, Rsn=0)

    att_err: Callable[[float], float] = inject.get("att_err", lambda _t: 0.0)
    pm_nlon: Callable[[float], int] = inject.get("pm_nlon", lambda _t: rng.randint(0, 3))
    dt = 0.01  # 100 Hz master tick
    clip = [0, 0, 0]
    mah = 0.0
    armed_logged = disarmed_logged = False
    for i in range(int(duration / dt)):
        t = i * dt
        armed = ARM_S <= t < DISARM_S
        flying = ARM_S + 3 <= t < DISARM_S - 3
        if armed and not armed_logged:
            w.write("EV", TimeUS=_us(t), Id=10)
            w.write("MODE", TimeUS=_us(t), Mode=5, ModeNum=5, Rsn=1)  # LOITER
            armed_logged = True
        if t >= DISARM_S and not disarmed_logged:
            w.write("EV", TimeUS=_us(t), Id=18)  # LAND_COMPLETE
            w.write("EV", TimeUS=_us(t), Id=11)  # DISARMED
            disarmed_logged = True
        alt = min(20.0, (t - ARM_S - 3) * 2.0, (DISARM_S - 3 - t) * 2.0) if flying else 0.0
        thr = 0.0 if not armed else (0.38 if flying else 0.12)
        if flying:
            thr += 0.04 * math.sin(t * 0.7)
        curr = 0.5 if not armed else 4 + 60 * thr
        mah += curr * dt / 3.6

        if i % 4 == 0:  # 25 Hz: ATT, IMU x3
            dr = 8 * math.sin(t * 0.4) if flying else 0.0
            dp = 6 * math.sin(t * 0.3 + 1) if flying else 0.0
            dy = (t * 3) % 360 - 180
            w.write(
                "ATT",
                TimeUS=_us(t),
                DesRoll=dr,
                Roll=dr + rng.gauss(0, 0.6) + att_err(t),
                DesPitch=dp,
                Pitch=dp + rng.gauss(0, 0.6),
                DesYaw=dy,
                Yaw=dy + rng.gauss(0, 1.0),
                AEKF=3,
            )
            for inst in range(3):
                w.write(
                    "IMU",
                    TimeUS=_us(t),
                    I=inst,
                    GyrX=rng.gauss(0, 0.02),
                    GyrY=rng.gauss(0, 0.02),
                    GyrZ=rng.gauss(0, 0.01),
                    AccX=rng.gauss(0, 0.3),
                    AccY=rng.gauss(0, 0.3),
                    AccZ=-9.81 + rng.gauss(0, 0.5),
                    T=45 + inst,
                    GH=1,
                    AH=1,
                    GHz=25,
                    AHz=25,
                )
        if i % 10 != 0:
            continue
        # -- 10 Hz streams
        for inst in range(3):
            base = 8.0 if flying else 0.5
            vz = base + rng.gauss(0, 1.5)
            vib = inject.get("vibe")
            if vib and inst == vib["imu"] and vib["t0"] <= t <= vib["t1"]:
                vz = vib["level"] + rng.gauss(0, 4)
                clip[inst] += vib["clips_per_s"] // 10
            w.write(
                "VIBE",
                TimeUS=_us(t),
                IMU=inst,
                VibeX=abs(base * 0.8 + rng.gauss(0, 1)),
                VibeY=abs(base * 0.8 + rng.gauss(0, 1)),
                VibeZ=abs(vz),
                Clip=clip[inst],
            )
        for inst, (field_mg, ofs) in enumerate(((480.0, (45, -80, 120)), (300.0, (10, 22, -35)))):
            m = field_mg
            ci = inject.get("compass_interference")
            if ci and inst == ci["inst"] and armed:
                m = field_mg * (1 + ci["gain"] * (thr - 0.12) / 0.3)
            hdg = math.radians((t * 3) % 360)
            w.write(
                "MAG",
                TimeUS=_us(t),
                I=inst,
                MagX=m * 0.45 * math.cos(hdg) + rng.gauss(0, 3),
                MagY=m * 0.45 * math.sin(hdg) + rng.gauss(0, 3),
                MagZ=m * 0.89 + rng.gauss(0, 3),
                OfsX=ofs[0],
                OfsY=ofs[1],
                OfsZ=ofs[2],
                Health=1,
                S=int(t * 1000),
            )
        for core in range(2):
            sv = 0.1 + abs(rng.gauss(0, 0.05))
            ek = inject.get("ekf")
            if ek and core == ek["core"] and ek["t0"] <= t <= ek["t1"]:
                sv = ek["ratio"]
            w.write(
                "XKF4",
                TimeUS=_us(t),
                C=core,
                SV=sv,
                SP=0.1 + abs(rng.gauss(0, 0.05)),
                SH=0.2 + abs(rng.gauss(0, 0.05)),
                SM=0.05 + abs(rng.gauss(0, 0.03)),
                PI=inject.get("primary_core", 0),
            )
        volt = 16.6 - 1.6 * (t / END_S) - 0.03 * curr
        bd = inject.get("brownout")
        if bd and t >= bd["t0"]:
            volt -= bd["drop"]
        w.write(
            "BAT",
            TimeUS=_us(t),
            Inst=0,
            Volt=volt,
            VoltR=volt + 0.03 * curr,
            Curr=curr,
            CurrTot=mah,
            EnrgTot=mah * 0.016,
            Temp=30,
            Res=0.02,
            RemPct=100 - mah / 52,
            H=1,
        )
        if inject.get("bat2"):
            v2 = inject["bat2"]
            w.write("BAT", TimeUS=_us(t), Inst=1, Volt=v2, VoltR=v2, Curr=0.4, CurrTot=t * 0.1, Temp=30, H=1)
        rc = [1500, 1500, 1000 + int(thr * 1000) if armed else 1000, 1500, 1000, 1000, 1500, 1500] + [0] * 6
        flags = 1  # RCI2.Flags HAS_VALID_INPUT
        if t < inject.get("rc_link_at", 0.0):
            rc, flags = [0] * 14, 0  # receiver outputs nothing until the TX is switched on (bench)
        rl = inject.get("rc_loss")
        if rl and rl[0] <= t <= rl[1]:
            rc, flags = [0] * 14, 2  # no valid input, IN_RC_FAILSAFE
        w.write("RCIN", TimeUS=_us(t), **{f"C{k + 1}": v for k, v in enumerate(rc)})
        if not inject.get("no_rci2"):
            w.write("RCI2", TimeUS=_us(t), Flags=flags)
        mot = 1000 + int(thr * 1000) if armed else 1000
        mo = [mot + int(rng.gauss(0, 15)) if armed else 1000 for _ in range(4)]
        mh = inject.get("motor_hot")
        if mh and armed:
            mo[mh["motor"]] = min(2000, mo[mh["motor"]] + mh["extra"])
        out = mo + [inject.get("c5", 0)] + [0] * 9
        w.write("RCOU", TimeUS=_us(t), **{f"C{k + 1}": v for k, v in enumerate(out)})
        vcc = 5.05 + rng.gauss(0, 0.01)
        vd = inject.get("vcc_dip")
        if vd and vd[0] <= t <= vd[1]:
            vcc = vd[2]
        w.write("POWR", TimeUS=_us(t), Vcc=vcc, VServo=0.1, Flags=3)
        w.write(
            "CTUN",
            TimeUS=_us(t),
            ThI=thr,
            ThO=thr,
            ThH=0.38,
            DAlt=alt,
            Alt=alt + rng.gauss(0, 0.1),
            BAlt=alt + rng.gauss(0, 0.2),
        )
        w.write("POS", TimeUS=_us(t), Lat=LAT0, Lng=LNG0, Alt=AMSL0 + alt, RelHomeAlt=alt, RelOriginAlt=alt)

        if i % 20 == 0:  # 5 Hz GPS x2
            for inst in range(2):
                status, sats, hdop = 3, 15 - inst * 2, 0.7 + inst * 0.1
                for gi in inject.get("gps", []):
                    if inst == gi["inst"] and gi["t0"] <= t <= gi["t1"]:
                        status, sats, hdop = gi["status"], gi["sats"], gi["hdop"]
                w.write(
                    "GPS",
                    TimeUS=_us(t),
                    I=inst,
                    Status=status,
                    GMS=int(t * 1000) + 300_000_000,
                    GWk=2390,
                    NSats=sats,
                    HDop=hdop,
                    Lat=LAT0,
                    Lng=LNG0,
                    Alt=AMSL0 + alt,
                    Spd=0.3,
                    U=1,
                )
        if i % 100 == 0:  # 1 Hz PM: a few long loops per window is normal on a busy H7
            w.write("PM", TimeUS=_us(t), LR=400, NLon=pm_nlon(t), NL=400, MaxT=3100, Mem=65535, Load=420)
    for when, (subsys, ecode) in inject.get("err", []):
        w.write("ERR", TimeUS=_us(when), Subsys=subsys, ECode=ecode)
    w.close()


_NO_FIX = {"status": 1, "sats": 3, "hdop": 99.99}

SCENARIOS: tuple[Scenario, ...] = (
    Scenario("clean_flight"),
    Scenario("clean_tx_off_on_bench", {"rc_link_at": 6.0}),
    Scenario("clean_tx_off_on_bench_no_rci2", {"rc_link_at": 6.0, "no_rci2": True}),
    Scenario("clean_payload_bat2_7v4", {"bat2": 7.4}),
    Scenario("clean_gimbal_servo_c5", {"c5": 1500}),
    # The Cube's third IMU is deliberately unisolated: noticeable, never CRITICAL on its own.
    Scenario(
        "clean_cube_imu2_unisolated",
        {"vibe": {"imu": 2, "t0": 15, "t1": 95, "level": 40, "clips_per_s": 0}},
        (Expect("vibration", "info", "IMU2", max_severity="warn"),),
    ),
    # A dead second receiver is a real hardware fault worth a WARN — never a CRITICAL.
    Scenario(
        "gps2_never_fixes",
        {"gps": [{"inst": 1, "t0": 0, "t1": 999, **_NO_FIX}]},
        (Expect("gps", "warn", "GPS2", max_severity="warn"),),
    ),
    Scenario(
        "fault_vibe_clipping_imu0",
        {"vibe": {"imu": 0, "t0": 40, "t1": 80, "level": 62, "clips_per_s": 30}},
        (Expect("vibration", "critical", "clipping"), Expect("vibration", "critical", "VibeZ")),
    ),
    # GPS2 keeps a fix and GPS_AUTO_SWITCH defaults to UseBest: a receiver fault, not a nav loss.
    Scenario(
        "fault_gps1_loss_gps2_ok",
        {"gps": [{"inst": 0, "t0": 50, "t1": 60, **_NO_FIX}]},
        (Expect("gps", "warn", "GPS1", max_severity="warn"),),
    ),
    Scenario(
        "fault_gps1_loss_no_autoswitch",
        {"gps": [{"inst": 0, "t0": 50, "t1": 60, **_NO_FIX}], "params": {"GPS_AUTO_SWITCH": 0}},
        (Expect("gps", "critical", "GPS1"),),
    ),
    Scenario(
        "fault_both_gps_lost",
        {"gps": [{"inst": 0, "t0": 50, "t1": 60, **_NO_FIX}, {"inst": 1, "t0": 50, "t1": 60, **_NO_FIX}]},
        (Expect("gps", "critical", "GPS1"), Expect("gps", "critical", "GPS2")),
    ),
    Scenario(
        "fault_compass_interference",
        {"compass_interference": {"inst": 0, "gain": 0.9}},
        (Expect("compass", "warn", "interference"),),
    ),
    Scenario(
        "fault_ekf_velocity_reject",
        {"ekf": {"core": 0, "t0": 55, "t1": 58, "ratio": 1.6}},
        (Expect("ekf", "critical", "velocity"),),
    ),
    # Same rejection on the standby core: informational only.
    Scenario(
        "standby_ekf_core_strains",
        {"ekf": {"core": 1, "t0": 55, "t1": 58, "ratio": 1.6}},
        (Expect("ekf", "info", "velocity", max_severity="info"),),
    ),
    Scenario(
        "fault_battery_brownout", {"brownout": {"t0": 60.0, "drop": 4.5}}, (Expect("power", "critical"),)
    ),
    Scenario("fault_rc_loss_in_flight", {"rc_loss": (50.0, 54.0)}, (Expect("rcin", "critical"),)),
    Scenario(
        "fault_rc_loss_legacy_no_rci2",
        {"rc_loss": (50.0, 54.0), "no_rci2": True},
        (Expect("rcin", "critical"),),
    ),
    Scenario(
        "fault_motor1_hot",
        {"motor_hot": {"motor": 0, "extra": 320}},
        (Expect("motors", "critical", "imbalance"),),
    ),
    Scenario(
        "fault_attitude_loss",
        {"att_err": lambda t: 40.0 if 70 <= t <= 73 else 0.0},
        (Expect("attitude", "critical"),),
    ),
    Scenario("fault_fc_vcc_brownout", {"vcc_dip": (60.0, 61.5, 4.25)}, (Expect("power", "warn", "Vcc"),)),
    Scenario("fault_cpu_overload", {"pm_nlon": lambda _t: 120}, (Expect("timing", "warn", "long loop"),)),
    Scenario("fault_crash_err", {"err": [(80.0, (12, 1))]}, (Expect("events", "critical", "CRASH"),)),
    # Power lost at t=70 s in flight: the log simply stops while armed.
    Scenario(
        "fault_power_loss_in_flight",
        {},
        (Expect("integrity", "warn", "ended while armed"),),
        duration=70.0,
    ),
)
