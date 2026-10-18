"""Logging advisor: what to switch on so the next log can answer more.

A check that skips because its data was not logged is a lost diagnosis. For
Copter/Heli this maps each missing message to the ``LOG_BITMASK`` bit that logs it,
and advises it only when that bit is actually off (if the bit is on and the
message is still missing, the sensor is the problem, not the logging, and the
sensors check covers that). Plane/Rover use different bit numbers, so for them we
say nothing rather than guess.

Bit numbers: Copter-4.6.3 ArduCopter/defines.h (MASK_LOG_*); which bit gates which
message: ArduCopter/Copter.cpp (Log_Write_* scheduling) and system.cpp (GPS,
compass and baro log bits).
"""

from __future__ import annotations

from dataclasses import dataclass

from .flight_log import FlightLog
from .model import LoggingAdvice


@dataclass(frozen=True)
class _Rule:
    message: str
    bits: tuple[int, ...]  # any of these bits logs the message
    bit_name: str
    needed_for: str


# Copter-4.6.3: MASK_LOG_ATTITUDE_FAST=0, ATTITUDE_MED=1, GPS=2, PM=3, CTUN=4, RCIN=6,
# IMU=7, RCOUT=10, COMPASS=13, MOTBATT=17, IMU_FAST=18, IMU_RAW=19.
_COPTER_RULES: tuple[_Rule, ...] = (
    _Rule("VIBE", (7, 18, 19), "IMU (bit 7)", "vibration levels and accelerometer clipping"),
    _Rule("ATT", (1, 0), "ATTITUDE_MED (bit 1)", "attitude tracking (demanded vs achieved)"),
    _Rule("RCIN", (6,), "RCIN (bit 6)", "RC link loss and RC failsafe flags"),
    _Rule("RCOU", (10,), "RCOUT (bit 10)", "motor output balance, saturation and thrust loss"),
    _Rule("CTUN", (4,), "CTUN (bit 4)", "throttle/altitude control and compass interference vs throttle"),
    _Rule("PM", (3,), "PM (bit 3)", "scheduler load (long loops)"),
    _Rule("GPS", (2,), "GPS (bit 2)", "GPS fix, satellites and HDOP"),
    _Rule("MAG", (13,), "COMPASS (bit 13)", "compass field stability and interference"),
    _Rule("MOTB", (17,), "MOTBATT (bit 17)", "motor/battery limits (thrust boost, lift max)"),
)
_COPTER_KINDS = {"copter", "heli"}
# Below this IMU rate the gyro spectrum cannot show motor noise (notch.NOTCH_MIN_FS_HZ).
_FFT_MIN_HZ = 100.0


def logging_advice(log: FlightLog) -> list[LoggingAdvice]:
    if log.meta.vehicle_kind not in _COPTER_KINDS:
        return []
    mask_value = log.params.get("LOG_BITMASK")
    mask = int(mask_value) if mask_value is not None else None
    out: list[LoggingAdvice] = []
    for r in _COPTER_RULES:
        if log.has(r.message):
            continue
        if mask is not None and any(mask & (1 << b) for b in r.bits):
            continue  # logging was on; a silent sensor is the sensors check's business
        out.append(
            LoggingAdvice(
                missing=r.message,
                needed_for=r.needed_for,
                setting=f"LOG_BITMASK: enable {r.bit_name}",
                bitmask_bit=r.bits[0],
            )
        )
    # ISBH/ISBD are not loaded (parser._SKIP_TYPES), so the batch sampler is judged by its parameter.
    batch_on = (log.params.get("INS_LOG_BAT_MASK") or 0) > 0
    if log.has("IMU") and mask is not None and not mask & (1 << 18) and not batch_on:
        times = log.times("IMU", log.instances("IMU")[0] if log.instances("IMU") else None)
        if times.size > 2:
            dt = float((times[-1] - times[0]) / max(1, times.size - 1))
            if dt > 0 and 1.0 / dt < _FFT_MIN_HZ:
                out.append(
                    LoggingAdvice(
                        missing="IMU at loop rate",
                        needed_for="harmonic-notch frequency from the gyro spectrum (recommend_tuning)",
                        setting="LOG_BITMASK: enable IMU_FAST (bit 18), or INS_LOG_BAT_MASK=1 for the batch sampler",
                        bitmask_bit=18,
                    )
                )
    return out
