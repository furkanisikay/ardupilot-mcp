"""Synthetic MAVLink telemetry logs (.tlog) for hermetic tests.

A tlog is what a ground station records: each MAVLink packet prefixed with its
reception time as a big-endian uint64 of microseconds (pymavlink ``mavlogfile``).
"""

from __future__ import annotations

import struct

from pymavlink.dialects.v20 import ardupilotmega as mav

VEHICLE = (1, 1)
GCS = (255, 190)


def build_tlog(
    path: str, messages: list[tuple[float, object]], source: dict[int, tuple[int, int]] | None = None
) -> str:
    """Write ``(time_s, MAVLink message)`` pairs; ``source`` maps a message index to (sys, comp)."""
    source = source or {}
    links: dict[tuple[int, int], mav.MAVLink] = {}
    with open(path, "wb") as fh:
        for i, (t, msg) in enumerate(messages):
            sysid, compid = source.get(i, VEHICLE)
            link = links.setdefault((sysid, compid), mav.MAVLink(None, srcSystem=sysid, srcComponent=compid))
            fh.write(struct.pack(">Q", int(round(t * 1e6))) + msg.pack(link))
    return path


def heartbeat(mode: int, armed: bool, *, gcs: bool = False) -> object:
    if gcs:
        return mav.MAVLink_heartbeat_message(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3)
    base = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
    return mav.MAVLink_heartbeat_message(
        mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, base, mode, 4, 3
    )
