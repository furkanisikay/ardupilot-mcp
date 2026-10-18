"""Documented parameter metadata for the log's firmware, bundled for offline use.

ArduPilot publishes each release's parameter definitions as ``apm.pdef.xml``
(https://autotest.ardupilot.org/Parameters/versioned/). ``scripts/gen_param_meta.py``
reduces one to facts (documented range, units, listed value codes, bitmask bit
positions, ReadOnly / RebootRequired) in ``data/params/<Vehicle>-<version>.json.gz``;
no description or label is copied. The server reads them without network access.

The metadata is documentation, not a constraint the firmware enforces: a Range is
the recommended span (some defaults, such as 0 for "disabled", lie outside it), and a
values list often names only the common choices (a BAUD parameter also takes the raw
rate). It is shown next to a parameter for interpretation and never raises a finding.
"""

from __future__ import annotations

import gzip
import json
import re
from dataclasses import dataclass
from functools import lru_cache
from importlib import resources

from .model import ParamDoc

# Vehicle kind -> vehicle name of the bundled files (the archive's directory names;
# the Copter definitions include the traditional-helicopter parameters).
_VEHICLE = {"copter": "Copter", "heli": "Copter", "plane": "Plane", "rover": "Rover", "sub": "Sub"}
_FILE = re.compile(r"^(?P<vehicle>[A-Za-z]+)-(?P<version>\d+\.\d+\.\d+)\.json\.gz$")
_VERSION = re.compile(r"(\d+)\.(\d+)\.(\d+)")

Version = tuple[int, int, int]


@dataclass(frozen=True)
class ParamCatalog:
    vehicle: str
    version: Version
    match: str  # "exact", "same release series", "nearest older release", "nearest newer release"
    source: str
    params: dict[str, dict]

    @property
    def label(self) -> str:
        return f"{self.vehicle} {'.'.join(map(str, self.version))} ({self.match})"

    def doc(self, name: str) -> ParamDoc | None:
        d = self.params.get(name)
        if d is None:
            return None
        return ParamDoc(
            range=d.get("r"),
            units=d.get("u"),
            values=d.get("v"),
            bits=d.get("b"),
            read_only=bool(d.get("ro")),
            reboot_required=bool(d.get("rb")),
        )


def _parse_version(text: str) -> Version | None:
    m = _VERSION.search(text)
    return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None


@lru_cache(maxsize=1)
def bundled() -> dict[str, list[Version]]:
    """Vehicle -> bundled release versions, oldest first."""
    out: dict[str, list[Version]] = {}
    for entry in resources.files(__package__).joinpath("data", "params").iterdir():
        m = _FILE.match(entry.name)
        if m:
            v = _parse_version(m.group("version"))
            assert v is not None
            out.setdefault(m.group("vehicle"), []).append(v)
    return {k: sorted(v) for k, v in out.items()}


@lru_cache(maxsize=4)
def _load(vehicle: str, version: Version) -> tuple[str, dict[str, dict]]:
    name = f"{vehicle}-{'.'.join(map(str, version))}.json.gz"
    raw = resources.files(__package__).joinpath("data", "params", name).read_bytes()
    doc = json.loads(gzip.decompress(raw))
    return str(doc["source"]), dict(doc["params"])


def choose(versions: list[Version], want: Version) -> tuple[Version, str] | None:
    """The bundled release that best describes ``want``: same major version only.

    Design choice: parameters are stable within a release series (major.minor), so the
    bundled release of the same series stands in for any of its patch releases; across
    series the nearest older release is preferred (a newer one documents parameters the
    log's firmware may not have yet).
    """
    same_major = [v for v in versions if v[0] == want[0]]
    if not same_major:
        return None
    if want in same_major:
        return want, "exact"
    series = [v for v in same_major if v[:2] == want[:2]]
    if series:
        return series[-1], "same release series"
    older = [v for v in same_major if v < want]
    if older:
        return older[-1], "nearest older release"
    return same_major[0], "nearest newer release"


def catalog_for(firmware_version: str | None, vehicle_kind: str | None) -> ParamCatalog | None:
    """The bundled parameter metadata for this firmware, or None when none fits."""
    vehicle = _VEHICLE.get(vehicle_kind or "")
    want = _parse_version(firmware_version or "")
    if vehicle is None or want is None:
        return None
    picked = choose(bundled().get(vehicle, []), want)
    if picked is None:
        return None
    version, match = picked
    source, params = _load(vehicle, version)
    return ParamCatalog(vehicle=vehicle, version=version, match=match, source=source, params=params)
