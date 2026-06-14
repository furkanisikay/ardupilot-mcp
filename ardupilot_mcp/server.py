"""FastMCP server exposing the ArduPilot log-diagnosis tools.

Every tool is read-only (``readOnlyHint=True``): the server parses offline
``.bin`` files and never connects to a vehicle, arms, or writes anything. That
is the whole safety story for this MVP — it physically cannot move a drone.

Parsed logs are cached by (absolute path, mtime) so a conversation that calls
several tools on the same log parses it once.
"""

from __future__ import annotations

import math
import os
from fnmatch import fnmatch

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations

from . import ardupilot_docs as docs
from . import ardupilot_meta as meta
from .checks import get_checks
from .flight_log import FlightLog
from .model import (
    CheckCatalog,
    CheckInfo,
    DiagnosisReport,
    EventRecord,
    EventTimeline,
    LogIntegrity,
    LogSummary,
    ParamResult,
    TimeseriesResult,
    TuningArea,
    TuningReport,
    VehicleProfile,
)
from .orchestrator import diagnose
from .param_file import parse_param_file
from .parser import parse_bin
from .profile import build_profile

# A full ArduPilot config is ~800-1400 params; far fewer suggests an incomplete dump.
_LOW_PARAM_COUNT = 150

mcp = FastMCP("ardupilot-mcp")

_READONLY = ToolAnnotations(readOnlyHint=True, openWorldHint=False)

# (abs_path -> (mtime, FlightLog)) cache so repeated tools don't re-parse.
_CACHE: dict[str, tuple[float, FlightLog]] = {}


def _load(path: str) -> FlightLog:
    apath = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(apath):
        raise FileNotFoundError(f"log file not found: {path}")
    mtime = os.path.getmtime(apath)
    cached = _CACHE.get(apath)
    if cached is not None and cached[0] == mtime:
        return cached[1]
    log = parse_bin(apath)
    _CACHE[apath] = (mtime, log)
    return log


def _rec_time(rec: dict) -> float | None:
    """The normalised flight-relative timestamp of a record, as a float or None."""
    t = rec.get("timestamp")
    return float(t) if isinstance(t, (int, float)) else None


def _in_window(t: float | None, start_s: float | None, end_s: float | None) -> bool:
    if t is None:
        return start_s is None and end_s is None
    if start_s is not None and t < start_s:
        return False
    if end_s is not None and t > end_s:
        return False
    return True


@mcp.tool(annotations=_READONLY)
def analyze_log(path: str) -> DiagnosisReport:
    """Run the full deterministic diagnostic suite over an ArduPilot .bin log.

    Returns severity-ranked findings (each with concrete evidence AND official
    ArduPilot doc links), per-check results, a plain-language summary, and
    `guidance`. Read the cited references for the log's firmware version, call
    `vehicle_profile` for the airframe's physical configuration, and fold in any
    real-world details the user gives — then explain and cite sources. Every
    finding is grounded in the log; do not invent causes beyond them.
    """
    return diagnose(_load(path))


@mcp.tool(annotations=_READONLY)
def log_summary(path: str) -> LogSummary:
    """Cheap orientation: vehicle, firmware, duration, message counts, flight modes, max altitude, integrity."""
    return _load(path).summary()


@mcp.tool(annotations=_READONLY)
def vehicle_profile(path: str) -> VehicleProfile:
    """Physical / architecture profile of the aircraft inferred from the log.

    Frame class & type, motor count, battery cells/capacity, and — the key physical
    insight — the hover throttle and resulting power margin / thrust-to-weight.
    Use this to reason about the craft physically and to combine with real-world
    details the user gives (weight, prop diameter, motor KV, wind).
    """
    return build_profile(_load(path))


@mcp.tool(annotations=_READONLY)
def list_events(
    path: str,
    kinds: list[str] | None = None,
    start_s: float | None = None,
    end_s: float | None = None,
) -> EventTimeline:
    """Decoded ERR/MODE/EV/MSG timeline. ``kinds`` filters families (ERR, MODE, EV, MSG); times are seconds."""
    log = _load(path)
    wanted = {k.upper() for k in kinds} if kinds else None
    events: list[EventRecord] = []

    if wanted is None or "ERR" in wanted:
        for r in log.get("ERR"):
            subsys = int(r.get("Subsys", 0))
            ecode = int(r.get("ECode", 0))
            name = meta.err_subsystem_name(subsys)
            label = f"{name} cleared" if ecode == 0 else f"{name} error (ECode {ecode})"
            events.append(
                EventRecord(
                    time_s=_rec_time(r), kind="ERR", name=label, raw={"Subsys": subsys, "ECode": ecode}
                )
            )
    if wanted is None or "MODE" in wanted:
        for r in log.get("MODE"):
            num = r.get("Mode", r.get("ModeNum"))
            if num is None:
                continue
            events.append(
                EventRecord(
                    time_s=_rec_time(r),
                    kind="MODE",
                    name=f"Mode -> {meta.mode_name(int(num))}",
                    raw={"Mode": int(num)},
                )
            )
    if wanted is None or "EV" in wanted:
        for r in log.get("EV"):
            eid = r.get("Id")
            if eid is None:
                continue
            events.append(
                EventRecord(time_s=_rec_time(r), kind="EV", name=meta.ev_name(int(eid)), raw={"Id": int(eid)})
            )
    if wanted is None or "MSG" in wanted:
        for r in log.get("MSG"):
            text = r.get("Message") or r.get("Msg")
            if isinstance(text, str):
                events.append(EventRecord(time_s=_rec_time(r), kind="MSG", name=text.strip()))

    events = [e for e in events if _in_window(e.time_s, start_s, end_s)]
    events.sort(key=lambda e: e.time_s if e.time_s is not None else math.inf)
    return EventTimeline(path=path, events=events, total=len(events))


@mcp.tool(annotations=_READONLY)
def query_timeseries(
    path: str,
    message_type: str,
    fields: list[str],
    start_s: float | None = None,
    end_s: float | None = None,
    max_points: int = 1000,
) -> TimeseriesResult:
    """Numeric time series for a message type and fields, for drilling into a finding.

    Downsamples (decimates) to at most ``max_points`` per field to keep payloads small.
    """
    log = _load(path)
    if not log.has(message_type):
        return TimeseriesResult(
            path=path,
            message_type=message_type,
            fields=fields,
            note=f"message type {message_type!r} not present in this log.",
        )
    times = log.times(message_type)
    # boolean mask for the time window
    if times.size and (start_s is not None or end_s is not None):
        lo = start_s if start_s is not None else -math.inf
        hi = end_s if end_s is not None else math.inf
        mask = (times >= lo) & (times <= hi)
    else:
        mask = None

    series: dict[str, list[float]] = {}
    for f in fields:
        t2, v = log.series(message_type, f)
        if mask is not None and t2.size == times.size:
            v = v[mask]
        series[f] = v.tolist()

    sel_times = times[mask] if mask is not None else times
    n = int(sel_times.size)
    stride = max(1, math.ceil(n / max_points)) if max_points and n > max_points else 1
    downsampled = stride > 1
    out_times = sel_times[::stride].tolist()
    out_series = {k: v[::stride] for k, v in series.items()}

    return TimeseriesResult(
        path=path,
        message_type=message_type,
        fields=fields,
        times_s=out_times,
        series=out_series,
        sample_count=n,
        downsampled=downsampled,
        note=(f"decimated by {stride}x ({n} -> {len(out_times)} points)" if downsampled else None),
    )


@mcp.tool(annotations=_READONLY)
def get_params(path: str, name_glob: str | None = None) -> ParamResult:
    """Parameter values recorded in the log's PARM dump. ``name_glob`` is a case-insensitive glob, e.g. 'INS_HNTCH_*'.

    Includes `total` (full param count), `metadata_url` (version-specific definitions to interpret values), and a
    `note` if the snapshot looks incomplete (truncated log) — in which case ask the user for their exported .param
    file and read it with `load_param_file`.
    """
    log = _load(path)
    all_params = log.params
    total = len(all_params)
    params = all_params
    if name_glob:
        pat = name_glob.upper()
        params = {k: v for k, v in params.items() if fnmatch(k.upper(), pat)}

    note: str | None = None
    if log.meta.integrity != LogIntegrity.OK:
        note = (
            "the log is truncated/partial, so this parameter snapshot may be incomplete — "
            "ask the user for their exported .param file and read it with load_param_file."
        )
    elif total < _LOW_PARAM_COUNT:
        note = (
            f"only {total} parameters were logged (a full ArduPilot config is ~800-1400); the PARM "
            "dump may be incomplete — consider load_param_file for the full config."
        )
    return ParamResult(
        path=path,
        params=dict(sorted(params.items())),
        matched=len(params),
        total=total,
        source="log",
        metadata_url=docs.versioned_param_doc_url(log.meta.firmware_version, log.meta.vehicle_kind),
        note=note,
    )


@mcp.tool(annotations=_READONLY)
def load_param_file(path: str, name_glob: str | None = None) -> ParamResult:
    """Load a user-exported ArduPilot parameter file (.param / .params).

    The authoritative *complete* configuration — use it when the log's PARM snapshot is truncated/incomplete,
    or to compare the flown config (get_params) against the user's current/intended config. ``name_glob`` filters.
    """
    apath = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(apath):
        raise FileNotFoundError(f"parameter file not found: {path}")
    all_params = parse_param_file(apath)
    total = len(all_params)
    params = all_params
    if name_glob:
        pat = name_glob.upper()
        params = {k: v for k, v in params.items() if fnmatch(k.upper(), pat)}
    note = (
        f"loaded {total} parameters from a parameter file"
        if total
        else ("no parameters parsed — is this a Mission Planner .param or QGC .params file?")
    )
    return ParamResult(
        path=path,
        params=dict(sorted(params.items())),
        matched=len(params),
        total=total,
        source="file",
        note=note,
    )


@mcp.tool(annotations=_READONLY)
def recommend_tuning(path: str, area: str | None = None) -> TuningReport:
    """Advisory tuning recommendations (harmonic notch from gyro FFT, PID, autotune). Recommendation ONLY — never applied."""
    log = _load(path)
    notes: list[str] = []
    recs = []
    try:
        from .tuning import analyze_tuning  # lazy: keep server importable if module is mid-build
    except Exception as exc:  # noqa: BLE001
        return TuningReport(path=path, recommendations=[], notes=[f"tuning module unavailable: {exc}"])

    sel: TuningArea | None = None
    if area:
        try:
            sel = TuningArea(area.lower())
        except ValueError:
            notes.append(f"unknown tuning area {area!r}; valid: notch, pid, autotune. Running all.")
    recs = analyze_tuning(log, sel.value if sel else None)
    for r in recs:
        if not r.references:
            r.references = docs.tuning_references(r.area.value, log.meta.vehicle_kind)
    return TuningReport(path=path, recommendations=recs, notes=notes)


@mcp.tool(annotations=_READONLY)
def list_checks() -> CheckCatalog:
    """List the registered diagnostic checks (the extensible catalogue)."""
    infos: list[CheckInfo] = [c.info() for c in get_checks()]
    return CheckCatalog(checks=infos, total=len(infos))
