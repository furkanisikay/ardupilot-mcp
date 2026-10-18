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
from collections import OrderedDict
from dataclasses import dataclass
from fnmatch import fnmatch

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations

from . import ardupilot_docs as docs
from . import ardupilot_meta as meta
from .checks import get_checks
from .compare import compare_logs as compare_logs_impl
from .flight_log import FlightLog
from .forum_report import build_forum_report
from .model import (
    CheckCatalog,
    CheckInfo,
    DiagnosisReport,
    EventRecord,
    EventTimeline,
    ForumReport,
    IncidentTimeline,
    LogComparison,
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
from .param_meta import catalog_for
from .parser import parse_bin
from .profile import build_profile
from .timeline import build_timeline

# A full ArduPilot config is ~800-1400 params; far fewer suggests an incomplete dump.
_LOW_PARAM_COUNT = 150
# Design choice: documented metadata is attached when a call returns at most this many
# parameters (a full dump of ~1000 would bury the values; a name_glob narrows it).
_DOC_LIMIT = 200

mcp = FastMCP("ardupilot-mcp")

_READONLY = ToolAnnotations(readOnlyHint=True, openWorldHint=False)


@dataclass
class _Entry:
    key: tuple[float, int]  # (mtime, size) of the file when it was parsed
    log: FlightLog
    report: DiagnosisReport | None = None
    timeline: IncidentTimeline | None = None


# Parsed logs (and their diagnosis, once computed), most recently used last.
# Bounded, because a parsed log can take hundreds of MB and a long conversation may
# open many logs; a conversation usually works on one or two at a time.
_CACHE: OrderedDict[str, _Entry] = OrderedDict()
_CACHE_MAX = 2


def _entry(path: str) -> _Entry:
    apath = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(apath):
        raise FileNotFoundError(f"log file not found: {path}")
    st = os.stat(apath)
    key = (st.st_mtime, st.st_size)
    cached = _CACHE.get(apath)
    if cached is not None and cached.key == key:
        _CACHE.move_to_end(apath)
        return cached
    _CACHE.pop(apath, None)
    while len(_CACHE) >= _CACHE_MAX:
        _CACHE.popitem(last=False)
    entry = _Entry(key, parse_bin(apath))
    _CACHE[apath] = entry
    return entry


def _load(path: str) -> FlightLog:
    return _entry(path).log


def _report(entry: _Entry) -> DiagnosisReport:
    """The diagnosis of a cached log, computed once: analyze_log and incident_timeline share it."""
    if entry.report is None:
        entry.report = diagnose(entry.log)
    return entry.report


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
    return _report(_entry(path))


@mcp.tool(annotations=_READONLY)
def incident_timeline(path: str) -> IncidentTimeline:
    """What went wrong first, and what followed: the onset (first anomaly while armed), the impact (crash
    check, crash disarm, or the log ending while armed), and every firmware verdict, error, failsafe mode
    change and finding in time order, each tagged before_onset / onset / after_onset / impact / after_impact.
    Use it to answer "what failed first, and is the rest just the crash?". Time order only — never causation.
    """
    entry = _entry(path)
    if entry.timeline is None:
        entry.timeline = build_timeline(entry.log, _report(entry).findings)
    return entry.timeline


@mcp.tool(annotations=_READONLY)
def forum_report(path: str) -> ForumReport:
    """A Markdown reply to paste into a discuss.ardupilot.org thread: vehicle and firmware, what went
    wrong first (onset and impact), the findings with their corroborating log records, and a small text
    plot of the data behind each one. It contains no GPS positions and is returned as text (nothing is
    written to disk). Review it with the user and add what they saw before they post it.
    """
    entry = _entry(path)
    # The report's own incident summary (onset, impact, conditions) is all the reply needs.
    report = _report(entry)
    return ForumReport(path=path, markdown=build_forum_report(entry.log, report, report.incident))


@mcp.tool(annotations=_READONLY)
def compare_logs(path_a: str, path_b: str) -> LogComparison:
    """What changed between two logs of the same vehicle ("it flew fine last week")? Lists configuration
    parameters that differ (separately from values the autopilot learns/counts itself), parameters present
    in only one log, the firmware change, and flight metrics side by side (vibration, hover throttle,
    battery/board voltage minimums). Differences only — it does not judge which side is right.
    By convention A is the earlier/good flight and B the later/bad one.
    """
    return compare_logs_impl(_load(path_a), _load(path_b))


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
            num = r.get("ModeNum", r.get("Mode"))
            raw: dict[str, float | int | str]
            if isinstance(num, str):
                name, raw = meta.mode_name(num), {"Mode": num}
            elif isinstance(num, (int, float)):
                name, raw = meta.mode_name(int(num), log.meta.vehicle_kind), {"Mode": int(num)}
            else:
                continue
            events.append(EventRecord(time_s=_rec_time(r), kind="MODE", name=f"Mode -> {name}", raw=raw))
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
    instance: int | None = None,
) -> TimeseriesResult:
    """Numeric time series for a message type and fields, for drilling into a finding.

    Multi-instance messages (VIBE/IMU per IMU, GPS per receiver, MAG per compass, BAT per
    battery, XKF* per EKF core) are returned one instance at a time: pass ``instance``
    (default: the first one; see ``available_instances``). Values are aligned with
    ``times_s`` by index. Downsamples (decimates) to at most ``max_points`` points.
    """
    log = _load(path)
    if not log.has(message_type):
        return TimeseriesResult(
            path=path,
            message_type=message_type,
            fields=fields,
            note=f"message type {message_type!r} not present in this log.",
        )
    insts = log.instances(message_type)
    multi = log.instance_field(message_type) is not None
    notes: list[str] = []
    if multi and instance is None:
        instance = insts[0]
        if len(insts) > 1:
            notes.append(f"returning instance {instance} of {insts}; pass instance= for the others")
    if multi and instance not in insts:
        return TimeseriesResult(
            path=path,
            message_type=message_type,
            fields=fields,
            instance=instance,
            available_instances=insts,
            note=f"instance {instance} not present; available: {insts}.",
        )
    present = set(log.fields(message_type))
    missing = [f for f in fields if f not in present]
    if missing:
        notes.append(f"field(s) {missing} not in {message_type}; available: {sorted(present)}")

    times, cols = log.aligned(message_type, fields, instance if multi else None)
    lo = start_s if start_s is not None else -math.inf
    hi = end_s if end_s is not None else math.inf
    win = (times >= lo) & (times <= hi)
    n = int(win.sum())
    # max_points <= 0 means "no decimation".
    cap = max_points if max_points > 0 else n
    stride = math.ceil(n / cap) if n > cap else 1
    sel_t = times[win][::stride]
    series = {f: [_num_or_none(v) for v in cols[f][win][::stride]] for f in fields}
    if stride > 1:
        notes.append(f"decimated by {stride}x ({n} -> {sel_t.size} points)")
    return TimeseriesResult(
        path=path,
        message_type=message_type,
        fields=fields,
        instance=instance if multi else None,
        available_instances=insts if multi else [],
        times_s=sel_t.tolist(),
        series=series,
        sample_count=n,
        downsampled=stride > 1,
        note="; ".join(notes) or None,
    )


def _num_or_none(v: object) -> float | None:
    if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v):
        return float(v)
    return None


@mcp.tool(annotations=_READONLY)
def get_params(path: str, name_glob: str | None = None) -> ParamResult:
    """Parameter values recorded in the log's PARM dump. ``name_glob`` is a case-insensitive glob, e.g. 'INS_HNTCH_*'.

    Includes `total` (full param count), `metadata_url` (version-specific definitions to interpret values), and a
    `note` if the snapshot looks incomplete (truncated log) — in which case ask the user for their exported .param
    file and read it with `load_param_file`. `documented` gives each returned param's documented range, units,
    listed values and bitmask bits for the log's firmware (bundled, offline); a value outside the documented range
    or list is not necessarily wrong (some defaults, e.g. 0 = disabled, lie outside). `undocumented` names params
    that release does not document.
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
    result = ParamResult(
        path=path,
        params=dict(sorted(params.items())),
        matched=len(params),
        total=total,
        source="log",
        metadata_url=docs.versioned_param_doc_url(log.meta.firmware_version, log.meta.vehicle_kind),
        note=note,
    )
    return _document(result, log.meta.firmware_version, log.meta.vehicle_kind)


def _document(result: ParamResult, firmware_version: str | None, vehicle_kind: str | None) -> ParamResult:
    """Attach the bundled documented metadata of the returned params, when it fits."""
    catalog = catalog_for(firmware_version, vehicle_kind)
    if catalog is None:
        return result
    result.metadata_version = catalog.label
    result.undocumented = [k for k in result.params if catalog.doc(k) is None]
    if result.matched <= _DOC_LIMIT:
        result.documented = {k: d for k in result.params if (d := catalog.doc(k)) is not None}
    else:
        hint = f"documented metadata is attached for at most {_DOC_LIMIT} params; narrow with name_glob"
        result.note = f"{result.note}; {hint}" if result.note else hint
    return result


@mcp.tool(annotations=_READONLY)
def load_param_file(path: str, name_glob: str | None = None, firmware: str | None = None) -> ParamResult:
    """Load a user-exported ArduPilot parameter file (.param / .params).

    The authoritative *complete* configuration — use it when the log's PARM snapshot is truncated/incomplete,
    or to compare the flown config (get_params) against the user's current/intended config. ``name_glob`` filters.
    ``firmware`` (e.g. 'ArduCopter V4.6.3', from the log's log_summary) attaches the documented metadata and
    lists names that release does not document (typos, script or other-firmware params).
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
    result = ParamResult(
        path=path,
        params=dict(sorted(params.items())),
        matched=len(params),
        total=total,
        source="file",
        note=note,
    )
    if not firmware:
        return result
    return _document(result, firmware, meta.vehicle_kind(firmware, all_params.get("FRAME_CLASS")))


@mcp.tool(annotations=_READONLY)
def recommend_tuning(path: str, area: str | None = None) -> TuningReport:
    """Advisory tuning recommendations (harmonic notch from the batch sampler, a fast IMU, the in-flight FFT or ESC RPM; PID; autotune). Recommendation ONLY — never applied."""
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
