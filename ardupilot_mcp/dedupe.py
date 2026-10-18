"""One event, one finding.

A single in-flight event reaches the log several ways. A radio failsafe is an ERR
FAILSAFE_RADIO record, a MODE change with reason "radio failsafe", and the loss of RC
input itself. The events and firmware checks echo the first two, and the RC check
measures the third. Reported separately, the same event fills the top of the report
three times and pushes the next, different problem down.

This pass merges echoes into the finding that measured the event:

* Each finding gets a *topic* (radio, battery, GCS link, GPS, EKF, EKF lane switch, vibration,
  thrust loss, crash). For the specific checks the topic follows from the check; for the events and
  firmware checks from the ERR subsystem, ModeReason or verdict the finding carries; for
  in-flight status lines (prearm check) from their text.
* An echo (events/firmware/status line) that starts within ``MERGE_SLACK_S`` of a specific
  finding of the same topic is absorbed into it: the specific finding keeps its evidence,
  takes the higher severity, starts at the echo if the echo came first, and lists the echo
  under ``related``. An INFO finding only takes in INFO echoes.
* Echoes with no specific finding to join collapse into the first echo of their topic
  when they fall within ``MERGE_SLACK_S`` of it (one failsafe, logged as ERR + MODE).

Nothing is dropped: every absorbed finding is named in ``related`` with its time.
"""

from __future__ import annotations

from .model import Finding, Severity

# Design choice: an echo belongs to an event when it starts within this many seconds of
# the event's evidence window. Firmware failsafes act within a second or two of the
# condition they report; 5 s covers that with margin without joining separate events.
MERGE_SLACK_S = 5.0
# The battery failsafe fires only after the voltage has stayed below its threshold for
# BATT_LOW_TIMER seconds (default 10, AP_BattMonitor_Params.cpp), so its ERR and mode change
# trail the measured threshold crossing by that much.
BATT_LOW_TIMER_DEFAULT_S = 10.0

# Checks that measure a topic directly (they own the merged finding).
_SPECIFIC: dict[str, str] = {
    "thrust_loss": "thrust_loss",
    "rcin": "radio",
    "gps": "gps",
    "vibration": "vibration",
    "ekf": "ekf",
}
# power reports battery and board-voltage findings; only the battery ones share a topic
# with the battery failsafe.
_POWER_BATTERY = "battery"
# The ekf check's own lane-switch finding (from XKF4.PI) and the events check's
# ERR EKF_PRIMARY records describe the same switches.
_LANE = "ekf_lane"
_EKF_LANE_TITLE = "EKF primary core changed (lane switch)"

# ERR subsystem -> topic (ardupilot_meta.ERR_SUBSYSTEMS, AP_Logger.h LogErrorSubsystem).
_ERR_TOPICS: dict[int, str] = {
    2: "radio",  # RADIO
    5: "radio",  # FAILSAFE_RADIO
    6: "battery",  # FAILSAFE_BATT
    7: "gps",  # FAILSAFE_GPS
    8: "gcs",  # FAILSAFE_GCS
    11: "gps",  # GPS
    12: "crash",  # CRASH_CHECK
    16: "ekf",  # EKFCHECK
    17: "ekf",  # FAILSAFE_EKFINAV
    24: _LANE,  # EKF_PRIMARY
    25: "thrust_loss",  # THRUST_LOSS_CHECK
    29: "vibration",  # FAILSAFE_VIBE
}
# ModeReason -> topic (checks/firmware.FAILSAFE_REASONS, AP_Vehicle/ModeReason.h).
_MODE_REASON_TOPICS: dict[int, str] = {
    3: "radio",
    4: "battery",
    5: "gcs",
    6: "ekf",
    7: "gps",
    19: "crash",
    50: "ekf",
}
# Firmware verdict title (checks/firmware.VERDICTS) -> topic. A test keeps these in step
# with VERDICTS, so a reworded title cannot silently turn merging off.
VERDICT_TOPICS: dict[str, str] = {
    "Firmware detected a crash and disarmed": "crash",
    "EKF failsafe changed the flight mode": "ekf",
    "Firmware reported EKF variance": "ekf",
    "Firmware switched on vibration compensation": "vibration",
    "Firmware reported a GPS glitch or compass error": "gps",
    "Firmware fell back to dead reckoning": "ekf",
    "Firmware triggered a radio failsafe": "radio",
    "Firmware triggered a battery failsafe": "battery",
    "Firmware triggered a GCS failsafe": "gcs",
}
# The prearm check also surfaces in-flight status lines; their text is free-form.
_STATUS_TEXT_TOPICS: tuple[tuple[str, str], ...] = (
    ("radio failsafe", "radio"),
    ("battery failsafe", "battery"),
    ("gcs failsafe", "gcs"),
    ("gps glitch", "gps"),
    ("ekf failsafe", "ekf"),
    ("ekf variance", "ekf"),
    ("vibration compensation", "vibration"),
)
_ECHO_CHECKS = ("events", "firmware", "prearm")


def topic(f: Finding) -> str | None:
    """The event topic a finding reports, or None if it is not merged with others."""
    if f.check_id == "ekf" and f.title == _EKF_LANE_TITLE:
        return _LANE
    if f.check_id in _SPECIFIC:
        return _SPECIFIC[f.check_id]
    if f.check_id == "power":
        return _POWER_BATTERY if f.title.lower().startswith("battery") else None
    s = f.evidence.samples
    if f.check_id == "events":
        sub = s.get("Subsys")
        return _ERR_TOPICS.get(int(sub)) if sub is not None else None
    if f.check_id == "firmware":
        rsn = s.get("mode_reason")
        if rsn is not None:
            return _MODE_REASON_TOPICS.get(int(rsn))
        return VERDICT_TOPICS.get(f.title)
    if f.check_id == "prearm":
        low = f.title.lower()
        return next((t for key, t in _STATUS_TEXT_TOPICS if key in low), None)
    return None


def _window(f: Finding) -> tuple[float, float] | None:
    t0 = f.evidence.time_start_s
    if t0 is None:
        return None
    t1 = f.evidence.time_end_s
    return t0, (t1 if t1 is not None and t1 >= t0 else t0)


def _near(echo: Finding, host: Finding, lag: float = 0.0) -> float | None:
    """Distance (s) from the echo's start to the host's window, or None if beyond the slack.

    ``lag`` is how long after the measured condition the firmware acts on it (an echo may
    trail the host's window by that much more).
    """
    ew, hw = _window(echo), _window(host)
    if ew is None or hw is None:
        return None
    t = ew[0]
    if hw[0] <= t <= hw[1]:
        return 0.0
    if t < hw[0]:
        d = hw[0] - t
        return d if d <= MERGE_SLACK_S else None
    d = t - hw[1]
    return d if d <= MERGE_SLACK_S + lag else None


def _absorb(host: Finding, echo: Finding) -> None:
    when = f" at t={echo.evidence.time_start_s:.1f}s" if echo.evidence.time_start_s is not None else ""
    host.related.append(f"{echo.check_id}: {echo.title}{when}")
    host.related.extend(echo.related)
    if echo.severity.rank > host.severity.rank:
        host.severity = echo.severity
    for m in echo.evidence.message_types:
        if m not in host.evidence.message_types:
            host.evidence.message_types.append(m)
    # An echo that came first (within the slack) is when the event began: the firmware
    # acted before the measured symptom crossed its threshold. The end is kept: an
    # aggregated echo (a mode change repeated through the flight) would stretch it to the
    # last repeat, and later echoes would chain onto the stretched window.
    t_echo, t_host = echo.evidence.time_start_s, host.evidence.time_start_s
    if t_echo is not None and t_host is not None and t_echo < t_host:
        host.evidence.time_start_s = t_echo


def merge_duplicates(findings: list[Finding], params: dict[str, float] | None = None) -> list[Finding]:
    """Findings with each event's echoes folded into the finding that measured it (order kept)."""
    params = params or {}
    # Either pack's failsafe may be the one that fired (BATT_ / BATT2_LOW_TIMER).
    timers = [params.get(k) for k in ("BATT_LOW_TIMER", "BATT2_LOW_TIMER")]
    lags = {"battery": max((float(v) for v in timers if v is not None), default=BATT_LOW_TIMER_DEFAULT_S)}
    hosts: dict[str, list[Finding]] = {}
    for f in findings:
        t = topic(f)
        if t is not None and f.check_id not in _ECHO_CHECKS and _window(f) is not None:
            hosts.setdefault(t, []).append(f)

    absorbed: set[int] = set()
    echo_hosts: dict[str, list[Finding]] = {}
    echoes = [f for f in findings if f.check_id in _ECHO_CHECKS and topic(f) is not None]
    echoes.sort(
        key=lambda f: f.evidence.time_start_s if f.evidence.time_start_s is not None else float("inf")
    )
    for e in echoes:
        t = topic(e)
        assert t is not None
        # An INFO finding (a lane switch, a minor clip count) measures no problem, so it only
        # takes in INFO echoes: a failsafe would inherit a title that understates it.
        candidates = [
            h for h in hosts.get(t, []) if h.severity != Severity.INFO or e.severity == Severity.INFO
        ]
        near = [(d, h) for h in candidates if (d := _near(e, h, lags.get(t, 0.0))) is not None]
        if near:
            # The most severe nearby measurement, then the closest one.
            _, host = min(near, key=lambda dh: (-dh[1].severity.rank, dh[0]))
            _absorb(host, e)
            absorbed.add(id(e))
            continue
        prior = [(d, h) for h in echo_hosts.get(t, []) if (d := _near(e, h)) is not None]
        if prior:
            _absorb(min(prior, key=lambda dh: dh[0])[1], e)
            absorbed.add(id(e))
            continue
        if _window(e) is not None:
            echo_hosts.setdefault(t, []).append(e)
    return [f for f in findings if id(f) not in absorbed]
