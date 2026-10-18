"""One event, one finding.

A single in-flight event reaches the log several ways. A radio failsafe is an ERR
FAILSAFE_RADIO record, a MODE change with reason "radio failsafe", and the loss of RC
input itself. The events and firmware checks echo the first two, and the RC check
measures the third. Reported separately, the same event fills the top of the report
three times and pushes the next, different problem down.

This pass merges echoes into the finding that measured the event:

* Each finding gets a *topic* (radio, battery, GPS, EKF, vibration, thrust loss, crash).
  For the specific checks the topic follows from the check; for the events and firmware
  checks it follows from the ERR subsystem, verdict or ModeReason named in the title.
* An echo (events/firmware) that starts within ``MERGE_SLACK_S`` of a WARN/CRITICAL specific finding of
  the same topic is absorbed into it: the specific finding keeps its evidence, takes the
  higher severity, and lists the echo under ``related``.
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

# Echo titles (events: "Error: <ERR subsystem> (ECode n)"; firmware: verdict titles and
# "Mode changed by <ModeReason>") -> topic. ERR subsystem names: ardupilot_meta.ERR_SUBSYSTEMS;
# ModeReason labels: checks/firmware.FAILSAFE_REASONS.
_ECHO_TOPICS: tuple[tuple[str, str], ...] = (
    ("THRUST_LOSS_CHECK", "thrust_loss"),
    ("FAILSAFE_RADIO", "radio"),
    ("Error: RADIO", "radio"),
    ("radio failsafe", "radio"),
    ("FAILSAFE_BATT", "battery"),
    ("battery failsafe", "battery"),
    ("FAILSAFE_GPS", "gps"),
    ("Error: GPS", "gps"),
    ("GPS glitch", "gps"),
    ("FAILSAFE_VIBE", "vibration"),
    ("vibration compensation", "vibration"),
    ("EKFCHECK", "ekf"),
    ("FAILSAFE_EKFINAV", "ekf"),
    ("EKF variance", "ekf"),
    ("EKF failsafe", "ekf"),
    ("CRASH_CHECK", "crash"),
    ("crash and disarmed", "crash"),
    ("crash failsafe", "crash"),
)
_ECHO_CHECKS = ("events", "firmware")


def topic(f: Finding) -> str | None:
    """The event topic a finding reports, or None if it is not merged with others."""
    if f.check_id in _SPECIFIC:
        return _SPECIFIC[f.check_id]
    if f.check_id == "power":
        return _POWER_BATTERY if f.title.lower().startswith("battery") else None
    if f.check_id in _ECHO_CHECKS:
        low = f.title.lower()
        return next((t for key, t in _ECHO_TOPICS if key.lower() in low), None)
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
    hw, ew = _window(host), _window(echo)
    if hw is not None and ew is not None and ew[1] > hw[1]:
        host.evidence.time_end_s = ew[1]


def merge_duplicates(findings: list[Finding], params: dict[str, float] | None = None) -> list[Finding]:
    """Findings with each event's echoes folded into the finding that measured it (order kept)."""
    params = params or {}
    lags = {"battery": float(params.get("BATT_LOW_TIMER", BATT_LOW_TIMER_DEFAULT_S))}
    hosts: dict[str, list[Finding]] = {}
    for f in findings:
        t = topic(f)
        # An INFO finding (a lane switch, a minor clip count) measures no problem, so it never
        # takes in a failsafe: the echo would inherit a title that understates it.
        if (
            t is not None
            and f.check_id not in _ECHO_CHECKS
            and f.severity != Severity.INFO
            and _window(f) is not None
        ):
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
        near = [(d, h) for h in hosts.get(t, []) if (d := _near(e, h, lags.get(t, 0.0))) is not None]
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
