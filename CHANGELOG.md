# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- Arming refusals were dropped as boot chatter when they contained "calibrated" or
  " ready" ("PreArm: Compass calibrated requires reboot", "Arm: GPS not ready"), and an
  "Arm:" refusal was rated INFO. Both prefixes now always count, as WARN.
- Fields compared sample by sample (ATT demand vs actual, GPS HDop vs Status, MAG X/Y/Z,
  BAT Volt vs Curr, the RCIN primary channels) were read one by one and paired by index,
  so one missing or NaN value shifted every later pair. A record with a NaN demand turned
  perfect roll tracking into a "Poor roll tracking (25.8 deg RMS)" advice and a tracking
  spike. `FlightLog.series_many` keeps the fields of one record together.
- The battery thresholds counted samples taken before arming (a monitor reads 0 V before
  the pack is connected) and any momentary sag. A threshold now counts only while armed and
  when the voltage stays below it for `BATT_LOW_TIMER`, as the firmware's failsafe does, or
  until the armed data ends.
- "Sudden voltage drop" kept only the largest step; when that was a glitch that recovered,
  a smaller drop that persisted went unreported. The largest persistent step is reported,
  judged while armed.
- An EV, MODE or MSG record written before the first IMU sample got a negative time; the
  time origin now includes them.
- A motor held exactly at 1950 us did not count as saturated; the attitude check judged
  motor saturation over the whole flight instead of the divergence window; RC loss reported
  the lowest PWM of the whole log instead of the loss window.
- Compass offsets were judged for a compass with `COMPASS_USE{n} = 0`.
- An ERR FLIGHT_MODE record now names the mode the vehicle could not switch to; corrupt
  ERR, MODE and EV records no longer break `list_events`; `recommend_tuning` accepts an
  area with surrounding spaces.
- Real-crash benchmark: top-1 11, top-3 22 (unchanged), false criticals 101 -> 98.
- Checks that skip the pre-arm phase (GPS acquisition) read arming from EV records only; the
  armed window now comes from every arm/disarm record, ARM included.
- Pre-4.1 logs were judged on their primary sensor and EKF core only. Firmware before 4.1
  logged a second or third IMU, GPS, compass, barometer and battery, and EKF cores 1 and 2,
  under their own names (`IMU2`, `MAG2`, `BAR2`, `BAT2`, `CUR2`, `NKF6`-`NKF9`, `NK11`-`NK14`,
  ...). These are now folded into the base message with the modern instance field when their
  fields match, so every check judges them per instance as on 4.1+ logs. On 40 forum logs from
  3.2-3.6, 31 had such messages (IMU 29, compass 24, EKF cores 24, barometer 2, battery 1) and
  the findings changed on 20; the benchmark (all 4.x) is unchanged.
- The vehicle profile's cell count, the summary's barometric max altitude and the timing
  check's log-gap analysis read every battery / barometer / IMU as one interleaved series.
  They now read the first instance, so a second pack no longer sets the cell count and a
  primary IMU that stops logging is no longer hidden by a second one that keeps going.
- `metadata_url` was described as giving parameter defaults; `apm.pdef.xml` carries ranges,
  values and bitmasks but no defaults.
- The parser left pymavlink's memory map of the log open until a garbage collection, so on
  Windows a log the server had read could not be replaced or deleted. The reader is now closed
  once the log is decoded.
- Notch advice: the spectrum was taken over every IMU interleaved into one stream (4.x logs
  all IMUs as one message), which mixes sensors and multiplies the apparent sample rate by the
  IMU count. It also recommended a throttle notch with `INS_HNTCH_REF` 1.0, the value the
  parameter documentation gives for RPM/ESC tracking; for a throttle notch REF is the throttle
  FREQ was measured at. A throttle notch now gets the hover throttle as REF (or none, when it
  is not known), a fixed-wing-only plane a fixed notch, and ESC RPM telemetry the ESC notch.
  A peak must stand out from its local noise floor, so the rising low-frequency end of the
  spectrum no longer passes for motor noise.
- `power`: a battery-threshold finding now spans the run below the threshold that the failsafe
  timer counts (the first one lasting BATT_LOW_TIMER, AP_BattMonitor), up to the lowest voltage.
  It used to be a single point at the minimum, often after the failsafe had already acted; a
  momentary sag under load at takeoff does not start it.
- ERR EKF_PRIMARY was reported as an error. Its ECode is the new primary EKF core
  (ArduCopter/ekf_check.cpp), so ECode 1 or 2 is a lane switch and ECode 0 a switch to core 0,
  not a cleared error. It is now INFO "EKF primary core changed to N" (38 WARNs on the benchmark),
  and the incident timeline shows it as context, never as the onset.
- A status line that merely contained the word "prearm" (a script's own message) was reported
  as a pre-arm failure. Only a line that starts with the firmware's "PreArm:" counts now.
- **Multi-instance sensors were interleaved.** Firmware 4.x logs every IMU, GPS, compass,
  battery and EKF core under one message name (told apart by an instance field). Checks now
  judge each instance on its own samples, using the log's own FMTU `#` declarations. This
  removes false alarms such as a CRITICAL "GPS lost 3D fix" caused by a second receiver,
  a CRITICAL "battery critical voltage" caused by a 2S payload pack on the second monitor,
  and "unstable compass" caused by two healthy compasses with different field strengths.
- **Accelerometer clipping was never detected on 4.x logs**: modern `VIBE` has one `Clip`
  counter per IMU (the check only read the pre-4.0 `Clip0/1/2`). Counter resets are handled.
- `timing`: "long loops" warned whenever `PM.NLon > 0`, i.e. on almost every healthy log. It
  now judges the long-loop percentage per PM window (`NLon/NL`), as ArduPilot's LogAnalyzer
  did, and cites `PM.Ex`.
- Flight modes were decoded with the Copter table for every vehicle (a Plane in MANUAL was
  reported as STABILIZE). Plane, Rover and Sub have their own tables; Copter gains TURTLE.
- `max_altitude_m` mixed AMSL (`POS/GPS.Alt`) with relative altitude; it is now the maximum
  altitude above home (`POS.RelHomeAlt`, else `CTUN.Alt`/`BARO.Alt`).
- `rcin`: RC loss is judged only while armed (a receiver with no transmitter on the bench is
  no longer a CRITICAL), and uses `RCI2.Flags` (no valid input / RC failsafe) when logged —
  most receivers hold the last values on link loss, which the PWM heuristic cannot see.
- `motors`: motor outputs come from `SERVOn_FUNCTION` (or the frame's motor count), so a
  gimbal servo is no longer a "motor imbalance"; only armed time is averaged, so a real
  imbalance is no longer diluted by disarmed samples.
- `query_timeseries`: when a field was missing from some records, the time window was
  silently not applied and values no longer lined up with `times_s`. Values are now aligned
  by record (`null` where a record lacks the field) and unknown fields are reported.
- `sensors` recognises `GPS1_TYPE` (4.6 renamed `GPS_TYPE`); compass health is per compass.
- NaN/inf samples no longer poison max/mean comparisons (they are dropped from series).
- Arming detection on real logs: with `LOG_DISARMED=0` the log opens at arming and the arm record
  is often lost, so a log whose first edge is a disarm (or, with `LOG_DISARMED=0`, has none) is
  treated as armed from its start. Many forum logs were previously "never armed".
- `vibration`: fewer than 100 clips is INFO, not WARN (the wiki calls a small count around a hard
  landing likely OK); 100+ stays CRITICAL.
- `sensors`: "GPS configured but no data" is INFO when `GPS1_TYPE` is the default AUTO and no GPS
  mode was flown (indoor/optical-flow craft), WARN otherwise.
- `attitude`: judged in flight only (armed, and between Copter's NOT_LANDED and LAND_COMPLETE
  events when logged), and Plane yaw is skipped (ArduPlane logs `ATT.DesYaw` as a 0 placeholder).
  On 100 forum logs this was the most common CRITICAL: an upside-down crashed or carried
  airframe, and every Plane's heading, read as "attitude tracking failure". The PID tuning advice
  uses the same flight window. Compass field-strength stability uses it too.
- `timing`: a logging gap that starts while disarmed is expected with `LOG_DISARMED=0` (logging
  pauses between flights in the same file) and is no longer a "main-loop stall"; half of the gap
  warnings on the forum corpus were this.
- Logs without GPS parsed ~40x slower than the rest: pymavlink scans the whole log for a GPS clock
  we never use. The reader now skips that scan (15 MB forum log: 6.9 s -> 0.15 s).

### Added
- `ardupilot-mcp --version` prints the installed version without starting the server.
- `docs/reference.md` lists every tool with its arguments and every check with the vehicles,
  log types and messages it needs. `scripts/gen_docs.py` generates it from the code and
  keeps the version and license in `server.json` and the plugin manifests in step; a test
  fails when any of them is out of date.
- Repository automation: CodeQL and dependency review on every PR, Dependabot for pip and
  GitHub Actions, a PR check for the Conventional Commits title, the changelog entry and the
  CLA, labels from `.github/labels.json`, release notes grouped by label, and pre-commit
  hooks (ruff, the generated docs, whitespace).
- MAVLink telemetry logs (`.tlog`): every tool accepts the log a ground station records,
  mapped onto the DataFlash messages the checks read (ATT with the demand interpolated at
  each attitude sample, VIBE, XKF4, BAT, GPS, IMU, MAG, RCIN, RCOU, CTUN, POS, MODE, ARM,
  MSG and the parameters), reading only the vehicle's autopilot. Checks that would mistake
  a radio dropout for a vehicle fault (log integrity, timing gaps, configured-but-silent
  sensors) do not run on it, and a telemetry log that stops while armed is not taken as the
  impact. On seven SITL flights recorded both ways the WARN/CRITICAL findings match the
  onboard log's, except a fast voltage drop the 2 Hz battery stream cannot show; fifteen
  forum tlogs (3.1-4.5; Copter, Plane and Rover) parse in under 4 s each.
- Firmware verdicts for the failsafes Copter announces ("Radio Failsafe - ...", "Battery
  Failsafe", "GCS Failsafe"), the only record of them in a telemetry log; in an onboard log
  they merge into the RC and battery findings.
- Documented parameter metadata, offline: `get_params` (and `load_param_file` given a
  `firmware`) returns each parameter's documented range, units, listed values and bitmask
  bits for the log's firmware release, and lists the names that release does not document
  (script parameters, typos, another firmware's parameters). Facts from ArduPilot's
  `apm.pdef.xml` for Copter, Plane and Rover 4.4-4.7 and Sub 4.1/4.5/4.7 are bundled (330 KB),
  regenerated with `scripts/gen_param_meta.py`; a name documented twice (Copter's multirotor
  and heli `ATC_RAT_*`, two battery backends) keeps the widest range and every listed value
  and bit of both. It informs and never raises a finding: on
  30 Copter 4.6 forum logs a range/values check would have flagged 895 values, mostly
  defaults outside the recommended range and raw BAUD rates.
- Notch advice from the data that can show motor noise: the batch sampler (`ISBH`/`ISBD`,
  raw gyro at the sensor rate, decoded into `FlightLog.batches`), else the primary IMU when it
  is logged fast enough, else the in-flight FFT (`FTN1.PkAvg`). With pre- and post-filter
  batches the energy left after the filters is reported, and the in-flight FFT is quoted when
  it agrees or disagrees. With ESC RPM telemetry the advice is the ESC notch, FREQ set below
  the hover motor frequency and above `INS_GYRO_FILTER` as the wiki asks, and the gyro line is
  matched to the motors' harmonics: a line on no harmonic is called out as not motor noise,
  and a harmonic outside `INS_HNTCH_HMNCS` is added to it. A notch that already follows an
  RPM sensor or the in-flight FFT is kept, with a warning when its FREQ floor sits above the
  measured line. On 100 forum logs, 53 of the 61
  with such data get advice; where ESC RPM and gyro data meet (20 logs) the line is the
  fundamental on 11, a harmonic on 6 and no motor harmonic on 3.
- `forum_report(path)` tool: a Markdown reply for a discuss.ardupilot.org thread — vehicle and
  firmware, what went wrong first, up to six findings with their corroborating records, and a
  text sparkline of the data behind each (vibration axis, demanded vs actual attitude, pack
  voltage and current, satellites and HDOP, the EKF test ratio, the lost motor's output and
  throttle). It never reads a position (no GPS latitude/longitude), keeps only the file name,
  and returns text without writing anything.
- Real-crash benchmark (`bench/`): 49 forum flight logs whose cause a developer or experienced
  helper stated after analysing the log, stored as source URL + sha256 + label (logs are not
  redistributed), and an offline scorer (`python -m bench.run --logs DIR`) for top-1/top-3 cause
  hits and misleading CRITICAL findings.
- `incident_timeline` tool and `DiagnosisReport.incident`: the onset (first anomaly while armed)
  and the impact (ERR CRASH_CHECK, "Crash: Disarming", or the log ending while armed). Findings
  that start at or after the impact are flagged `after_impact` and listed last, so a crash's
  consequences (vibration spikes, attitude error, GPS loss) no longer top the report. Flight-long
  conditions (compass statistics, parameters, boot messages, windows covering most of the flight)
  are listed as context and never chosen as the onset. Time order only; no causal claims.
- `compare_logs(path_a, path_b)` tool: what changed between two flights — configuration
  parameter differences (kept apart from STAT_* counters, learned hover throttle, gyro offsets,
  calibration temperatures and other values the autopilot writes itself), parameters only in one
  log, the firmware change, and vibration / hover / voltage metrics side by side.
- Logging advice (`DiagnosisReport.logging_advice`, Copter/Heli): for data a check needed but the
  log lacks, the `LOG_BITMASK` bit that logs it — only when that bit is actually off — and
  `IMU_FAST`/`INS_LOG_BAT_MASK` when the IMU rate is too low for a notch spectrum.
- `thrust_loss` check (Copter, incl. Heli): the firmware's thrust-loss detection from any of its three traces
  ("Potential Thrust Loss (n)", ERR THRUST_LOSS_CHECK, `MOTB.FailFlags` thrust boost) is CRITICAL
  and names the motor and the output it is wired to. Without a detection, the same pattern in the
  motor outputs is WARN: one motor (or coaxial pair) at maximum, the motor opposite it in the
  frame's geometry the lowest of the rest, sinking at high throttle, near level and under control,
  for at least 1 s. Frame geometry for every quad/hexa/octa/octaquad/Y6/deca/dodecahexa type is
  generated from `AP_MotorsMatrix.cpp` (`scripts/gen_motor_geometry.py`); the output pattern is
  not judged on Y6, V-tail and A-tail quads, where no motor sits directly opposite another. On ArduCopter 4.6.3 SITL
  with a weakened motor the pattern names the same motor 0.9 s before the firmware does. The
  finding starts when the loss began (the firmware fires after 1 s of the pattern), so it ranks
  above the attitude error the loss causes. Real-crash benchmark: top-1 9 -> 10.
- `firmware` check: the autopilot's own in-flight verdicts — "Crash: Disarming", "EKF Failsafe", "Yaw Imbalance", "Vibration compensation ON", "EKF
  variance", GPS glitch, dead reckoning — and failsafe-driven mode changes decoded from
  `MODE.Rsn` (radio, battery, GCS, EKF, fence, crash, ...).
- `power`: flight-controller board voltage (`POWR.Vcc`) — CRITICAL below 4.6 V while armed,
  WARN on more than 0.3 V of ripple. Per-battery thresholds (`BATT2_*` for the second pack).
- `compass`: throttle-correlated interference estimate (% field change at full throttle, the
  quantity CompassMot shows), graded with ArduPilot's 30 % / 60 % bands.
- `ekf`: per-core evaluation; full severity only while the core was primary (`XKF4.PI`),
  standby-core strain is INFO, and an EKF lane switch is reported.
- `integrity`: "Recording ended while armed" — the log stops with no disarm, the signature of
  an in-flight power loss or crash.
- `query_timeseries(..., instance=)` and `available_instances` for multi-instance messages.
- `FlightLog.instances()`, `series(..., instance=)`, `armed_intervals()`, `armed_mask()`.
- `tests/test_scenarios.py`: 23 labelled fault-injection flights written in Copter-4.6.3's
  exact log format (dual GPS, dual compass, three IMUs, two EKF cores) run through the full
  pipeline; a clean flight that raises a single warning fails CI.

### Changed
- **License: AGPL-3.0-only** from this change on, with a commercial license available
  (contact@furkanisikay.com). The packages published on PyPI up to 0.1.3 remain under the
  MIT license.
  Contributions need the [Contributor License Agreement](.github/CLA.md).
- README (Turkish and English) cut to what a new user needs; client setup and
  troubleshooting moved to `docs/clients.md`. Community files (contributing, security,
  code of conduct, support) live in `.github/`.
- **One event, one finding.** A radio failsafe used to appear as ERR FAILSAFE_RADIO, ERR RADIO,
  a "radio failsafe" mode change and the RC-loss finding, each on its own line. ERR, failsafe
  mode-change and firmware echoes now join the finding that measured the event (RC, battery,
  GPS, EKF, vibration, thrust loss) when they start within 5 s of it. The merged finding keeps
  the higher severity and lists the echoes in `Finding.related`. Echoes with nothing to join
  (a crash recorded three ways) collapse into the first. On the real-crash benchmark: WARN
  findings 262 -> 178, CRITICAL 199 -> 169, false criticals 113 -> 101, top-1 10 -> 11,
  top-3 21 -> 22.
- `compass`: field-strength peak-to-peak spread no longer raises a warning on its own (it is
  kept as evidence). One glitched sample sets it; on 100 forum logs it fired on 36 % of compass
  instances, none of them a compass fault among the labelled flights. The coefficient of
  variation still grades the field. Real-crash benchmark: top-1 8 -> 9, top-3 20 -> 21.
- **Parsing is ~20x faster and uses ~4x less memory.** Messages are decoded into numpy
  columns straight from pymavlink's own index instead of one Python object per record (a
  20-minute log with 400 Hz IMU: 26 s / 1.6 GB -> 1.2 s / 0.4 GB). The decoder is checked
  field-for-field against pymavlink and falls back to it for unusual layouts. Raw sensor
  dumps no check reads (`ISBH/ISBD/ACC/GYR`) are not loaded.
- `analyze_log` lists each finding once (in `findings`); `results[]` now carries
  `finding_count` instead of a second copy, roughly halving the payload sent to the LLM.
- The server keeps at most two parsed logs in memory (LRU) instead of every log ever opened.
- A half-written final record (what unplugging the battery after landing leaves) is dropped
  without flagging the log as truncated.

## [0.1.3] — 2026-10-17

### Fixed
- Pinned `mcp` to `>=1.27,<2`. mcp 2.0 (July 2026) removed `mcp.server.fastmcp`, so fresh
  installs (`uvx ardupilot-mcp`, `pip install ardupilot-mcp`) crashed on startup with
  `ModuleNotFoundError`.

### Added
- `.oss-scanner/Dockerfile` and `.oss-scanner/threat_model.md` for Anthropic's OSS Scanner.
- A security contact email in `SECURITY.md`.

## [0.1.2] — 2026-06-14

### Added
- One-click "Add to X" install buttons (Cursor, VS Code, VS Code Insiders, LM Studio) and
  per-client setup snippets for Claude Code, Claude Desktop, Cursor, VS Code, Antigravity
  (IDE + CLI), Windsurf, Cline, Continue, Zed, Goose and LM Studio in both READMEs, plus an
  example-scenarios section. All launch via `uvx ardupilot-mcp`.
- `server.json` — Model Context Protocol registry manifest (PyPI package, stdio transport).
- Claude Code plugin: `.claude-plugin/plugin.json`, `.claude-plugin/marketplace.json`, a
  root `.mcp.json`, and a `/diagnose` command that drives a full log diagnosis with the tools.
- Beginner onboarding in both READMEs: a plain-language opener (what it is, and that it
  plugs into an AI client), a 5-minute Quick start with one client, a "where to get a `.bin`
  log" section, a "try it with no AI app" path (clone + `examples/`), a sample diagnosis
  output, and a Troubleshooting section. `uv`/`uvx` is now explained and linked, and the
  pip-vs-uvx config choice is made explicit.
- A "why this" section comparing the tool, honestly, to Mission Planner Auto Analysis and
  the UAV Log Viewer — what each does, and when to use the others.

### Changed
- README reordered so install/usage comes before architecture/philosophy. Documentation
  only — no code or check behaviour changes.

## [0.1.1] — 2026-06-14

### Changed
- README is now Turkish (primary, `README.md`) with a plain link to the English
  version (`README.en.md`); removed audience/positioning meta-text from the README
  and package/repo descriptions. Documentation only — no code changes.

## [0.1.0] — 2026-06-14

First release: an offline, read-only MCP server that diagnoses ArduPilot DataFlash
`.bin` flight logs with a deterministic check engine; the LLM explains the grounded
findings.

### Added
- **Parser** (`pymavlink` DFReader) → pure `FlightLog` domain model; defensive against
  truncated/corrupt logs; flight-relative timestamps robust to corrupt outliers.
- **16 diagnostic checks** across two families:
  - Flight dynamics: events/errors, EKF health, vibration, power (BAT/CURR), GPS,
    compass, attitude tracking, motor balance, RC-link loss, timing.
  - Configuration & setup: risky parameters, parameter audit (contradictory/disabled/
    implausible values), calibration, configured-but-silent sensors, firmware pre-arm
    messages.
- **Vehicle awareness** (copter/heli/plane/rover) — checks gate to the vehicles they
  apply to.
- **Advisory tuning** — harmonic notch (gyro FFT), PID, autotune.
- **Documentation grounding** — every finding links authoritative ArduPilot docs;
  reports surface version-specific parameter definitions (`apm.pdef.xml`) for 4.x.
- **Physical profile** (`vehicle_profile`) — frame, motor count, battery cells, hover
  throttle → power margin and thrust-to-weight.
- **9 read-only MCP tools**: `analyze_log`, `log_summary`, `vehicle_profile`,
  `list_events`, `query_timeseries`, `get_params`, `load_param_file`, `recommend_tuning`,
  `list_checks`.
- **Parameter completeness** signals + `.param`/`.params` file ingestion.
- `examples/analyze.py` CLI and an MCP stdio smoke test.
- [`docs/SOURCES.md`](docs/SOURCES.md): every threshold/enum/fact audited against
  authoritative sources.

[Unreleased]: https://github.com/furkanisikay/ardupilot-mcp/commits/main
[0.1.3]: https://pypi.org/project/ardupilot-mcp/0.1.3/
[0.1.2]: https://pypi.org/project/ardupilot-mcp/0.1.2/
[0.1.1]: https://pypi.org/project/ardupilot-mcp/0.1.1/
[0.1.0]: https://pypi.org/project/ardupilot-mcp/0.1.0/
