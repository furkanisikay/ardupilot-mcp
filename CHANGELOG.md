# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `landing` (Plane): **a flare that did not slow the descent**. For each automatic-landing flare
  the firmware announces ("Flare 2.0m sink=1.92 ..."), the check follows TECS.dh to the
  touchdown and reports a flare that reached the ground still sinking at half its starting rate
  or more, with the flare's height, its time to the ground, LAND_FLARE_ALT / LAND_FLARE_SEC,
  the TECS_LAND_SINK target and the touchdown jolt. The touchdown of the log's last such flare
  is the incident's impact, so what happens on the ground afterwards is listed after it. Forum
  log 142129 (flare started too low, fixed by raising LAND_FLARE_ALT) now names its cause: 2.07
  and 1.36 m/s at touchdown against a 0.25 m/s target. Validated on 26 flares in 23 Plane logs,
  21 of them downloaded for this: the 12 soft touchdowns stay clean, all 8 reported flares hit
  the ground at 26-81 m/s^2, and four hard touchdowns that kept less of the sink are missed. The
  benchmark counts it towards the configuration category, as it names the flare parameters.
- `attitude` (Plane): a loss of control that began above AIRSPEED_MAX, on a flight that spent
  much of its time there, now says the speed it began at and how much of their tuned deflection
  the surfaces had (SCALING_SPEED / airspeed, Plane::calc_speed_scaler). The report's first
  finding then points at the airspeed parameters instead of leaving them four findings down:
  forum logs 137589 (a wing at 50 m/s on AIRSPEED_MAX 28, surfaces at 0.30x) and 142491
  (TAKEOFF at 27 m/s on AIRSPEED_MAX 22, 0.56x), where the thread traced the crash to
  SCALING_SPEED and the airspeed range. It states the measurement only: forum log 136852, whose
  thread blamed roll tuning, shows the same.
- `esc` (QuadPlane): **VTOL motors lost power while still commanded**. A QuadPlane's VTOL motors
  run the multicopter motor library under Q_ parameters (Q_M_SPIN_MIN, Q_M_PWM_MIN/MAX) on the
  outputs whose SERVOn_FUNCTION is a motor; the copter power-cut test now runs on them, and ESC
  temperatures are read on planes too. Forum log 142142 (a 4-in-1 ESC overheating in an enclosed
  fuselage: 46 A to 0 A with the motors at 1800+ us) now opens with it; no other of the 38
  plane logs triggers it.
- `config`: **an arm/disarm switch on a stick channel** (RCx_OPTION ArmDisarm, ArmDisarm with
  AirMode or Arm/Emergency Stop on the roll, pitch, throttle or yaw channel of RCMAP_*): a stick
  movement then disarms the vehicle in flight. An in-flight disarm by RC switch with this set is
  a WARN that names the stick, not a commanded INFO. Found by forum thread 144992 (RC4_OPTION 153
  on the yaw stick of a QuadPlane); no other of the 145 validation logs has it.
- `firmware` (QuadPlane, Plane 4.7+): **VTOL thrust loss** ("Potential VTOL Thrust Loss (n)",
  CRITICAL, with the suspect motor). The copter `thrust_loss` check does not run on a plane, so
  this message went unreported beyond its ERR record. The QuadPlane transition-failure and VTOL
  assist messages are now confirmed on real forum logs (140907, 142096, 143642, 143882).
- `gps`: **poor reported accuracy in flight**. EKF3 starts using a GPS only once its horizontal
  accuracy is within 5 m and holds it to 1.3 times that once in use (EK3_CHECK_SCALE scales
  both); GPA.HAcc outside that for 10 s at a stretch in flight is a WARN. On the benchmark it
  fires only on 141345_0, the GPS glitch that corrupted the height before the crash.
- Vibration compensation (FAILSAFE_VIBE, "Vibration compensation ON") now says what the measured
  vibration was in the 3 s before. The firmware triggers it from the EKF's vertical innovations,
  not from the vibration level, and on the benchmark and forum logs it split cleanly: peaks of
  57 m/s^2 and more with clipping at crashes and tumbles, at most 15 m/s^2 with no clipping
  where the helpers found a height-estimate cause (a GPS glitch, poor GPS accuracy, prop wash on
  the barometer, an oscillating altitude controller). In the second case the finding says
  vibration did not trigger it and points at those causes.
- `gps`: **a receiver too slow for the autopilot**. AP_GPS counts a frame more than 245 ms after
  the last as delayed and holds the receiver unhealthy after two in a row or with an average
  of 215 ms; the same rule is applied to GPA.Delta, and a receiver unhealthy by it for 10% of the
  armed time is a WARN. It fires on one benchmark log: two NMEA receivers at 1 Hz (138886_0),
  which ArduPilot does not configure itself.
- `power` (multirotor): **the battery monitor is not reading the flight pack**. A configured
  voltage-and-current monitor whose current never reaches 1 A over 10 s of flight, or whose
  voltage stays under 4 V, is wired or set up wrong, and the capacity failsafe, current-based
  compass compensation and the voltage failsafe work from a wrong number (WARN). It fires on
  two benchmark logs with unconfigured sensors (141734_0, 144203_1) and one forum log reading
  0.11 V in flight.
- `health` check: **the flight controller was reset by its watchdog**. The firmware writes
  WDOG only on the boot after a watchdog reset; the finding names the fault type (a hard fault
  is CRITICAL, as the wiki calls it extremely serious), the fault line and task, and whether
  the vehicle was armed or its safety switch off at the reset. A telemetry log's `WDG:` text
  is read too. The same check reports a main loop stalled while armed (MON, WARN at the
  firmware's 500 ms main_loop_stuck line) and internal errors decoded by name from PM.InE or
  the "Internal errors 0x.." message, which the startup-message check no longer repeats.
  Validated on four forum watchdog logs; no benchmark log changes.
- Logging advice for Plane and Rover. Which LOG_BITMASK bit to turn on for a missing message
  was given for Copter only, as the bit numbers differ; Plane and Rover now get their own
  (Plane RC bit 13 and COMPASS 10, Rover RC 14 and COMPASS 11, from each vehicle's defines.h).
- `firmware` (QuadPlane): a forward transition that timed out ("Transition failed, exceeded time
  limit", CRITICAL, with the QLAND/QRTL it triggers counted as a failsafe mode change) and the
  VTOL motors assisting fixed-wing flight for low altitude or a large attitude error (WARN).
- `baro` check (multirotor): **the barometer follows the throttle**. Against the GPS altitude,
  with the slow drift between them removed, a baro reading that rises and falls with CTUN.ThO
  is prop wash or duct airflow on the barometer, which makes altitude hold surge. It fires on
  the two benchmark flights of the cinewhoop whose thread found exactly this (145480) and on no
  other log.
- `circling` check: **toilet bowling**, which the README listed as not detected. In LOITER,
  POSHOLD or QLOITER, the position error (PSCN/PSCE) turning the same way around a still target
  (a target the pilot moves faster than 2 m/s is not judged), orbit after orbit; the wiki ties it
  to a compass problem. Validated on a forum QuadPlane that
  toilet-bowled once a fan was fitted (3.8 turns) against its log without the fan (0.3) and the
  48 benchmark logs (at most 0.8).
- `esc` check (multirotor). **Motors lost power while still commanded**: the battery current
  collapses in flight while the motor outputs still ask for thrust, the pack voltage holds and
  the log keeps running, so only the ESCs stopped drawing power (an ESC protection shutdown or
  a failed ESC power feed). This is the signature the forum's helpers diagnose by hand; on
  the 48 benchmark logs it fires only on the labelled ESC-overheating crash (132194_0).
  **Hot ESCs**: ESC telemetry temperatures held at or above 80 degC, the bottom of BLHeli_S's
  programmable thermal-protection range (WARN above its 140 degC top); a reading already that
  high on a stopped motor at power-on is named and not judged.
- `param_audit`: MOT_BAT_VOLT_MAX / _MIN that the pack never enters. The firmware clamps the
  resting voltage into that range before scaling thrust, so a pack above MAX all flight (a 12S
  pack with the 6S default left in) gets no voltage compensation, and one below MIN gets its
  gains raised all flight. Found on one benchmark log (140954_0).
- `param_audit` (Plane): **flew far above AIRSPEED_MAX** (CTUN.As above it for 20% of the flight
  or more), with the surface speed scaling that results (SCALING_SPEED / airspeed, as
  `calc_speed_scaler` applies it), and **AIRSPEED_MAX not above AIRSPEED_MIN**. On the
  benchmark it fires on the six plane logs whose threads found the airspeed parameters wrong
  for the airframe and on one with MAX 8 below MIN 9, and on no other plane.
- `param_audit`: an EKF3 primary height source of GPS or rangefinder (EK3_SRC1_POSZ 3 / 2) is
  noted with what the EKF sources page says of it; a GPS glitch corrupts a GPS height directly.
- `events`: a disarm while still flying, with who did it (ARM.Method decoded: switch, rudder,
  GCS, a failsafe, ...). An autopilot-initiated one (a failsafe or termination) is a WARN; a
  commanded one is INFO, since it is most often the pilot's emergency stop after control was
  lost. A disarm right after a LAND_COMPLETE_MAYBE is a landing, not flight.

### Fixed
- `attitude` (Copter, heli): judged the way the firmware judges loss of control. Roll and pitch
  were two separate Euler errors critical at 25 deg; they are now one thrust-vector ("lean")
  error, named after the axis that carried most of it and critical at Copter's own line of
  30 deg off the target for 1 s (crash_check.cpp). A heading divergence is at most a warning
  ("Heading not held"): the motor mixer gives yaw only the room roll and pitch leave, and the
  crash check does not count heading. The heading error is now the rotation left once the thrust
  directions are aligned, as AC_AttitudeControl splits it, and is not judged while the lean
  error is over 30 deg, where the controller stops holding heading. Near 90 deg of pitch the
  Euler roll and yaw differences were reported as two critical tracking failures (forum log
  145303_4, 28 deg each with the thrust direction on target); they are gone. Plane is unchanged.
  On the 145 validation logs, attitude criticals fell from 107 to 80, none new.
- An ERR record written while the vehicle was disarmed (the transmitter switched off after
  landing, an EKF stopping its aiding at the disarm) was a CRITICAL fault at the top of the
  report; with arming recorded it is now an INFO note saying the vehicle was disarmed. A failsafe
  in flight is unchanged.
- When an event reaches the log as both an ERR record and a firmware message, the firmware's
  message now carries the merged finding ("Firmware detected a crash and disarmed" instead of
  "Error: CRASH_CHECK (ECode 1)"): it names the suspect motor or the failsafe action, and the ERR
  record stays listed under `related`. Benchmark top-3 22 -> 23 (144128_0).
- A compass rejected by the EKF (XKF4.SM over 1 for a second) was CRITICAL on its own. The
  firmware's EKF check (Copter and Plane) counts the compass only together with velocity or
  position and never acts on it alone, so with SV and SP under FS_EKF_THRESH it is now a WARN
  that says navigation carried on from GPS. Benchmark: top-1 10 -> 11, false criticals 48 -> 45.
- GPS and RC input are judged in flight, not over all armed time. A plane with ARMING_REQUIRE=0
  is armed from boot, so its GPS acquiring satellites and its receiver binding on the ground
  were CRITICAL (forum log 145430_0); an RC failsafe tested on the ground was a CRITICAL link
  loss (141676_0). Plane uses STAT.isFlying, Copter its land-detector events; the firmware's
  failsafe mode change is still reported. Benchmark false criticals 49 -> 48.
- "GPS / rangefinder configured but no data logged" when the data was simply not logged:
  LOG_BITMASK without the GPS bit (2), or on a copter without the CTUN bit (4) that carries RFND,
  says nothing about the sensor. The logging advice names the bit instead. Three benchmark and
  three forum findings go. One benchmark top-1/top-3 hit went with them (142202_0, whose
  labelled cause is a GNSS update rate, scored through the unrelated rangefinder line).
- A log with a corrupt stretch flooded stderr: pymavlink's compiled indexer prints a "bad
  header" line per bad byte below Python (146 MB for one 30 MB forum log), and MCP clients
  write the server's stderr to a log file on every call. File descriptor 2 is now silenced
  while the indexer runs; integrity is still judged from the record index.
- RC dropouts the receiver did not report. With RCI2 in the log, only its "no valid input"
  flag was read, so a receiver that sends its own failsafe values on a dropout (every channel
  at 875 us while still flagged valid, a FrSky Fport link in forum thread 145881) went unseen
  while the throttle and mode channel fell with it. The PWM signature is now checked too, and
  it is CRITICAL when FS_THR_ENABLE is 0, since nothing then catches the low throttle; mode
  changes during a drop are named.
- A board-voltage dip as a vehicle hits the ground no longer reads as an in-flight brown-out:
  CRITICAL also needs the rail under 4.6 V for 10% of the armed time (144427_0: 56%, 145881_0: 1%).
- Board voltage (POWR.Vcc) below 4.6 V was always a CRITICAL "brown-out", including a 0.1 V dip
  on a board running at 4.64 V that kept logging. 4.6 V is LogAnalyzer's FAIL level, which this
  project reports as WARN elsewhere; it is CRITICAL now only when the recording then stops while
  armed, the brown-out itself (benchmark log 144427_0).
- Copter 3.x logs: EV, ERR, MODE, MSG and PM carry no timestamp there, so their events had
  no time and a crash check never reached the incident timeline. Each such record now takes
  the time of the last boot-clock record before it in the file. A logging gap that holds a
  re-arm is a disarmed pause (3.x logged only while armed), not a stalled loop, and a log
  re-armed with no disarm record at all no longer claims it "ended while armed".
- Copter 3.x GPS records were timed by GPS time of week (GPS.TimeMS), hours away from the rest
  of the log, so no GPS sample fell inside the flight; they now use their boot clock (GPS.T).
- Bench logs judged as flights. An onboard log recorded from boot (LOG_DISARMED 1 or 3) with
  no arm record was treated as "arming unknown", so a USB-powered bench session raised a
  critical battery voltage and attitude failures. The firmware records every arm in such a
  log, so it is now read as never armed (4.x firmware); this removes 10 false CRITICALs from
  six forum logs and changes no benchmark log.
- A bare "PreArm:" line no longer becomes its own startup warning.
- Vibration false alarms. A single VIBE sample above 60 m/s^2 made the axis CRITICAL, and
  samples before arming and after landing counted. Levels are now judged in flight only, and
  CRITICAL needs 2 s above 60 m/s^2 or 10% of the flight (the wiki's levels are sustained
  levels); a shorter run is reported as a spike (INFO). 100 or more clips are a WARN, and
  CRITICAL only when the count keeps rising (3+ episodes, or one lasting 8 s or more): one
  burst of clipping is the tumble or impact, not its cause. On the 48 benchmark crash logs
  no run above 60 m/s^2 before the impact lasted more than 0.6 s, and every 100+ clip count
  came from bursts of at most 4.6 s.
- ERR FAILSAFE_VIBE was CRITICAL. The firmware only switches the altitude controller to
  vibration-resistant gains (no mode change), and it fires on an EKF pattern that a fall
  produces too (ekf_check.cpp); it is now a WARN, as the "Vibration compensation ON" text
  already was.
- EKF test ratios were judged on every sample, on the ground too, and a single sample above
  1.0 was CRITICAL. They are now judged in flight, and a rejection is CRITICAL when it lasts
  1 s, as long as Copter's own EKF failsafe waits (EKF_CHECK_ITERATIONS_MAX); a shorter one
  is a WARN.
- GPS: a single sample without a fix or with fewer than 4 satellites was CRITICAL, the time
  on the ground between two flights of one log was judged as flight, and a receiver that
  never had a fix (indoors) got a CRITICAL "0 satellites" on top of "never achieved a 3D
  fix". A fix loss, satellite drop or HDOP excursion is now CRITICAL when it lasts 1 s, each
  armed interval is judged on its own, and a receiver with no fix in flight gets the fix
  finding only.
- Plane: in MANUAL, STABILIZE, TRAINING and ACRO the logged roll/pitch target is no target
  (STABILIZE logs 0 while the pilot banks), so a normal banked turn was a CRITICAL "attitude
  tracking failure" and a "poor roll tracking" tuning advice. Those modes are no longer
  judged. PID advice names the vehicle's own parameters (Plane RLL_RATE_* / PTCH_RATE_*,
  not ATC_RAT_*) and is not given for Rover or Sub, which have no such parameters.
- A vehicle with no RC receiver (an ArduSub piloted by joystick, a GCS- or companion-flown
  vehicle) got a CRITICAL "RC signal loss" for the whole flight. A log that never had a
  valid RC input now gets an INFO "No RC input in this log".
- Helicopters were advised a throttle-based notch; a governed rotor does not follow throttle,
  and the helicopter wiki sets a static notch (REF 0) without an RPM sensor or ESC telemetry.
- A helicopter's default rate P (0.024; it flies on feed-forward) was "unusually low".
- A Tri's tail servo (Motor7 function) was counted as a motor, so it showed as a motor
  imbalance. Motor saturation is now judged at 95% of the vehicle's own MOT_PWM_MIN..MAX
  range instead of a fixed 1950 us, which a motor limited to 1900 us never reached; the
  evidence key `frac_above_1950_Cn` is now `frac_saturated_Cn`, with `saturation_pwm`.
- ArduSub: mode 21 (SURFTRAK) is named, the leak and depth-sensor failsafes (ERR 27, 26) are
  CRITICAL, and a "bad depth" mode change (ModeReason 28) counts as a failsafe.
- The battery cell count rounded the peak voltage at 4.2 V per cell, so a 12S pack at
  3.97 V per cell read 11S (and the derived low-voltage thresholds 11S), and a 6S pack at
  3.84 V per cell gave no estimate. Every cell count that fits 3.5-4.4 V per cell is now a
  candidate; MOT_BAT_VOLT_MAX / _MIN (4.2 / 3.3 V x cells, Copter wiki) decide between them
  when they agree with the voltage, else the one nearest 4.1 V per cell. On the 48 benchmark
  logs this matches MOT_BAT_VOLT_MAX on all 26 where that parameter fits the measured voltage.
- "Sudden voltage drop" fired on a pack sagging under a current spike (a 50 kg copter at
  120 A). When BAT.VoltR (the firmware's load-compensated resting voltage) is logged, a drop
  it does not share is now treated as load, not a loose connector.
- A non-finite parameter value (NaN or inf) in a log became `null` in `get_params`, which
  broke the tool's output schema and failed the whole call; such values are dropped, in logs
  and in `.param` files.
- A `.param` file saved with a UTF-8 byte-order mark lost its first parameter.
- `query_timeseries` with `instance` on a single-instance message returned the data anyway,
  as if it were that instance; it now says the instance is not present.
- A `.tlog` of a vehicle that never armed got an onset and the caveat "arming could not be
  determined"; it now has no onset and says the vehicle never armed.
- The vehicle profile estimated a hover throttle for a plane from its cruise throttle; only a
  multirotor gets that estimate.
- Real-crash benchmark on all 49 logs against 1.0.0: logs with a false critical 35 -> 23, false
  criticals 97 -> 36 (vibration 29 -> 0, ekf 25 -> 11, attitude 25 -> 17, gps 6 -> 1, events
  5 -> 1), top-1 11 -> 13 of 41, top-3 22 -> 22. Gained top-1: 132194_0 (the ESC-overheating
  crash, through `esc`), 144337_0 (`thrust_loss`, once a lost heading stopped outranking it),
  128873_2, 144427_0 (once a compass rejected alone stopped outranking them), 145429_0 (`config`)
  and 145616_0. Gained top-3: the two flights of 117787 through `param_audit`'s airspeed finding
  (flown at 27 m/s on a SCALING_SPEED of 13, which halves the surface throw; the labelled cause
  is a V-tail MIXING_GAIN of 0.5, another shortfall of elevator authority) and 144128_0. Lost:
  142202_0, whose hit came from a rangefinder line unrelated to its GPS cause; 138886_0 and
  145352_0, whose hits came from an EKF variance written after the disarm or after landing;
  144203_1, whose top-1 now goes to its current sensor reading 0 A, a real fault but not the
  labelled one; 139371_0 (top-1 now its compass finding) and 145616_2 (no longer in the top 3).
- Corrupt logs passed as clean. Integrity was judged from pymavlink's "bad header"
  warnings, but its default compiled indexer prints them below Python, where they cannot be
  captured. A log with garbage in the middle, or random bytes named `.bin`, came back with
  integrity `ok` (the latter as "no issues detected"). The parser now reads pymavlink's
  record index instead: bytes inside the log that belong to no record make it `partial`;
  so does a log whose reading stopped early (pymavlink stops at a record whose type has no
  format, which used to drop the rest of the flight without a word); and a file with no
  records at all is an error. On the 48 benchmark logs, no record gap exists and no finding
  changed.
- `python examples/generate_demo_log.py --help` wrote a log file named `--help`. It now
  prints its usage; the output path stays an optional argument.

### Changed
- `pymavlink>=2.4.50` (was 2.4.49). The compiled log indexer in 2.4.49 ends the whole process
  with exit status 1 when it meets an FMT record with an invalid length, so one corrupt log
  stopped the server; 2.4.50 reports it and stops reading instead (pymavlink commit 20774b3).
- `scripts/gen_docs.py` now generates the README example output, the check and
  benchmark-log counts, and the benchmark table in `bench/README.md` (from the new
  `bench/results.json`). CI fails when these sections drift from the code and
  `bench/results.json`.
- `python -m bench.run --json` refuses to write when a log was skipped, unless
  `--allow-partial` is given, and records the version and revision it measured.

## [1.0.0] — 2026-10-18

The first stable release: the tool set, the report schema and the check catalogue are now
covered by semantic versioning.

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
  the pack is connected) and any momentary sag. A threshold now counts only within one armed
  period and when the voltage stays below it for `BATT_LOW_TIMER`, as the firmware's
  failsafe does, or for at least 1 s until that armed period ends.
- "Sudden voltage drop" kept only the largest step; when that was a glitch that recovered,
  a smaller drop that persisted went unreported. The largest persistent step is reported,
  judged while armed.
- An EV, MODE or MSG record written just before the first IMU sample (up to 10 s, as when
  logging starts at arming) got a negative time; the time origin now includes them.
- A motor held exactly at 1950 us did not count as saturated; the attitude check judged
  motor saturation over the whole flight instead of the divergence window; RC loss reported
  the lowest PWM of the whole log instead of the loss window.
- Compass offsets were judged for a compass with `COMPASS_USE{n} = 0`.
- An ERR FLIGHT_MODE record names the mode the vehicle could not switch to; its ECode 0
  is mode 0 (STABILIZE / MANUAL), not a cleared error. Corrupt ERR, MODE and EV records
  (missing, NaN, inf) no longer break `list_events`; `recommend_tuning` accepts an area
  with surrounding spaces.
- Real-crash benchmark: top-1 11, top-3 22 (unchanged), false criticals 101 -> 97.
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
- `tests/test_protocol.py`: an MCP client lists the tools and runs `analyze_log` against the
  server in-process, over the initialize handshake and over 2026-07-28.
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
- **MCP Python SDK 2** (`mcp>=2.0,<3`): the server is an `MCPServer` and speaks the
  2026-07-28 protocol revision as well as the initialize handshake (2024-11-05 to
  2025-11-25) that most desktop clients use today. It now tells clients its title, version,
  website and how to use the tools (`instructions`), and marks every tool read-only,
  idempotent and closed-world. The SDK runs the tools on worker threads, so the log cache is
  locked. mcp 1.x is no longer supported; pin `ardupilot-mcp<1` to keep it.
- `pydantic>=2.12` is declared (the models use it directly). The floors `mcp 2.0`,
  `pymavlink 2.4.49`, `numpy 2.0` and `pydantic 2.12` are tested in CI, and a weekly job
  tests the newest releases of those libraries, past our upper bounds and pre-releases
  included, and opens an issue when one breaks the suite.
- **License: AGPL-3.0-only** from this release on, with a commercial license available
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

[Unreleased]: https://github.com/furkanisikay/ardupilot-mcp/compare/v1.0.0...main
[1.0.0]: https://pypi.org/project/ardupilot-mcp/1.0.0/
[0.1.3]: https://pypi.org/project/ardupilot-mcp/0.1.3/
[0.1.2]: https://pypi.org/project/ardupilot-mcp/0.1.2/
[0.1.1]: https://pypi.org/project/ardupilot-mcp/0.1.1/
[0.1.0]: https://pypi.org/project/ardupilot-mcp/0.1.0/
