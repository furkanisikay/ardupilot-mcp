# Sources & assumptions audit

Every factual claim, numeric threshold, enum and behavioural assumption in this
codebase, audited against authoritative sources (ArduPilot wiki/firmware, MAVLink,
MCP spec) on 2026-06-14. Generated from a fan-out verification pass over **198 assumptions**.

- **confirmed** — matches an authoritative source (cited).
- **heuristic** — a reasonable engineering threshold; closest official guidance cited.
- **design choice** — a severity cut-off we chose; not an external fact.
- **corrected** — was wrong; fixed in code (old → right value).

## Corrections applied (were wrong → fixed)

| File · location | Claim | Corrected to | Source |
|---|---|---|---|
| `ardupilot_mcp/ardupilot_meta.py` · line 89-111, EV_IDS | EV Id maps to event names (ARMED=10, DISARMED=11, TAKEOFF=16, LAND_COMPLETE=18, etc.) | Correct ids: EKF_ALT_RESET=60, LAND_CANCELLED_BY_PILOT=61, EKF_YAW_RESET=62, ZIGZAG_STORE_A=71, FENCE_FLOOR_ENABLE=80, SET_SUPERSIMPLE_ON=29; TAKEOFF is not a LogEvent. | [ArduPilot AP_Logger.h LogEvent enum](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/AP_Logger.h) |
| `ardupilot_mcp/ardupilot_meta.py` · line 151, HELI_FRAME_CLASSES | FRAME_CLASS values 6, 13, 14 represent traditional helicopters (single/dual/quad heli) | HELI_FRAME_CLASSES = {6, 11, 13} (6=Heli, 11=Heli_Dual, 13=HeliQuad). 14 = Deca (multirotor). | [ArduPilot FRAME_CLASS parameter values](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/config.py` · line 32, FS_THR_DISABLED | FS_THR_ENABLE=0 disables the RC/throttle failsafe (copter/plane/heli only) | FS_THR_ENABLE: Copter/Heli only (0=Disabled). Plane uses THR_FAILSAFE instead. | [ArduPilot FS_THR_ENABLE (Copter) vs THR_FAILSAFE (Plane)](https://autotest.ardupilot.org/Parameters/versioned/Plane/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/config.py` · line 35, FS_THR_VEHICLES | FS_THR_ENABLE only meaningfully applies to copter, plane, and heli vehicles | FS_THR_ENABLE applies to Copter and Heli (Copter firmware); Plane uses THR_FAILSAFE. | [ArduPilot Plane parameter docs (no FS_THR_ENABLE; THR_FAILSAFE present)](https://autotest.ardupilot.org/Parameters/versioned/Plane/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/config.py` · line 35, FS_THR_VEHICLES | FS_THR_ENABLE only meaningfully applies to copter, plane, heli | FS_THR_ENABLE: copter/heli. Plane uses THR_FAILSAFE. | [ArduPilot Plane params (no FS_THR_ENABLE; THR_FAILSAFE present)](https://autotest.ardupilot.org/Parameters/versioned/Plane/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/ekf.py` · SENSOR_FIELDS (SVT) | EKF field SVT is airspeed test ratio | SVT is airspeed only for XKF4/EKF3 ('Square root of the total airspeed variance'). For NKF4/EKF2, SVT is the 'tilt error convergence metric', NOT an airspeed test ratio. Treating SVT as airspeed for the NKF4 fallback is incorrect; also note the XKF4 message-level 'Squared Innovation Test Ratio' description covers SV/SP/SH/SM only (not SVT), so SVT's value is a sqrt-variance, not strictly the same normalised test ratio. | [ArduPilot AP_NavEKF2/LogStructure.h NKF4 SVT vs AP_NavEKF3/LogStructure.h XKF4 SVT](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_NavEKF2/LogStructure.h) |
| `ardupilot_mcp/checks/events.py` · KEY_EVENT_NAMES | Key lifecycle events are ARMED, DISARMED, TAKEOFF, LAND_COMPLETE, LAND_COMPLETE_MAYBE | ARMED=10, DISARMED=11, LAND_COMPLETE_MAYBE=17, LAND_COMPLETE=18; TAKEOFF is not a LogEvent id. | [ArduPilot AP_Logger.h LogEvent enum (Copter-4.5.7)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/AP_Logger.h) |
| `ardupilot_mcp/checks/events.py` · line 35-41, KEY_EVENT_NAMES | Key lifecycle events are ARMED, DISARMED, TAKEOFF, LAND_COMPLETE, LAND_COMPLETE_MAYBE | Use ARMED/DISARMED/LAND_COMPLETE/LAND_COMPLETE_MAYBE; TAKEOFF is not an EV event. | [ArduPilot AP_Logger.h LogEvent enum](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/AP_Logger.h) |

## Confirmed against authoritative sources

| File · location | Claim | Source |
|---|---|---|
| `ardupilot_mcp/ardupilot_docs.py` · line 34, versioned_param_doc_url | ArduPilot firmware version 4.0+ has versioned parameter definitions available in archive | [ArduPilot versioned parameter archive](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/ardupilot_meta.py` · line 11-37, COPTER_MODES | ArduCopter flight mode numbers 0-27 map to specific mode names (STABILIZE=0, ACRO=1, etc.) | [ArduPilot Copter mode.h Number enum](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/ArduCopter/mode.h) |
| `ardupilot_mcp/ardupilot_meta.py` · line 40-72, ERR_SUBSYSTEMS | ERR Subsys field (1-31) maps to named subsystems (MAIN=1, RADIO=2, etc.) | [ArduPilot AP_Logger.h LogErrorSubsystem enum](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/AP_Logger.h) |
| `ardupilot_mcp/ardupilot_meta.py` · line 75-86, CRITICAL_ERR_SUBSYSTEMS | Subsystem IDs 5,6,7,9,12,16,17,25,29,30 are safety-critical failsafes/errors | [ArduPilot AP_Logger.h LogErrorSubsystem enum](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/AP_Logger.h) |
| `ardupilot_mcp/ardupilot_meta.py` · line 154-169, FRAME_CLASSES | Copter FRAME_CLASS map: 1=Quad(4),2=Hexa(6),3=Octo(8),4=OctoQuad(8),5=Y6(6),6=Heli,7=Tri(3),8=Single(1),9=Coax(2),10=BiCopter(2),11=Heli_Dual,12=DodecaHexa(12),13=HeliQuad,14=Deca(10) | [ArduPilot FRAME_CLASS parameter values](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/ardupilot_meta.py` · line 199-207, estimate_cells docstring and cells = round(max_voltage / 4.2) | LiPo full voltage is 4.2 V/cell | [Voltages \| Li-Ion & LiPoly Batteries \| Adafruit Learning System](https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages) |
| `ardupilot_mcp/checks/attitude.py` · docstring | AttitudeTrackingCheck only applies to copter/heli/plane, not rovers | [ArduPilot DataFlash ATT message (LogStructure)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/attitude.py` · line 56, vehicles | Attitude tracking check only applies to copter/heli/plane (not rover) | [ArduPilot ATT message semantics](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/attitude.py` · line 88, yaw error wrapping | Yaw error is circular and wrapped into [-180, 180] degrees | [Circular heading wrapping (general principle; ArduPilot wrap_180)](https://ardupilot.org/dev/docs/learning-ardupilot-introduction.html) |
| `ardupilot_mcp/checks/attitude.py` · docstring (lines 6-7) and _axis_finding (lines 86-88) | Yaw error is wrapped into [-180, 180] because yaw is circular | [Standard angle-wrapping convention (geometric fact, no single canonical web source needed)](https://en.wikipedia.org/wiki/Thrust-to-weight_ratio) |
| `ardupilot_mcp/checks/calibration.py` · line 48-49, ACC_DEFAULT_SCALE/OFFSET | Accelerometer factory defaults: INS_ACCSCAL_X/Y/Z exactly 1.0, INS_ACCOFFS_X/Y/Z exactly 0.0 means never calibrated | [ArduPilot AP_InertialSensor.cpp (_ACCSCAL default 1.0)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_InertialSensor/AP_InertialSensor.cpp) |
| `ardupilot_mcp/checks/calibration.py` · line 53-57, _COMPASS_INSTANCES | Three compass instances: COMPASS_OFS_X/Y/Z, COMPASS_OFS2_X/Y/Z, COMPASS_OFS3_X/Y/Z | [ArduPilot COMPASS_OFS/OFS2/OFS3 parameters](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/calibration.py` · line 48-49, ACC_DEFAULT_SCALE and ACC_DEFAULT_OFFSET | Accelerometer factory defaults are scale=1.0 and offset=0.0 (never calibrated) | [ArduPilot AP_InertialSensor.cpp (_ACCSCAL 1.0)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_InertialSensor/AP_InertialSensor.cpp) |
| `ardupilot_mcp/checks/config.py` · line 30, ARMING_CHECK_DISABLED | ARMING_CHECK=0 disables all pre-arm checks; non-zero (incl. negative) leaves checks enabled | [ArduPilot ARMING_CHECK parameter (Bitmask 0:All)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/config.py` · line 31, BATT_MONITOR_NONE | BATT_MONITOR=0 means no battery monitor at all (no voltage/current sensing, no low-battery failsafe) | [ArduPilot BATT_MONITOR parameter (0=Disabled)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/config.py` · line 30, ARMING_CHECK_DISABLED | ARMING_CHECK=0 (exactly) disables all pre-arm checks; non-zero (incl. negative bitmasks) enables them | [ArduPilot ARMING_CHECK parameter (Bitmask 0:All)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/config.py` · line 31, BATT_MONITOR_NONE | BATT_MONITOR=0 means no battery monitor at all (no voltage/current sensing) | [ArduPilot BATT_MONITOR parameter (0=Disabled)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/config.py` · line 32, FS_THR_DISABLED | FS_THR_ENABLE=0 disables the RC/throttle failsafe | [ArduPilot FS_THR_ENABLE parameter (0=Disabled)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/ekf.py` · CRIT_RATIO | EKF test ratio > 1.0 means sensor rejection | [ArduPilot - EKF Failsafe / Diagnosing problems using logs](https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html) |
| `ardupilot_mcp/checks/ekf.py` · line 30, CRIT_RATIO | EKF variance test ratio > 1.0 means sensor rejected (rejection boundary) | [ArduPilot EKF / Diagnosing problems using logs](https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html) |
| `ardupilot_mcp/checks/ekf.py` · line 35-36 | XKF4 (EKF3) preferred; NKF4 (EKF2) legacy fallback with same layout | [ArduPilot NavEKF3/NavEKF2 LogStructure (XKF4/NKF4)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_NavEKF3/LogStructure.h) |
| `ardupilot_mcp/checks/ekf.py` · SENSOR_FIELDS / PREFERRED_MSG / FALLBACK_MSG | EKF3 message is XKF4; EKF2 fallback is NKF4 | [ArduPilot AP_NavEKF3/LogStructure.h (XKF4) and AP_NavEKF2/LogStructure.h (NKF4)](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_NavEKF3/LogStructure.h) |
| `ardupilot_mcp/checks/ekf.py` · SENSOR_FIELDS (SV) | EKF field SV is velocity test ratio | [ArduPilot AP_NavEKF3/LogStructure.h XKF4 field SV](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_NavEKF3/LogStructure.h) |
| `ardupilot_mcp/checks/ekf.py` · SENSOR_FIELDS (SP) | EKF field SP is position test ratio | [ArduPilot AP_NavEKF3/LogStructure.h XKF4 field SP](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_NavEKF3/LogStructure.h) |
| `ardupilot_mcp/checks/ekf.py` · SENSOR_FIELDS (SH) | EKF field SH is height test ratio | [ArduPilot AP_NavEKF3/LogStructure.h XKF4 field SH](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_NavEKF3/LogStructure.h) |
| `ardupilot_mcp/checks/ekf.py` · SENSOR_FIELDS (SM) | EKF field SM is magnetometer test ratio | [ArduPilot AP_NavEKF3/LogStructure.h XKF4 field SM](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_NavEKF3/LogStructure.h) |
| `ardupilot_mcp/checks/events.py` · docstring | ERR ECode == 0 means error clearing; ECode != 0 is error onset | [ArduPilot ERR / LogErrorCode (ERROR_RESOLVED=0)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/AP_Logger.h) |
| `ardupilot_mcp/checks/events.py` · line 70, ECode == 0 | ERR with ECode=0 clears a previously raised error (not a finding) | [ArduPilot ERR / LogErrorCode](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/AP_Logger.h) |
| `ardupilot_mcp/checks/events.py` · docstring | ERR message has fields Subsys and ECode | [ArduPilot AP_Logger/LogStructure.h ERR definition](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/events.py` · docstring | MODE message has fields Mode (name) and ModeNum | [ArduPilot AP_Logger/LogStructure.h MODE definition](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/events.py` · docstring | EV message has field Id (discrete event identifier) | [ArduPilot AP_Logger/LogStructure.h EV definition](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/gps.py` · FIX_3D | GPS Status >= 3 is a usable 3D fix | [ArduPilot AP_GPS.h (Copter-4.5.7)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_GPS/AP_GPS.h) |
| `ardupilot_mcp/checks/gps.py` · SATS_CRIT | Below 4 satellites a 3D fix cannot be held | [GNSS positioning (4-satellite requirement) - general GNSS principle](https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html) |
| `ardupilot_mcp/checks/gps.py` · docstring | Good HDOP is under ~1.5 | [ArduPilot - Diagnosing problems using logs](https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html) |
| `ardupilot_mcp/checks/gps.py` · docstring | GPS Status enum: 0/1 = no fix, 2 = 2D, 3 = 3D, 4 = DGPS, 5 = RTK float, 6 = RTK fixed | [ArduPilot AP_GPS.h (Copter-4.5.7)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_GPS/AP_GPS.h) |
| `ardupilot_mcp/checks/gps.py` · line 31, FIX_3D | GPS Status >= 3 is a usable 3D fix | [ArduPilot AP_GPS.h GPS_Status enum](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_GPS/AP_GPS.h) |
| `ardupilot_mcp/checks/gps.py` · line 38-39, HDOP_WARN and HDOP_CRIT | Good HDOP <~1.5; GPS_HDOP_GOOD arming gate 1.4; >2 poor; >5 unusable | [ArduPilot config.h GPS_HDOP_GOOD_DEFAULT 140](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/ArduCopter/config.h) |
| `ardupilot_mcp/checks/integrity.py` · docstring | LogIntegrity.OK means clean parse, nothing to flag | Matches the codebase: model.py defines LogIntegrity.OK = 'ok |
| `ardupilot_mcp/checks/integrity.py` · docstring | LogIntegrity.TRUNCATED means byte stream ends abruptly without clean shutdown | [Project parser.py _assess_integrity (internal); pymavlink DFReader bad-header/bad-msg warnings](https://raw.githubusercontent.com/ArduPilot/pymavlink/master/DFReader.py) |
| `ardupilot_mcp/checks/integrity.py` · docstring | LogIntegrity.PARTIAL means some messages could not be decoded | Matches model.py comment ('parsed but errors were encountere |
| `ardupilot_mcp/checks/motors.py` · docstring | PWM range is nominally ~1000 (idle/min) to ~2000 (full) | [ArduPilot MOT_PWM_MIN/MOT_PWM_MAX parameter docs](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/motors.py` · docstring | MotorsCheck only applies to copter (multirotor), not heli/plane/rover | [ArduPilot RCOU message / heli swashplate output](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/motors.py` · line 56, vehicles | Motor balance check only applies to copter (not heli/plane/rover) | [ArduPilot RCOU output semantics by vehicle](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/motors.py` · MOTOR_CHANNELS | RCOU has channels C1-C8 for motor outputs | [ArduPilot AP_Logger/LogStructure.h RCOU definition](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/param_audit.py` · line 23-26 | Copter rate-controller P gain reference firmware default ~0.135; <0.04 unusually low; >0.5 unusually high | [ArduPilot AC_AttitudeControl_Multi.h (RATE_RP_P 0.135f)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h) |
| `ardupilot_mcp/checks/param_audit.py` · line 27 | Vehicle kind 'copter','heli', or None are multirotors for rate/accel checks; None still evaluated | [ArduPilot ATC_RAT_*/ATC_ACCEL_* parameters (Copter/Heli)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 58 | ATC_ACCEL_R_MAX/P_MAX/Y_MAX=0 disables angular acceleration limit for roll/pitch/yaw (oscillation crash cause) | [ArduPilot ATC_ACCEL_R_MAX/P_MAX/Y_MAX (0=Disabled)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 64 | ATC_ACCEL_R/P default ~110000 cdeg/s/s for roll/pitch (yaw not specified) | [ArduPilot AC_AttitudeControl.h ACCEL_RP_MAX_DEFAULT_CDSS 110000](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AC_AttitudeControl/AC_AttitudeControl.h) |
| `ardupilot_mcp/checks/param_audit.py` · line 75 | ATC_RAT_RLL_P=0 or ATC_RAT_PIT_P=0 disables roll/pitch rate-loop P term, making stable flight impossible | [ArduPilot ATC_RAT_RLL_P/PIT_P (range 0.01-0.5)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 80 | Copter default rate P gain is ~0.135 (used in recommendation) | [ArduPilot AC_AttitudeControl_Multi.h](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h) |
| `ardupilot_mcp/checks/param_audit.py` · line 124 | MOT_PWM_MIN >= MOT_PWM_MAX means inverted/empty range; MOT_PWM_MIN=0 and MOT_PWM_MAX=0 means 'use RC range' (not an error) | [ArduPilot MOT_PWM_MIN/MOT_PWM_MAX parameter docs](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 128 | Typical MOT_PWM values: ~1000 (min) and ~2000 (max) | [ArduPilot MOT_PWM_MIN/MAX parameter docs](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 131 | INS_HNTCH_ENABLE=1 with INS_HNTCH_FREQ=0 means notch enabled but unconfigured (no attenuation) | [ArduPilot INS_HNTCH_ENABLE / INS_HNTCH_FREQ (range 10-495 Hz)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 142 | FENCE_ENABLE=1 with FENCE_ALT_MAX=0 and FENCE_RADIUS=0 means geofence enabled with no boundary | [ArduPilot FENCE_ENABLE/FENCE_ALT_MAX/FENCE_RADIUS params](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 24-26, RATE_P constants | Copter rate P default ~0.135; <0.04 unusually low, >0.5 unusually high | [ArduPilot AC_AttitudeControl_Multi.h + ATC_RAT_*_P range](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h) |
| `ardupilot_mcp/checks/param_audit.py` · line 58, ATC_ACCEL_*_MAX=0 | ATC_ACCEL_R/P/Y_MAX=0 disables angular acceleration limit (oscillation crash cause) | [ArduPilot ATC_ACCEL_*_MAX (0=Disabled)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 104, BATT_CRT_VOLT >= BATT_LOW_VOLT | BATT_CRT_VOLT must be below BATT_LOW_VOLT for staged battery failsafe response | [ArduPilot Battery Failsafe (BATT_LOW_VOLT/BATT_CRT_VOLT)](https://ardupilot.org/copter/docs/failsafe-battery.html) |
| `ardupilot_mcp/checks/param_audit.py` · line 115, MOT_SPIN_ARM > MOT_SPIN_MIN | MOT_SPIN_ARM > MOT_SPIN_MIN is backwards (idle faster when armed than min flight throttle) | [ArduPilot MOT_SPIN_ARM/MOT_SPIN_MIN params (defaults 0.10/0.15)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 124, MOT_PWM_MIN >= MOT_PWM_MAX | MOT_PWM_MIN >= MOT_PWM_MAX means range empty/inverted; 0 means use RC range | [ArduPilot MOT_PWM_MIN/MAX parameter docs](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 131, INS_HNTCH_ENABLE && INS_HNTCH_FREQ=0 | Harmonic notch enabled but center frequency=0 is misconfiguration (does nothing) | [ArduPilot INS_HNTCH_FREQ parameter (range 10-495)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/param_audit.py` · line 142, FENCE_ALT_MAX=0 && FENCE_RADIUS=0 | Geofence enabled with altitude and radius both 0 has no actual boundary | [ArduPilot FENCE_ALT_MAX/FENCE_RADIUS params](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/power.py` · docstring | ArduPilot failsafe parameters are BATT_LOW_VOLT and BATT_CRT_VOLT | [ArduPilot Copter parameter docs (apm.pdef.xml)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/power.py` · PER_CELL_LOW_V (line 30) | LiPo low voltage is ~3.5 V/cell | [Beginners Guide to LiPo Batteries - FPV Freedom Coalition](https://fpvfc.org/beginners-guide-to-lipo-batteries) |
| `ardupilot_mcp/checks/power.py` · PER_CELL_CRIT_V (line 31) | LiPo critical voltage is ~3.3 V/cell | [Complete Guide to LiPo Battery Voltage - Ufine Battery](https://www.ufinebattery.com/blog/useful-overview-of-lipo-battery-voltage/) |
| `ardupilot_mcp/checks/power.py` · line 30-31, PER_CELL_LOW_V and PER_CELL_CRIT_V | LiPo per-cell thresholds: ~3.5 V/cell low, ~3.3 V/cell critical | [Beginners Guide to LiPo Batteries - FPV Freedom Coalition (land at 3.5 V/cell)](https://fpvfc.org/beginners-guide-to-lipo-batteries) |
| `ardupilot_mcp/checks/power.py` · _BATTERY_SOURCES | Battery message is BAT (preferred) or CURR (older firmware) | [ArduPilot AP_BattMonitor/LogStructure.h (BAT) and legacy Copter-3.5 DataFlash/LogStructure.h (CURR)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-3.5/libraries/DataFlash/LogStructure.h) |
| `ardupilot_mcp/checks/power.py` · docstring | Battery message fields are Volt (pack V) and Curr (A) | [ArduPilot AP_BattMonitor/LogStructure.h BAT fields Volt/Curr](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_BattMonitor/LogStructure.h) |
| `ardupilot_mcp/checks/rcin.py` · _PRIMARY | Primary RC channels are C1 (roll), C2 (pitch), C3 (throttle), C4 (yaw) | [ArduPilot RCMAP_ parameters (default channel mapping)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/rcin.py` · docstring | Sticks sit near centre (~1500 us) in normal flight | [ArduPilot RCx_TRIM (default 1500)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/rcin.py` · line 28, _PRIMARY | Primary RC channels C1-C4 are roll, pitch, throttle, yaw | [ArduPilot RCMAP defaults](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/sensors.py` · line 38-40, RNGFND message families | Modern firmware uses RNGFND1_TYPE param and RFND message; older firmware used un-numbered RNGFND_TYPE and RNGFND message | [ArduPilot RNGFND1_TYPE param + RFND LogStructure](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/sensors.py` · line 42-44 | GPS_TYPE > 0 declares a GPS receiver; GPS_MESSAGE='GPS' proves actual data logged | [ArduPilot GPS_TYPE parameter + GPS log message](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/timing.py` · line 49, PM_LONG_LOOP_FIELDS | PM scheduler message uses 'NLon' field for count of long loops | [ArduPilot PM message LogStructure (NLon)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/timing.py` · PM_LONG_LOOP_FIELDS | PM message field NLon is the long-loop counter | [ArduPilot AP_Logger/LogStructure.h PM definition](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/checks/vibration.py` · WARN_MS2 | Vibration below ~30 m/s^2 is healthy | [ArduPilot - Measuring Vibration](https://ardupilot.org/copter/docs/common-measuring-vibration.html) |
| `ardupilot_mcp/checks/vibration.py` · CRIT_MS2 | Vibration above 60 m/s^2 is likely to degrade the EKF | [ArduPilot - Measuring Vibration](https://ardupilot.org/copter/docs/common-measuring-vibration.html) |
| `ardupilot_mcp/checks/vibration.py` · line 19-20, WARN_MS2 and CRIT_MS2 | ArduPilot guidance: vibration <30 m/s^2 healthy, 30-60 marginal, >60 likely degrades EKF | [ArduPilot - Measuring Vibration](https://ardupilot.org/copter/docs/common-measuring-vibration.html) |
| `ardupilot_mcp/flight_log.py` · line 83, _CLOCK_REFERENCE_MSGS | IMU, IMU2, ATT, CTUN, RATE, NKF1, XKF1, BARO, RCOU are high-rate FC messages with reliable boot clock | [ArduPilot DataFlash LogStructure (TimeUS clock)](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/flight_log.py` · line 42-45, _timestamp_of | TimeUS field is microseconds since boot; TimeMS field is milliseconds since boot | [ArduPilot pymavlink DFReader.py timestamp math (TimeUS*1e-6, TimeMS*1e-3)](https://raw.githubusercontent.com/ArduPilot/pymavlink/master/DFReader.py) |
| `ardupilot_mcp/parser.py` · line 20, _SKIP_TYPES | DataFlash message types FMT, FMTU, UNIT, MULT are structural metadata only and should be skipped | [ArduPilot AP_Logger/LogStructure.h FMT/FMTU/UNIT/MULT descriptions](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Logger/LogStructure.h) |
| `ardupilot_mcp/parser.py` · line 62-64, comments | DFReader._timestamp is Unix epoch (~1.7e9 s), which is unreliable for GPS-clocked logs | [ArduPilot pymavlink DFReader.py _gpsTimeToTime / set_message_timestamp](https://raw.githubusercontent.com/ArduPilot/pymavlink/master/DFReader.py) |
| `ardupilot_mcp/profile.py` · line 46 | MOT_THST_HOVER present and 0.0 < value <= 1.0 gives learned hover throttle estimate (preferred source) | [ArduPilot MOT_THST_HOVER parameter (0..1, learned)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/profile.py` · line 65 | Battery cell count estimated from max voltage using meta.estimate_cells() | [LiPo cell voltage (4.2V full) - ArduPilot battery monitoring](https://ardupilot.org/copter/docs/common-powermodule-landingpage.html) |
| `ardupilot_mcp/profile.py` · line 73 (nominal_v = cells * 3.7) | Nominal battery voltage = cells * 3.7 V per cell | [Voltages \| Li-Ion & LiPoly Batteries \| Adafruit Learning System](https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages) |
| `ardupilot_mcp/profile.py` · line 69 (t2w = 1.0 / hover) and assessment text lines 95-107 | Thrust-to-weight rough estimate = 1.0 / hover (when hover > 0) | [Thrust-to-Weight Ratio Calculator for Drones - Fly Eye (50% throttle = 2:1 TWR rule of thumb)](https://www.flyeye.io/drone-calculators-thrust-to-weight-ratio/) |
| `ardupilot_mcp/profile.py` · line 73, nominal_v calculation | Nominal LiPo voltage is 3.7 V/cell | [Voltages \| Li-Ion & LiPoly Batteries \| Adafruit Learning System](https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages) |

## Heuristic thresholds (our conservative choices)

| File · location | Claim | Source |
|---|---|---|
| `ardupilot_mcp/ardupilot_meta.py` · line 205-211, estimate_cells (per_cell range 3.3-4.4, min 4.0 V, max 70 V) | Per-cell voltage range 3.3-4.4 V is sane for LiPo batteries; min pack voltage 4.0V, max 70V to detect valid monitor | [Complete Guide to LiPo Battery Voltage - Ufine (4.2 V full, 4.25 specialty, 3.0 floor, 3.2-3.3 cutoff)](https://www.ufinebattery.com/blog/useful-overview-of-lipo-battery-voltage/) |
| `ardupilot_mcp/checks/calibration.py` · line 45, LARGE_OFFSET_MGAUSS | Compass hard-iron offset magnitude > 600 mGauss indicates poor calibration; healthy < ~300 mGauss | [ArduPilot COMPASS_OFS_X parameter (units mGauss, range -400..400)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/compass.py` · docstring | Clean compass sits well under 0.1 coefficient of variation | [ArduPilot - Diagnosing problems using logs (compass interference)](https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html) |
| `ardupilot_mcp/checks/gps.py` · SATS_WARN | Below 6 satellites the GPS solution is marginal | [ArduPilot - Diagnosing problems using logs](https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html) |
| `ardupilot_mcp/checks/gps.py` · HDOP_CRIT | HDOP above 5.0 means horizontal position is effectively unusable | [ArduPilot - Diagnosing problems using logs](https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html) |
| `ardupilot_mcp/checks/gps.py` · HDOP_SENTINEL | When there is no fix receiver reports HDOP ~99.99 as sentinel | [ArduPilot AP_GPS.h GPS_UNKNOWN_DOP](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_GPS/AP_GPS.h) |
| `ardupilot_mcp/checks/gps.py` · line 34-35, SATS_WARN and SATS_CRIT | GPS wants roughly 6+ satellites; below 4 cannot maintain 3D fix | [ArduPilot - Diagnosing problems using logs](https://ardupilot.org/copter/docs/common-diagnosing-problems-using-logs.html) |
| `ardupilot_mcp/checks/gps.py` · line 42, HDOP_SENTINEL | No-fix receiver reports sentinel HDOP ~99.99, not real dilution | [ArduPilot AP_GPS.h GPS_UNKNOWN_DOP](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.5.7/libraries/AP_GPS/AP_GPS.h) |
| `ardupilot_mcp/checks/integrity.py` · _TRUNCATED_FLAG | LogIntegrity.TRUNCATED is flagged numerically as 1.0 | There is no external/canonical standard for this number. It  |
| `ardupilot_mcp/checks/integrity.py` · _PARTIAL_FLAG | LogIntegrity.PARTIAL is flagged numerically as 2.0 | Internal sentinel only, no external authority. Packed into s |
| `ardupilot_mcp/checks/integrity.py` · _UNKNOWN_FLAG | Unknown LogIntegrity state is flagged numerically as 9.0 | Internal defensive sentinel only, no external authority. 9.0 |
| `ardupilot_mcp/checks/motors.py` · ACTIVE_MIN_PWM | Channel mean PWM above 1100 us is an active motor; below is idle/unused | [ArduPilot MOT_PWM_MIN/MAX parameter docs](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/motors.py` · SATURATION_PWM | PWM at or above 1950 us is treated as near maximum | [ArduPilot MOT_PWM_MAX parameter docs](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/power.py` · PLAUSIBLE_MIN_V (line 51) | Pack voltage below 4.0 V indicates no real battery monitor | [Voltages \| Li-Ion & LiPoly Batteries \| Adafruit Learning System (confirms 4.2 V/cell full)](https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages) |
| `ardupilot_mcp/checks/power.py` · PLAUSIBLE_MAX_V (line 52) | Pack voltage above 70.0 V indicates odd scale or raw ADC reading | [Adafruit Li-Ion/LiPo Voltages (4.2 V/cell full -> 16S = 67.2 V)](https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages) |
| `ardupilot_mcp/checks/power.py` · line 51-52, PLAUSIBLE_MIN_V and PLAUSIBLE_MAX_V | Plausible pack voltage range 4-70V; outside this no real battery monitor | [Adafruit Li-Ion/LiPo Voltages](https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages) |
| `ardupilot_mcp/checks/rcin.py` · NO_SIGNAL_US | RC channel below 1000 us is outside normal range | [ArduPilot RC input ranges (RCx_MIN)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |
| `ardupilot_mcp/checks/rcin.py` · DEAD_US | RC channel at/near 900 us is a hard no-pulse | [ArduPilot Radio Failsafe / RC ranges](https://ardupilot.org/copter/docs/radio-failsafe.html) |
| `ardupilot_mcp/checks/timing.py` · GAP_SOURCES | Highest-rate messages for gap analysis are IMU > ATT > VIBE | [ArduPilot logging rates (LOG_BITMASK)](https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html) |
| `ardupilot_mcp/checks/timing.py` · line 34, GAP_SOURCES | High-rate reference message sources in order: IMU, ATT, VIBE (used for gap analysis) | [ArduPilot logging configuration (LOG_BITMASK)](https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html) |
| `ardupilot_mcp/checks/vibration.py` · docstring | Any accelerometer clipping is a hard problem | [ArduPilot - Measuring Vibration](https://ardupilot.org/copter/docs/common-measuring-vibration.html) |
| `ardupilot_mcp/server.py` · line 44, _LOW_PARAM_COUNT | Full ArduPilot config is ~800-1400 params; <150 suggests incomplete dump | [ArduPilot full parameter list (apm.pdef.xml)](https://autotest.ardupilot.org/Parameters/versioned/Copter/stable-4.5.7/apm.pdef.xml) |

## Design choices (our severity cut-offs — not external facts)

| File · location | Threshold / choice |
|---|---|
| `ardupilot_mcp/checks/attitude.py` · SUSTAINED_S | Divergence held for 1.0 second or more is sustained |
| `ardupilot_mcp/checks/attitude.py` · CRIT_DEG | Sustained attitude error above 25 degrees is critical loss-of-tracking |
| `ardupilot_mcp/checks/attitude.py` · WARN_DEG | Sustained attitude error above 15 degrees is concerning |
| `ardupilot_mcp/checks/attitude.py` · BRIEF_WARN_DEG | Brief error above 30 degrees warrants warning even if self-corrects |
| `ardupilot_mcp/checks/attitude.py` · MOTOR_SATURATION_US | RCOU PWM above 1950 us is treated as motor saturation |
| `ardupilot_mcp/checks/attitude.py` · line 29, SUSTAINED_S | Attitude error held for 1.0+ seconds is 'sustained' rather than transient |
| `ardupilot_mcp/checks/attitude.py` · line 31, CRIT_DEG | Sustained attitude error > 25 degrees is a loss-of-control emergency |
| `ardupilot_mcp/checks/attitude.py` · line 33, WARN_DEG | Sustained attitude error > 15 degrees is concerning but not critical |
| `ardupilot_mcp/checks/attitude.py` · line 35, BRIEF_WARN_DEG | Brief error > 30 degrees still warrants warning even if self-corrects |
| `ardupilot_mcp/checks/attitude.py` · line 37, MOTOR_SATURATION_US | Motor RCOU PWM >1950 us is treated as saturated/maxed-out during attitude analysis |
| `ardupilot_mcp/checks/calibration.py` · line 115 | Check compass zero-offset condition only for primary compass (instance 1) when device present to avoid false positives on defaulted secondary mags |
| `ardupilot_mcp/checks/calibration.py` · line 45, LARGE_OFFSET_MGAUSS | Compass hard-iron offset > 600 mGauss indicates poor calibration or interference (healthy <~300) |
| `ardupilot_mcp/checks/calibration.py` · line 115-117 | Compass offset check only flags primary compass (n=1) when enabled and device present |
| `ardupilot_mcp/checks/compass.py` · WARN_CV | Compass coefficient of variation > 0.30 indicates interference |
| `ardupilot_mcp/checks/compass.py` · CRIT_CV | Compass coefficient of variation > 0.60 is critical interference |
| `ardupilot_mcp/checks/compass.py` · WARN_RANGE_RATIO | Peak-to-peak spread (max-min)/mean > 0.60 indicates interference |
| `ardupilot_mcp/checks/compass.py` · MIN_SAMPLES | Need at least 10 samples for compass statistics to be meaningful |
| `ardupilot_mcp/checks/compass.py` · line 27-28, WARN_CV and CRIT_CV | Compass coefficient of variation: clean <0.1, warn >0.30, critical >0.60 (interference) |
| `ardupilot_mcp/checks/compass.py` · line 31, WARN_RANGE_RATIO | Compass peak-to-peak spread >0.60 of mean is a warning |
| `ardupilot_mcp/checks/compass.py` · line 33, MIN_SAMPLES | Need minimum 10 samples before compass statistics are meaningful |
| `ardupilot_mcp/checks/ekf.py` · WARN_RATIO | EKF test ratio > 0.8 means sensor variance is high |
| `ardupilot_mcp/checks/ekf.py` · line 32, WARN_RATIO | EKF variance test ratio > 0.8 means filter is straining (approaching rejection) |
| `ardupilot_mcp/checks/gps.py` · HDOP_WARN | HDOP above 2.0 means geometry is getting poor |
| `ardupilot_mcp/checks/motors.py` · IMBALANCE_WARN_PWM | Motor output imbalance spread > 150 PWM is a warning |
| `ardupilot_mcp/checks/motors.py` · IMBALANCE_CRIT_PWM | Motor output imbalance spread > 300 PWM is critical |
| `ardupilot_mcp/checks/motors.py` · SATURATION_WARN_FRACTION | Motor spending > 20% of flight saturated is a warning |
| `ardupilot_mcp/checks/motors.py` · SATURATION_CRIT_FRACTION | Motor spending > 50% of flight saturated is critical |
| `ardupilot_mcp/checks/motors.py` · line 34, ACTIVE_MIN_PWM | Motor channel active if mean PWM > 1100 us (real motor vs disarmed noise at ~1000) |
| `ardupilot_mcp/checks/motors.py` · line 37-38, IMBALANCE_WARN_PWM and IMBALANCE_CRIT_PWM | Motor imbalance spread: >150 PWM is WARN, >300 PWM is CRITICAL |
| `ardupilot_mcp/checks/motors.py` · line 41, SATURATION_PWM | Motor PWM >= 1950 us is near maximum (typical max ~2000) |
| `ardupilot_mcp/checks/motors.py` · line 43-44, SATURATION_WARN_FRACTION and SATURATION_CRIT_FRACTION | Motor saturation: >20% of flight is WARN, >50% is CRITICAL |
| `ardupilot_mcp/checks/param_audit.py` · line 24-26 | RATE_P_LOW=0.04 (well below default); RATE_P_HIGH=0.5 (above usual valid range) |
| `ardupilot_mcp/checks/param_audit.py` · line 24-26 | RATE_P_HIGH=0.5 threshold for unusually high rate gain |
| `ardupilot_mcp/checks/param_audit.py` · line 104 | BATT_CRT_VOLT (critical battery failsafe) must be below BATT_LOW_VOLT (low battery warning) to maintain staged response; CRT_VOLT >= LOW_VOLT is a misconfiguration |
| `ardupilot_mcp/checks/param_audit.py` · line 110 | Recommendation: set BATT_CRT_VOLT ~0.2-0.3 V/cell below BATT_LOW_VOLT |
| `ardupilot_mcp/checks/param_audit.py` · line 115 | MOT_SPIN_ARM > MOT_SPIN_MIN is misconfiguration: motors idle faster when armed than minimum flight throttle, backwards control |
| `ardupilot_mcp/checks/power.py` · DROP_WARN_V | Voltage decrease > 2.0 V in one fast step looks like connector dropout |
| `ardupilot_mcp/checks/power.py` · DROP_CRIT_V | Voltage decrease > 4.0 V in one fast step is critical brown-out |
| `ardupilot_mcp/checks/power.py` · DROP_MAX_DT_S | Only consider voltage deltas across gaps shorter than 0.5 seconds |
| `ardupilot_mcp/checks/power.py` · HIGH_CURRENT_A | Current >= 20.0 A is treated as high current |
| `ardupilot_mcp/checks/power.py` · SAG_MIN_V | Only surface sag INFO if spread under load is at least 1.0 volt |
| `ardupilot_mcp/checks/power.py` · line 39-41, DROP_WARN_V, DROP_CRIT_V, DROP_MAX_DT_S | Sudden voltage drop: >2V in <0.5s is WARN, >4V is CRITICAL (connector dropout signature) |
| `ardupilot_mcp/checks/power.py` · line 44, HIGH_CURRENT_A | Current >= 20 A is considered 'high current' for sag analysis |
| `ardupilot_mcp/checks/power.py` · line 46, SAG_MIN_V | Only report sag INFO if voltage span under load is >= 1.0V |
| `ardupilot_mcp/checks/prearm.py` · line 28-42, NOTABLE_TOKENS and WARN_TOKENS | Startup messages containing 'prearm', 'arm:', 'not calibrated', 'inconsistent', 'unhealthy', 'ground mag anomaly', 'error', 'bad ', 'failed', 'failsafe', ' reset', 'glitch', 'denied' are notable; subset of these (prearm, error, failed, bad , denied, not calibrated) are WARN severity |
| `ardupilot_mcp/checks/prearm.py` · line 58-71, ROUTINE_TOKENS | Startup lines containing 'initialis', 'initializ', 'alignment complete', ' ready', 'detected as', 'rcout', 'rcin', 'frame:', 'param space', 'u-blox', 'gps 1:', 'gps 2:' are routine boot chatter and ignored |
| `ardupilot_mcp/checks/prearm.py` · line 75, MAX_FINDINGS | Maximum 6 startup/pre-arm findings emitted; if more exist, emit first 6 and note overflow |
| `ardupilot_mcp/checks/prearm.py` · line 88-89 | Line containing 'calibrated' is routine unless it also contains 'not ' (e.g., 'not calibrated' is notable, 'Compass calibrated' is routine) |
| `ardupilot_mcp/checks/rcin.py` · SUSTAINED_S | All primary channels low for 0.5 second or more = link loss |
| `ardupilot_mcp/checks/rcin.py` · line 22, NO_SIGNAL_US | RC channel below 1000 us is outside normal range (min stick ~1000) |
| `ardupilot_mcp/checks/rcin.py` · line 24, SUSTAINED_S | All primary RC channels low for >= 0.5 seconds indicates sustained link loss |
| `ardupilot_mcp/checks/rcin.py` · line 26, DEAD_US | RC channel pinned at/near 900 us is a hard no-pulse condition |
| `ardupilot_mcp/checks/sensors.py` · line 46-48, MAG_UNHEALTHY_FRACTION | Compass MAG.Health flag: 1=healthy, 0=unhealthy; unhealthy fraction > 20% indicates failing/disconnected magnetometer |
| `ardupilot_mcp/checks/sensors.py` · line 48, MAG_UNHEALTHY_FRACTION | Compass unhealthy for > 20% of flight indicates failing/disconnected magnetometer |
| `ardupilot_mcp/checks/timing.py` · GAP_ABS_FLOOR_S | Never flag gap smaller than 0.5 seconds |
| `ardupilot_mcp/checks/timing.py` · GAP_REL_FACTOR | Never flag gap smaller than 10x the nominal sample spacing |
| `ardupilot_mcp/checks/timing.py` · MAX_GAP_FINDINGS | Report at most 3 gap findings per log |
| `ardupilot_mcp/checks/timing.py` · line 40, GAP_ABS_FLOOR_S | Never flag a logging gap smaller than 0.5 seconds |
| `ardupilot_mcp/checks/timing.py` · line 41, GAP_REL_FACTOR | Gap must be 10x the nominal sample spacing to be flagged |
| `ardupilot_mcp/checks/timing.py` · line 45, MAX_GAP_FINDINGS | Report at most 3 gap findings to avoid flooding output |
| `ardupilot_mcp/checks/vibration.py` · SUSTAINED_FRACTION | 10% of the flight above WARN threshold turns marginal note into warning |
| `ardupilot_mcp/checks/vibration.py` · _clip_findings | Accelerometer clipping count >= 100 is CRITICAL severity |
| `ardupilot_mcp/checks/vibration.py` · line 22, SUSTAINED_FRACTION | Fraction 0.10 (10%) of flight above WARN threshold turns marginal into warning |
| `ardupilot_mcp/checks/vibration.py` · line 110, clip finding severity | 100+ accelerometer clipping events is CRITICAL; <100 is WARN |
| `ardupilot_mcp/flight_log.py` · line 144, fence for IQR calculation | IQR fence of 5.0 (5x IQR) is far enough that clean data is never trimmed (outlier detection) |
| `ardupilot_mcp/profile.py` · line 18-19, PLAUSIBLE_MIN_V and PLAUSIBLE_MAX_V | Battery voltage must be within 4.0-70.0 V range to be plausible; outside this range is considered implausible |
| `ardupilot_mcp/profile.py` · line 21, HOVER_UNDERPOWERED | Hover throttle >= 0.62 (62%) indicates underpowered/heavy aircraft with little headroom for climb/wind/maneuver |
| `ardupilot_mcp/profile.py` · line 22, HOVER_OVERPOWERED | Hover throttle <= 0.30 (30%) indicates very overpowered aircraft |
| `ardupilot_mcp/profile.py` · line 52 | Fallback hover estimate: use median throttle-out while flying (from CTUN.ThO with 0.1 < throttle < 0.95); requires >20 samples |
| `ardupilot_mcp/profile.py` · line 52 | CTUN.ThO throttle flying range: 0.1 to 0.95 (fraction 0..1) filtered for median calculation |
| `ardupilot_mcp/profile.py` · line 89 | Hover throttle analysis only applies to copter and heli vehicle kinds; plane excluded |
| `ardupilot_mcp/profile.py` · line 68 | Power margin = (1.0 - hover) * 100.0, expressed as percentage of throttle headroom above hover |
| `ardupilot_mcp/profile.py` · line 21, HOVER_UNDERPOWERED | Hover throttle >= 0.62 means little headroom for climb/wind/maneuver (underpowered/heavy) |
| `ardupilot_mcp/profile.py` · line 22, HOVER_OVERPOWERED | Hover throttle <= 0.30 is very overpowered |
| `ardupilot_mcp/profile.py` · line 52, flying range for CTUN | Throttle in range (0.1, 0.95) is considered clearly flying (for hover throttle estimation fallback) |
| `ardupilot_mcp/profile.py` · line 53, MIN_FLYING_SAMPLES | Need > 20 samples to trust median throttle estimate (implicit in code logic) |
| `ardupilot_mcp/tuning.py` · line 31, NOTCH_MIN_HZ | Ignore frequencies below 10 Hz when searching for motor-noise peak (airframe motion dominates) |
| `ardupilot_mcp/tuning.py` · line 33, NOTCH_MAX_FS_FRACTION | Search motor-noise frequency up to 45% of sample rate (conservative slice of Nyquist) |
| `ardupilot_mcp/tuning.py` · line 36, NOTCH_MIN_FS_HZ | Need minimum 100 Hz IMU sample rate to resolve meaningful motor peak |
| `ardupilot_mcp/tuning.py` · line 38, NOTCH_MIN_SAMPLES | Minimum 64 IMU samples needed for usable FFT |
| `ardupilot_mcp/tuning.py` · line 41, NOTCH_PEAK_RATIO | Motor-noise peak must be at least 5x the median in-band power to count as a clear peak |
| `ardupilot_mcp/tuning.py` · line 46, PID_RMS_WARN_DEG | RMS attitude tracking error above 5 degrees is poor enough to advise rate-PID review (advisory only) |
