# Real-crash benchmark

`real_crashes.json` lists real ArduPilot flight logs that users posted on
[discuss.ardupilot.org](https://discuss.ardupilot.org), each labelled with the cause that
someone who analysed the log stated in the thread.

## What a label means

- **confirmed**: an ArduPilot developer stated the cause, or the pilot confirmed that the fix worked.
- **expert**: an experienced helper stated the cause and the pilot accepted it.
- Threads where the replies only guessed, contradicted each other or never reached a cause
  are left out. A label quotes nobody: `rationale` is a one-sentence summary and
  `label_post` links the post it rests on.

Categories: `vibration`, `power_brownout`, `thrust_loss_motor_esc`, `compass_interference`, `gps`,
`ekf`, `rc_failsafe_link`, `tuning_oscillation`, `mechanical_frame`, `configuration`,
`pilot_error`, `firmware_bug`, `not_a_fault`.

## The logs are not here

The logs belong to the people who posted them, so they are not redistributed. Each entry has
its `source_url` and `sha256`. Download the files, name each one `<id>.bin`, and run:

```bash
python -m bench.run --logs DIR [--json scores.json]
```

A file whose sha256 differs from the manifest is skipped, so a score always refers to the
labelled flight. The runner never touches the network.

## What is scored

For each log, `bench/scoring.py` takes the report's findings in report order: before the
impact first, then the most severe.

- **top-1**: the first WARN/CRITICAL finding points at the labelled cause.
- **top-3**: one of the first three does.
- **false critical**: a CRITICAL finding that points at neither the cause nor a listed
  secondary cause. Findings that only record the crash itself (ERR `CRASH_CHECK`, the
  crash-check disarm, a crash-failsafe mode change) name no cause and are never false. On a
  `not_a_fault` flight, every other CRITICAL finding is false.
- `pilot_error`, `firmware_bug` and `not_a_fault` have no check that could name them, so they
  count only towards false criticals.

A finding points at a cause through its check (`CATEGORY_CHECKS`). For the `events`,
`firmware` and `integrity` checks, which report many subsystems, the ERR subsystem, verdict
or symptom named in the title decides it (a recording that ends mid-flight is the brown-out
signature).

## Results

Current `main`; the changelog notes how each change moved these numbers.

| Scorable logs | Top-1 | Top-3 | Logs with a false critical | False criticals |
|---|---|---|---|---|
| 41 | 9 | 21 | 35 / 49 | 113 |

False criticals by check: vibration 29, attitude 25, EKF 25, events 17, GPS 6, power 4,
RC input 4, firmware 2, compass 1. 109 of the 113 start before the impact, so they reach the
top of the report rather than the after-impact tail.
