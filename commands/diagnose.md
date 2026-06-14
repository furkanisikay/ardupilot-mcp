---
description: Diagnose an ArduPilot .bin flight log using the ardupilot-mcp tools
---

Diagnose the ArduPilot DataFlash log at: $ARGUMENTS

Use the `ardupilot-mcp` MCP tools and ground every claim in their output:

1. **`analyze_log(path)` — run this first.** It is the authoritative result: severity-ranked
   findings, each carrying official ArduPilot doc links, plus a summary. Everything you say
   should trace back to it.
2. **`vehicle_profile(path)`** — frame, motor count, battery cells, hover throttle, power
   margin and thrust-to-weight. Use it to judge whether the airframe was physically capable
   (e.g. underpowered, marginal headroom) rather than guessing.
3. **Drill into any finding worth the detail:**
   - `list_events(path, ...)` for the `ERR`/`MODE`/`EV`/`MSG` timeline around the incident.
   - `query_timeseries(path, message_type, fields, start_s, end_s)` for the raw numeric
     series behind a finding (e.g. `VIBE`, `ATT` DesRoll vs Roll, `BAT` Volt).
   - `get_params(path, name_glob)` / `load_param_file(path)` to check the configuration the
     vehicle actually flew.
4. **If asked about tuning,** call `recommend_tuning(path, area)` and present it as advice
   only — it is never applied to a vehicle.

Then explain, in plain language, what happened and why — citing the doc links the report
gave you and the concrete numbers (timestamps, values, thresholds) from the tools. Do not
assert anything the tools did not report. If the log is truncated, or a message a check
needs wasn't logged, say so instead of guessing.
