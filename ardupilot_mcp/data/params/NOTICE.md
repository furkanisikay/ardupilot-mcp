# Parameter metadata

The `*.json.gz` files in this directory hold facts taken from ArduPilot's versioned
parameter definitions (`apm.pdef.xml`, https://autotest.ardupilot.org/Parameters/versioned/),
which ArduPilot generates from its GPLv3 source code: parameter names, documented ranges,
units, value codes, bitmask bit positions and the ReadOnly / RebootRequired flags. No
description, display name or value label is copied.

`scripts/gen_param_meta.py` in the ardupilot-mcp repository regenerates them.
ArduPilot: https://github.com/ArduPilot/ardupilot (GPL-3.0).
