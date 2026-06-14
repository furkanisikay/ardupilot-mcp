"""Pytest bootstrap: ensure the repo root is importable (flat layout).

Lets ``import ardupilot_mcp`` and ``from tests.synth_bin import ...`` work when
running ``pytest`` from anywhere in the repo.
"""

import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
