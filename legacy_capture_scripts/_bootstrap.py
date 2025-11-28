"""
Ensures the repository root is importable when running legacy scripts directly.
This allows absolute imports such as `realsense_planner` to resolve without
requiring callers to modify PYTHONPATH manually.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
