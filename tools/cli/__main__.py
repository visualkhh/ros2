#!/usr/bin/env python3
"""CLI 진입점 — `python -m cli [명령]` (start.sh가 ROS 환경을 source한 뒤 부른다)."""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent          # tools/cli
sys.path.insert(0, str(HERE))

from operator_tool import main  # noqa: E402

if __name__ == '__main__':
    raise SystemExit(main())
