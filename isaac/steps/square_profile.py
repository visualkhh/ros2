"""02 센서 싱크용 정사각형 주행 프로파일 (sim/real 공용).

2초 정지 → 왼쪽 사각형(반시계) → 0.5초 정지 → 오른쪽 사각형(시계) → 2초 정지.
각 변: `side` m 전진 → 0.25초 정지 → 제자리 90도 회전 → 0.25초 정지.

- 왼쪽 사각형: `w = +turn` (반시계)
- 오른쪽 사각형: `w = -turn` (시계)
- 같은 시간표를 sim(`02_sensor_sync.py`)과 real(`02_sensor_sync_real.py`)이 함께 쓴다.
"""
from __future__ import annotations

import math

START_HOLD = 2.0
END_HOLD = 2.0
MOVE_SETTLE = 0.25
TURN_SETTLE = 0.25
BETWEEN = 0.5
DEFAULT_SIDE = 0.5


def make_profile(speed: float = 0.15, turn: float = 1.0,
                 side: float = DEFAULT_SIDE):
    """[(t0, t1, v, w, label)] — label은 stop/corner/between/move_left/turn_left/..."""
    speed = max(1e-6, float(speed))
    turn = max(1e-6, float(turn))
    side = max(0.0, float(side))
    move_t = side / speed
    turn_t = (math.pi / 2.0) / turn

    segs = [(0.0, START_HOLD, 0.0, 0.0, "stop")]
    t = START_HOLD
    for name, sign in (("left", 1.0), ("right", -1.0)):
        for _ in range(4):
            segs.append((t, t + move_t, speed, 0.0, f"move_{name}"))
            t += move_t
            segs.append((t, t + MOVE_SETTLE, 0.0, 0.0, "corner"))
            t += MOVE_SETTLE
            segs.append((t, t + turn_t, 0.0, sign * turn, f"turn_{name}"))
            t += turn_t
            segs.append((t, t + TURN_SETTLE, 0.0, 0.0, "corner"))
            t += TURN_SETTLE
        if name == "left":
            segs.append((t, t + BETWEEN, 0.0, 0.0, "between"))
            t += BETWEEN
    segs.append((t, t + END_HOLD, 0.0, 0.0, "stop"))
    return segs


def total(segs) -> float:
    return segs[-1][1]


def cmd_at(t: float, segs):
    """t[s] → (v, w). segs는 make_profile() 결과."""
    for _t0, t1, v, w, _label in segs:
        if t < t1:
            return v, w
    return 0.0, 0.0
