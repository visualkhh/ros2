"""02 센서 싱크용 정사각형 주행 프로파일 (sim/real 공용).

2초 정지 → 왼쪽 사각형(반시계) → 180도 턴 → 같은 사각형을 오른쪽(시계)으로 역주행
→ 2초 정지. 각 변: `side` m 전진 → 0.25초 정지 → 제자리 90도 회전 → 0.25초 정지.

- 각 변의 전진 속도를 다르게 해서 속도별 gain/지연/τ를 볼 수 있게 한다.
  기본 `--speed 0.15` 기준 `(0.15, 0.10, 0.20, 0.12)` m/s.
- 첫 사각형: 반시계(`w=+turn`), 끝나면 시작점에서 원래 방향(+x)을 봄.
- 180도 턴(`w=-turn`) 후 오른쪽으로 90도 틀어 +y부터 역주행.
- 두 번째 사각형은 첫 사각형의 **같은 네 변을 반대 방향으로** 되짚는다.
- 같은 시간표를 sim(`02_sensor_sync_sim.py`)과 real(`02_sensor_sync_real.py`)이 함께 쓴다.
"""
from __future__ import annotations

import math

START_HOLD = 2.0
END_HOLD = 2.0
MOVE_SETTLE = 0.25
TURN_SETTLE = 0.25
DEFAULT_SIDE = 0.5
SIDE_SPEED_RATIOS = (1.0, 2.0 / 3.0, 4.0 / 3.0, 0.8)
MAX_SIDE_SPEED = 0.24


def make_profile(speed: float = 0.15, turn: float = 1.0,
                 side: float = DEFAULT_SIDE, speeds=None):
    """[(t0, t1, v, w, label)] — label은 stop/corner/turnaround/move_left/...

    `speeds`를 주면 그 4개 속도를 그대로 쓴다. 없으면 `speed` 기준 비율
    `(1.0, 2/3, 4/3, 0.8)`을 쓴다.
    """
    speed = max(1e-6, float(speed))
    turn = max(1e-6, float(turn))
    side = max(0.0, float(side))
    if speeds is None:
        speeds = tuple(min(MAX_SIDE_SPEED, max(0.03, speed * r)) for r in SIDE_SPEED_RATIOS)
    else:
        speeds = tuple(min(MAX_SIDE_SPEED, max(0.03, float(v))) for v in speeds)
    if len(speeds) != 4:
        raise ValueError("speeds는 4개여야 함")
    turn_t = (math.pi / 2.0) / turn

    segs = [(0.0, START_HOLD, 0.0, 0.0, "stop")]
    t = START_HOLD

    # 1) 왼쪽 사각형 (반시계) — 변마다 속도 다르게
    for v in speeds:
        move_t = side / v
        segs.append((t, t + move_t, v, 0.0, "move_left"))
        t += move_t
        segs.append((t, t + MOVE_SETTLE, 0.0, 0.0, "corner"))
        t += MOVE_SETTLE
        segs.append((t, t + turn_t, 0.0, +turn, "turn_left"))
        t += turn_t
        segs.append((t, t + TURN_SETTLE, 0.0, 0.0, "corner"))
        t += TURN_SETTLE

    # 2) 180도 뒤로 돌기
    segs.append((t, t + 2.0 * turn_t, 0.0, -turn, "turnaround"))
    t += 2.0 * turn_t
    segs.append((t, t + TURN_SETTLE, 0.0, 0.0, "corner"))
    t += TURN_SETTLE

    # 3) 같은 사각형을 반대 방향으로: +y부터 시계방향으로 되짚기
    segs.append((t, t + turn_t, 0.0, -turn, "align_right"))
    t += turn_t
    segs.append((t, t + TURN_SETTLE, 0.0, 0.0, "corner"))
    t += TURN_SETTLE
    for i, v in enumerate(reversed(speeds)):
        move_t = side / v
        segs.append((t, t + move_t, v, 0.0, "move_right"))
        t += move_t
        segs.append((t, t + MOVE_SETTLE, 0.0, 0.0, "corner"))
        t += MOVE_SETTLE
        if i < 3:
            segs.append((t, t + turn_t, 0.0, -turn, "turn_right"))
            t += turn_t
            segs.append((t, t + TURN_SETTLE, 0.0, 0.0, "corner"))
            t += TURN_SETTLE

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
