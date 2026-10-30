"""wheel_drift 보정/검증용 정사각형 프로파일 (sim/real/RL 공용).

`calibration`의 02 센서 싱크가 이 모듈을 import해서 같은 명령을 sim/real에 보낸다.
학습 후 `real_square.py`는 여기의 waypoint를 그대로 RL goal 노드에 보낸다.

open-loop 명령 (`make_profile`)
  2초 정지 → 왼쪽 사각형(반시계) → 180도 턴 → 같은 사각형 역주행
  → 사각형 안쪽 삼각형 → 2초 정지.
  각 변: `side` m 전진 → 0.25초 정지 → 회전 → 0.25초 정지.
  사각형 변은 속도를 다르게 해서 속도별 gain/지연/τ를 본다.

goal waypoint (`make_waypoints`)
  같은 사각형 8 waypoint + 사각형 안쪽 삼각형 4 waypoint(A,B,C,A)를 절대 좌표로 보낸다.
  RL goal 노드는 `goal_mode=absolute`로 이 sequence를 받는다.
"""
from __future__ import annotations

import math

START_HOLD = 2.0
END_HOLD = 2.0
MOVE_SETTLE = 0.25
TURN_SETTLE = 0.25
DEFAULT_SIDE = 0.5
TRI_INSET = 0.3       # 사각형 안쪽 여백 비율
TRI_SIDE_RATIO = 0.4  # 사각형 한 변 대비 삼각형 한 변
SQRT3_2 = math.sqrt(3.0) / 2.0
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

    # 4) 사각형 안쪽 삼각형: 역주행 후 (-x) → +x로 돌아 안쪽 A로 진입
    entry = TRI_INSET * side
    tri_side = TRI_SIDE_RATIO * side
    tri_turn = (2.0 * math.pi / 3.0) / turn
    half_pi_turn = turn_t
    entry_move = entry / speed
    tri_move = tri_side / speed

    # 180° 회전해서 +x
    segs.append((t, t + 2.0 * turn_t, 0.0, -turn, "align_triangle"))
    t += 2.0 * turn_t
    segs.append((t, t + TURN_SETTLE, 0.0, 0.0, "corner"))
    t += TURN_SETTLE
    # (0,0) → (entry,0)
    segs.append((t, t + entry_move, speed, 0.0, "move_triangle_entry"))
    t += entry_move
    segs.append((t, t + MOVE_SETTLE, 0.0, 0.0, "corner"))
    t += MOVE_SETTLE
    # (entry,0) → A=(entry,entry)
    segs.append((t, t + half_pi_turn, 0.0, +turn, "turn_triangle_entry"))
    t += half_pi_turn
    segs.append((t, t + TURN_SETTLE, 0.0, 0.0, "corner"))
    t += TURN_SETTLE
    segs.append((t, t + entry_move, speed, 0.0, "move_triangle_entry"))
    t += entry_move
    segs.append((t, t + MOVE_SETTLE, 0.0, 0.0, "corner"))
    t += MOVE_SETTLE
    # A에서 +x를 보게 정렬
    segs.append((t, t + half_pi_turn, 0.0, -turn, "turn_triangle_entry"))
    t += half_pi_turn
    segs.append((t, t + TURN_SETTLE, 0.0, 0.0, "corner"))
    t += TURN_SETTLE
    # A → B → C → A
    for i in range(3):
        segs.append((t, t + tri_move, speed, 0.0, "move_triangle"))
        t += tri_move
        segs.append((t, t + MOVE_SETTLE, 0.0, 0.0, "corner"))
        t += MOVE_SETTLE
        if i < 2:
            segs.append((t, t + tri_turn, 0.0, +turn, "turn_triangle"))
            t += tri_turn
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


def make_waypoints(side: float = DEFAULT_SIDE):
    """RL goal용 절대 waypoint (시작 자세 기준).

    왼쪽 사각형 4변 → 같은 사각형 역주행 4변 → 사각형 안쪽 삼각형 3변.
    """
    s = float(side)
    a = TRI_INSET * s
    b = a + TRI_SIDE_RATIO * s
    c = a + 0.5 * TRI_SIDE_RATIO * s
    d = a + SQRT3_2 * TRI_SIDE_RATIO * s
    tri = [(a, a), (b, a), (c, d), (a, a)]
    return [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0),
            (0.0, s), (s, s), (s, 0.0), (0.0, 0.0)] + tri


def square_path(side: float = DEFAULT_SIDE):
    """이상적인 사각형 외곽선 (plot용)."""
    s = float(side)
    return [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]


def profile_path(side: float = DEFAULT_SIDE):
    """open-loop profile 전체 외곽선: 사각형 → 역주행 → 안쪽 삼각형."""
    s = float(side)
    a = TRI_INSET * s
    b = a + TRI_SIDE_RATIO * s
    c = a + 0.5 * TRI_SIDE_RATIO * s
    d = a + SQRT3_2 * TRI_SIDE_RATIO * s
    sq = [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
    rev = [(0.0, s), (s, s), (s, 0.0), (0.0, 0.0)]
    entry = [(a, 0.0), (a, a)]
    tri = [(a, a), (b, a), (c, d), (a, a)]
    return sq + rev + entry + tri
