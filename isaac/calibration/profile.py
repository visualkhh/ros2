"""측정 명령 경로 (open-loop, 20 Hz): 2 s 정지 → 왼쪽 사각형(반시계) → 180° 턴 → 같은 사각형 역주행 → 2 s 정지.

각 변: 전진 → 0.25 s 정지 → 제자리 90° 회전 → 0.25 s 정지. 변마다 속도를 바꿔 (기준 × 1.0 / 0.67 / 1.33 / 0.8) gain을 여러 속도에서 잰다.
segs = [(t0, t1, v, w, label)] — label: stop · corner · move_left · turn_left · turnaround · align_right · move_right · turn_right.
sim(sensor_sync_sim) · real(sensor_sync_real) · dummy가 같은 경로를 쓴다.
"""
from __future__ import annotations

import math

START_HOLD = END_HOLD = 2.0
MOVE_SETTLE = TURN_SETTLE = 0.25
SIDE_SPEED_RATIOS = (1.0, 2.0 / 3.0, 4.0 / 3.0, 0.8)
MAX_SIDE_SPEED = 0.24


def make_profile(speed: float = 0.15, turn: float = 1.0, side: float = 0.5) -> list:
    speed, turn, side = max(1e-6, float(speed)), max(1e-6, float(turn)), max(0.0, float(side))
    speeds = [min(MAX_SIDE_SPEED, max(0.03, speed * r)) for r in SIDE_SPEED_RATIOS]
    turn_t = (math.pi / 2.0) / turn
    segs, t = [], 0.0

    def add(dur, v, w, label):
        nonlocal t
        segs.append((t, t + dur, v, w, label))
        t += dur

    add(START_HOLD, 0.0, 0.0, "stop")
    for v in speeds:                      # 1) 왼쪽 사각형 (반시계)
        add(side / v, v, 0.0, "move_left")
        add(MOVE_SETTLE, 0.0, 0.0, "corner")
        add(turn_t, 0.0, +turn, "turn_left")
        add(TURN_SETTLE, 0.0, 0.0, "corner")
    add(2.0 * turn_t, 0.0, -turn, "turnaround")   # 2) 뒤로 돌기
    add(TURN_SETTLE, 0.0, 0.0, "corner")
    add(turn_t, 0.0, -turn, "align_right")        # 3) 같은 사각형을 시계방향으로 되짚기
    add(TURN_SETTLE, 0.0, 0.0, "corner")
    for i, v in enumerate(reversed(speeds)):
        add(side / v, v, 0.0, "move_right")
        add(MOVE_SETTLE, 0.0, 0.0, "corner")
        if i < 3:
            add(turn_t, 0.0, -turn, "turn_right")
            add(TURN_SETTLE, 0.0, 0.0, "corner")
    add(END_HOLD, 0.0, 0.0, "stop")
    return segs


def total(segs) -> float:
    return segs[-1][1]


def cmd_at(t: float, segs) -> tuple[float, float]:
    for _t0, t1, v, w, _label in segs:
        if t < t1:
            return v, w
    return 0.0, 0.0


def profile_path(side: float = 0.5) -> list:
    """목표 경로 외곽선 (시작 자세 기준): 사각형 → 역주행."""
    s = float(side)
    return [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s), (0.0, 0.0), (0.0, s), (s, s), (s, 0.0), (0.0, 0.0)]


def speeds_text(segs) -> str:
    return " / ".join(f"{s[2]:.2f}" for s in segs if s[4] == "move_left")


if __name__ == "__main__":   # 자체 확인
    segs = make_profile()
    assert segs[0][4] == "stop" and segs[-1][4] == "stop" and abs(total(segs) - 54.0) < 2.0, total(segs)
    assert cmd_at(2.1, segs) == (0.15, 0.0) and cmd_at(1e9, segs) == (0.0, 0.0)
    assert all(a[1] == b[0] for a, b in zip(segs, segs[1:]))
    print("ok", round(total(segs), 1))
