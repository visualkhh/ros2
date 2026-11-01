"""기본 excitation: 정사각형. 한 파일 한 클래스.

`CalibExcitation` 계약의 기본 구현이다. 경로를 바꾸려면 서브클래스에서
`_extra` / `make_waypoints` / `profile_path`를 오버라이딩한다.
"""
from __future__ import annotations

import math

from calibration.excitation.CalibExcitation import CalibExcitation


class DefaultCalibExcitation(CalibExcitation):
    """정사각형: 2초 정지 → 왼쪽 사각형 → 180° 턴 → 역주행 → 2초 정지."""

    @classmethod
    def make_profile(cls, speed: float = 0.15, turn: float = 1.0,
                     side: float | None = None, speeds=None):
        """[(t0, t1, v, w, label)] — label은 stop/corner/turnaround/move_left/...

        `speeds`를 주면 그 4개 속도를 그대로 쓴다. 없으면 `speed` 기준 비율
        `(1.0, 2/3, 4/3, 0.8)`을 쓴다.
        """
        if side is None:
            side = cls.DEFAULT_SIDE
        speed = max(1e-6, float(speed))
        turn = max(1e-6, float(turn))
        side = max(0.0, float(side))
        if speeds is None:
            speeds = tuple(min(cls.MAX_SIDE_SPEED, max(0.03, speed * r)) for r in cls.SIDE_SPEED_RATIOS)
        else:
            speeds = tuple(min(cls.MAX_SIDE_SPEED, max(0.03, float(v))) for v in speeds)
        if len(speeds) != 4:
            raise ValueError("speeds는 4개여야 함")
        turn_t = (math.pi / 2.0) / turn

        segs = [(0.0, cls.START_HOLD, 0.0, 0.0, "stop")]
        t = cls.START_HOLD

        segs, t = cls._squares(segs, t, speed, turn, side, speeds, turn_t)
        segs, t = cls._extra(segs, t, speed, turn, side, turn_t)

        segs.append((t, t + cls.END_HOLD, 0.0, 0.0, "stop"))
        return segs

    @classmethod
    def _squares(cls, segs, t, speed, turn, side, speeds, turn_t):
        """왼쪽 사각형 → 180° 턴 → 같은 사각형 역주행. (segs, t) 반환."""
        # 1) 왼쪽 사각형 (반시계) — 변마다 속도 다르게
        for v in speeds:
            move_t = side / v
            segs.append((t, t + move_t, v, 0.0, "move_left"))
            t += move_t
            segs.append((t, t + cls.MOVE_SETTLE, 0.0, 0.0, "corner"))
            t += cls.MOVE_SETTLE
            segs.append((t, t + turn_t, 0.0, +turn, "turn_left"))
            t += turn_t
            segs.append((t, t + cls.TURN_SETTLE, 0.0, 0.0, "corner"))
            t += cls.TURN_SETTLE

        # 2) 180도 뒤로 돌기
        segs.append((t, t + 2.0 * turn_t, 0.0, -turn, "turnaround"))
        t += 2.0 * turn_t
        segs.append((t, t + cls.TURN_SETTLE, 0.0, 0.0, "corner"))
        t += cls.TURN_SETTLE

        # 3) 같은 사각형을 반대 방향으로: +y부터 시계방향으로 되짚기
        segs.append((t, t + turn_t, 0.0, -turn, "align_right"))
        t += turn_t
        segs.append((t, t + cls.TURN_SETTLE, 0.0, 0.0, "corner"))
        t += cls.TURN_SETTLE
        for i, v in enumerate(reversed(speeds)):
            move_t = side / v
            segs.append((t, t + move_t, v, 0.0, "move_right"))
            t += move_t
            segs.append((t, t + cls.MOVE_SETTLE, 0.0, 0.0, "corner"))
            t += cls.MOVE_SETTLE
            if i < 3:
                segs.append((t, t + turn_t, 0.0, -turn, "turn_right"))
                t += turn_t
                segs.append((t, t + cls.TURN_SETTLE, 0.0, 0.0, "corner"))
                t += cls.TURN_SETTLE
        return segs, t

    DIAG_SETTLE = 0.5
    DIAG_LOW_WHEEL = (0.10, 0.15, 0.20, 0.25, 0.30, 0.40)  # [rad/s] 데드밴드 탐색
    DIAG_LOW_HOLD = 1.5
    DIAG_SINGLE_WHEEL = 3.0   # [rad/s] 한쪽 바퀴만
    DIAG_SINGLE_HOLD = 1.5
    DIAG_POWER_HOLD = 2.0

    @classmethod
    def _diagnostics(cls, segs, t):
        """진단 구간: 저속(데드밴드) · 단륜 좌/우 · 고부하 직진/회전(포화).

        사각형은 양쪽 동시(직진·후진·회전)를 이미 본다. 여기서는
        (1) 데드밴드, (2) 좌/우 모터를 따로, (3) 고부하·포화를 추가로 본다.
        """
        from urdf.robot_spec import MAX_ANG, MAX_LIN, WHEEL_RADIUS, WHEEL_SEP  # lazy

        def stop(dur=None):
            nonlocal t
            dur = cls.DIAG_SETTLE if dur is None else dur
            segs.append((t, t + dur, 0.0, 0.0, "diag_stop"))
            t += dur

        stop()
        # 1) 저속 계단 — 데드밴드(바퀴가 안 도는 최소 명령) 탐색
        for wwheel in cls.DIAG_LOW_WHEEL:
            v = WHEEL_RADIUS * wwheel
            segs.append((t, t + cls.DIAG_LOW_HOLD, v, 0.0, "diag_low"))
            t += cls.DIAG_LOW_HOLD
            stop()

        # 2) 단륜(한쪽 바퀴만) — 모터를 좌/우 따로 분리해서 봄
        wwheel = cls.DIAG_SINGLE_WHEEL
        v = WHEEL_RADIUS * wwheel / 2.0
        w = WHEEL_RADIUS * wwheel / WHEEL_SEP
        segs.append((t, t + cls.DIAG_SINGLE_HOLD, v, -w, "diag_single_left"))
        t += cls.DIAG_SINGLE_HOLD
        stop()
        segs.append((t, t + cls.DIAG_SINGLE_HOLD, v, +w, "diag_single_right"))
        t += cls.DIAG_SINGLE_HOLD
        stop()

        # 3) 고부하 — 양쪽 동시 최고 속도 + 회전(바퀴 8 rad/s 포화 유발)
        segs.append((t, t + cls.DIAG_POWER_HOLD, MAX_LIN, 0.0, "diag_power_straight"))
        t += cls.DIAG_POWER_HOLD
        stop()
        segs.append((t, t + cls.DIAG_POWER_HOLD, MAX_LIN, MAX_ANG, "diag_power_turn"))
        t += cls.DIAG_POWER_HOLD
        stop()
        return segs, t

    @classmethod
    def _extra(cls, segs, t, speed, turn, side, turn_t):
        """확장 지점. 기본은 사각형 → 180° 턴 → 역주행으로 끝낸다 (진단 구간은 `_diagnostics`)."""
        return segs, t

    @classmethod
    def make_waypoints(cls, side: float | None = None):
        """RL goal용 절대 waypoint (시작 자세 기준). 왼쪽 사각형 4변 → 같은 사각형 역주행 4변."""
        if side is None:
            side = cls.DEFAULT_SIDE
        s = float(side)
        return [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0),
                (0.0, s), (s, s), (s, 0.0), (0.0, 0.0)]

    @classmethod
    def profile_path(cls, side: float | None = None):
        """open-loop profile 전체 외곽선: 사각형 → 역주행."""
        if side is None:
            side = cls.DEFAULT_SIDE
        s = float(side)
        sq = [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
        rev = [(0.0, s), (s, s), (s, 0.0), (0.0, 0.0)]
        return sq + rev
