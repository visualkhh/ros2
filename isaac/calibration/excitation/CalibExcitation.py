"""excitation 인터페이스. 한 파일 한 클래스.

구현은 `DefaultCalibExcitation`이 들고, task별 변형은 서브클래스가
메서드를 오버라이딩한다. 직접 인스턴스화하지 않는다.
"""
from __future__ import annotations

from abc import ABC, abstractmethod


class CalibExcitation(ABC):
    """정사각형 excitation 계약. 전부 classmethod로 쓴다."""

    START_HOLD = 2.0
    END_HOLD = 2.0
    MOVE_SETTLE = 0.25
    TURN_SETTLE = 0.25
    DEFAULT_SIDE = 0.5
    SIDE_SPEED_RATIOS = (1.0, 2.0 / 3.0, 4.0 / 3.0, 0.8)
    MAX_SIDE_SPEED = 0.24

    @classmethod
    @abstractmethod
    def make_profile(cls, speed: float = 0.15, turn: float = 1.0,
                     side: float | None = None, speeds=None):
        """[(t0, t1, v, w, label)] — label은 stop/corner/turnaround/move_left/..."""
        raise NotImplementedError

    @staticmethod
    def total(segs) -> float:
        return segs[-1][1]

    @staticmethod
    def cmd_at(t: float, segs):
        """t[s] → (v, w). segs는 make_profile() 결과."""
        for _t0, t1, v, w, _label in segs:
            if t < t1:
                return v, w
        return 0.0, 0.0

    @classmethod
    @abstractmethod
    def make_waypoints(cls, side: float | None = None):
        """RL goal용 절대 waypoint (시작 자세 기준)."""
        raise NotImplementedError

    @classmethod
    def square_path(cls, side: float | None = None):
        """이상적인 사각형 외곽선 (plot용)."""
        if side is None:
            side = cls.DEFAULT_SIDE
        s = float(side)
        return [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]

    @classmethod
    @abstractmethod
    def profile_path(cls, side: float | None = None):
        """open-loop profile 전체 외곽선."""
        raise NotImplementedError
