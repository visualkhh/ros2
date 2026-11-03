"""BaseCommand — 모든 명령 경로의 바탕. 표준 라이브러리만."""
from __future__ import annotations

import math


class BaseCommand:
    """시각 t [s] → (v [m/s], w [rad/s]). 생성자는 speed · turn을 받는다 (run-world.sh · run-calibration.sh --speed --turn)."""

    total = math.inf   # 길이 [s] — 끝 없는 명령(키보드 등)은 inf

    def __init__(self, speed: float = 0.15, turn: float = 1.0):
        self.speed, self.turn = speed, turn

    def reset(self) -> None:
        """처음부터 다시."""

    def __call__(self, t: float) -> tuple[float, float]:
        return 0.0, 0.0

    def done(self, t: float) -> bool:
        return t >= self.total

    def path(self):
        """목표 경로 [(x, y)] — 시작 자세 기준 (x = 앞, y = 왼쪽). 화면 노란 점. 없으면 None."""
        return None
