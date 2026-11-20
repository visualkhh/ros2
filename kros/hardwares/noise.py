"""진짜 센서/바퀴의 오차를 시뮬레이션 하드웨어 안에서 흉내 내는 수치. 진짜 하드웨어 구현은 쓰지 않는다 (원래 오차가 섞여 들어온다)."""
import random
from dataclasses import dataclass


@dataclass(frozen=True)
class Noise:
    """값 하나에 붙는 오차: 측정 = 값 * (1 + scale) + bias + 가우시안(0, sigma).
    sigma는 매번 달라지는 흔들림(백색 잡음), bias는 늘 같은 쪽으로 치우친 양, scale은 비율 오차(바퀴 반지름 오차 등). 모두 0이면 오차 없음."""

    sigma: float = 0.0
    bias: float = 0.0
    scale: float = 0.0

    def apply(self, value: float, rng: random.Random) -> float:
        return value * (1.0 + self.scale) + self.bias + rng.gauss(0.0, self.sigma)
