"""ROBOTIS Dynamixel XL430-W250 모터 한 개 (TurtleBot3 바퀴, 매니퓰레이터 관절 등)."""
import math
import random

from kros.hardwares.noise import Noise
from kros.worlds.clock import Clock
from kros.hardwares.motors.motor import Motor


class SimulatedXl430W250(Motor):
    """속도 모드 시뮬레이션: 목표 속도(max_velocity가 있으면 그 안으로 제한)로 돌고 position을 누적한다. 꺼지면 멈춘다.
    관성/가속은 아직 없다 (목표 속도로 바로 돈다).

    오차(기본은 없음): ticks_per_rev를 주면 엔코더 값(position)이 그 해상도로 끊겨 나온다 (XL430은 4096). slip은 바퀴가 바닥에서 미끄러지거나
    반지름이 다른 만큼 몸체가 엔코더가 말하는 것과 다르게 움직이게 한다 (contact_velocity에 적용; 도는 속도 velocity와 엔코더는 그대로라 오도메트리가 어긋난다). seed가 같으면 같은 오차가 나온다."""

    model = "XL430-W250"

    def __init__(
        self, max_velocity: float | None = None, period: float | None = None, ticks_per_rev: int | None = None, slip: Noise = Noise(), seed: int | None = None
    ) -> None:
        super().__init__(period)
        self.ticks_per_rev, self.slip = ticks_per_rev, slip
        self._rng = random.Random(seed)
        self._angle = 0.0  # 엔코더가 끊기 전의 진짜 회전각
        self.max_velocity = max_velocity  # rad/s, None이면 제한 없음

    def apply(self, command: float, clock: Clock) -> None:
        limit = self.max_velocity
        self.velocity = command if limit is None else max(-limit, min(limit, command))
        self._angle += self.velocity * clock.dt
        step = None if self.ticks_per_rev is None else 2 * math.pi / self.ticks_per_rev
        self.position = self._angle if step is None else round(self._angle / step) * step

    def contact_velocity(self) -> float:
        return self.slip.apply(self.velocity, self._rng) if self.velocity else 0.0  # 서 있는 바퀴는 미끄러지지 않는다

    def tick(self, clock: Clock) -> None:
        super().tick(clock)
        if not self.is_on:
            self.velocity = 0.0  # 전원이 꺼지면 돌지 않는다
