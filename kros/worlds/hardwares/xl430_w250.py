"""ROBOTIS Dynamixel XL430-W250 모터 한 개 (TurtleBot3 바퀴, 매니퓰레이터 관절 등)."""
from kros.worlds.clock import Clock
from kros.worlds.hardwares.motor import Motor
from kros.worlds.world import World


class SimulatedXl430W250(Motor):
    """속도 모드 시뮬레이션: 목표 속도(max_velocity가 있으면 그 안으로 제한)로 돌고 position을 누적한다. 꺼지면 멈춘다.
    관성/가속은 아직 없다 (목표 속도로 바로 돈다)."""

    model = "XL430-W250"

    def __init__(self, max_velocity: float | None = None, period: float | None = None) -> None:
        super().__init__(period)
        self.max_velocity = max_velocity  # rad/s, None이면 제한 없음

    def apply(self, command: float, clock: Clock, world: World) -> None:
        limit = self.max_velocity
        self.velocity = command if limit is None else max(-limit, min(limit, command))
        self.position += self.velocity * clock.dt

    def tick(self, clock: Clock, world: World) -> None:
        super().tick(clock, world)
        if not self.is_on:
            self.velocity = 0.0  # 전원이 꺼지면 돌지 않는다
