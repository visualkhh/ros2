from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from kros.msgs.geometry_msgs import Pose2D
from kros.worlds.clock import Clock
from kros.worlds.collision import Circle, Shape

if TYPE_CHECKING:
    from kros.worlds.world import World


class WorldObject(ABC):
    """월드 안에 놓이는 모든 것의 최상위. 위치(pose), 충돌 원(radius), 질량/마찰을 가지고, 월드가 주는 시계 신호(tick)에 맞춰 스스로 움직인다. kros와 무관하다."""

    pose: Pose2D
    radius: float = 0.0  # m, 충돌 판정용 원 반지름 (0이면 충돌하지 않는다)
    height: float = 0.0  # m, 바닥에서 윗면까지. 센서 높이보다 낮은 객체는 센서가 못 본다
    mass: float = 1.0  # kg
    friction: float = 0.3  # 이 객체 재질의 마찰 계수. 실제 마찰은 월드 바닥과 합쳐서 정해진다(World.friction_with_ground)
    static: bool = False  # True면 월드 지형(벽): 절대 움직이지 않고, 로봇/객체는 이걸 파고들지 못한다
    movable: bool = False  # True면 월드 물리가 움직인다(밀리고 미끄러짐). False면 스스로 움직인다(로봇): 부딪혀도 안 밀린다
    world: World | None = None  # 이 객체가 놓인 월드 (World.add가 정한다). 환경을 물어볼 때 쓴다
    velocity: tuple[float, float] = (0.0, 0.0)  # m/s, movable 객체의 미끄러지는 속도 (월드가 관리)

    @property
    def collider(self) -> Shape | None:
        """충돌/레이 도형. 기본은 radius의 원 (radius가 0이면 없음)."""
        return Circle(self.radius) if self.radius > 0 else None

    @abstractmethod
    def tick(self, clock: Clock) -> None:
        """월드가 시간을 진행시킬 때마다 시계 신호가 내려온다. 환경(벽, 다른 객체)은 self.world에게 묻는다."""
