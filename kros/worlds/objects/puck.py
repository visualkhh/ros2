from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING

from kros.msgs.geometry_msgs import Pose2D
from kros.worlds.clock import Clock
from kros.worlds.objects.world_object import WorldObject

if TYPE_CHECKING:  # 월드가 objects 패키지를 import하므로 실행 때는 import하지 않는다 (순환 방지)
    from kros.worlds.world import World


class PuckShape(Enum):
    """퍽의 위에서 본 모양. 값은 변의 수 (원은 많은 변으로 근사)."""

    CIRCLE = 32
    TRIANGLE = 3
    SQUARE = 4
    HEXAGON = 6


class Puck(WorldObject):
    """바닥에 놓인 납작한 퍽. 모양과 색을 가진다. 스스로는 움직이지 않고, 부딪히면 월드가 밀어낸다."""

    def __init__(self, shape: PuckShape, color: tuple[float, float, float], radius: float = 0.06, height: float = 0.04) -> None:
        self.shape = shape
        self.color = color  # (r, g, b) 0~1
        self.radius = radius  # m, 중심에서 꼭짓점(원이면 가장자리)까지. 충돌은 이 원으로 근사한다
        self.mass = 0.16  # kg, 하키 퍽 정도
        self.friction = 0.3  # 퍽 재질(PLA)의 마찰 계수. 실제 마찰은 월드 바닥과 합쳐진다
        self.movable = True  # 부딪히면 밀리고, 마찰로 멈출 때까지 미끄러진다
        self.height = height  # m, 라이다(0.132m)보다 낮아서 라이다에는 보이지 않는다
        self.pose = Pose2D(0.0, 0.0, 0.0)

    def tick(self, clock: Clock, world: World) -> None:
        pass
