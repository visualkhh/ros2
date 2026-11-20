from __future__ import annotations

import math

from kros.msgs.geometry_msgs import Pose2D
from kros.worlds.clock import Clock
from kros.worlds.collision import Box, Shape
from kros.worlds.world import World
from kros.worlds.world_object import WorldObject


class Wall(WorldObject):
    """벽 한 조각: 월드 축에 평행한 상자. 절대 움직이지 않는다(static, 질량 무한)."""

    static = True
    movable = False
    mass = math.inf

    def __init__(self, x: float, y: float, size_x: float, size_y: float, height: float) -> None:
        self.size_x, self.size_y = size_x, size_y  # m, 가로/세로 전체 크기
        self.height = height
        self.pose = Pose2D(x, y, 0.0)
        self._collider = Box(size_x / 2, size_y / 2)

    @property
    def collider(self) -> Shape:
        return self._collider

    def tick(self, clock: Clock, world: World) -> None:
        pass
