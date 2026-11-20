from __future__ import annotations

import math

from kros.msgs.geometry_msgs import Pose2D
from kros.worlds.clock import Clock
from kros.worlds.collision import Box, Compound, Shape
from kros.worlds.world import World
from kros.worlds.objects.world_object import WorldObject

# isaac rl_maze 트랙(track_source/grid.npz)의 벽 중심선 (x1, y1, x2, y2) m. 바깥 4.48 x 6.37 m 방 + 안쪽 벽
WALLS: tuple[tuple[float, float, float, float], ...] = (
    (-2.24, -3.20, 2.24, -3.20), (2.24, -3.20, 2.24, 3.17), (2.24, 3.17, -2.24, 3.17), (-2.24, 3.17, -2.24, -3.20),
    (-1.19, -1.95, -0.41, -1.95), (0.35, -1.94, 1.89, -1.94), (-1.19, -1.20, -0.41, -1.20), (0.41, -1.16, 1.19, -1.16),
    (1.15, 0.33, 1.93, 0.33), (-1.95, 1.09, -1.17, 1.09), (1.11, 1.11, 1.89, 1.11), (-0.39, 1.81, 0.39, 1.81),
    (-1.18, 0.33, -1.18, 1.11), (-1.17, -2.71, -1.17, -1.93), (-1.17, -1.21, -1.17, -0.43), (-1.11, 1.69, -1.11, 2.49),
    (-0.43, -1.97, -0.43, -1.17), (0.37, 1.79, 0.37, 2.57), (1.15, 1.81, 1.15, 2.59), (1.17, -1.19, 1.17, 0.35),
)
WALL_THICKNESS = 0.05  # m
WALL_HEIGHT = 0.25  # m
START = (0.0, 0.0, 0.0)  # x, y, theta. 벽까지 1.18 m 떨어진 빈 곳
CENTER = (0.0, 0.0)
SIZE = (4.48, 6.37)


class Track(WorldObject):
    """isaac rl_maze 트랙 전체가 하나의 WorldObject: 바깥 벽과 안쪽 벽을 한 덩어리(Compound)로 가진다. 절대 움직이지 않는다(static, 질량 무한).

    벽 한 줄(WALLS의 선분)이 상자 하나가 되고, 상자 위치는 트랙의 pose(보통 원점) 기준이다. 축에 평행한 벽만 있다.
    """

    static = True
    movable = False
    mass = math.inf
    height = WALL_HEIGHT

    def __init__(self) -> None:
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.boxes: tuple[tuple[float, float, Box], ...] = tuple(  # 중심선 양끝 + 두께로 만든 상자들 (dx, dy, 상자)
            ((x1 + x2) / 2, (y1 + y2) / 2, Box((abs(x2 - x1) + WALL_THICKNESS) / 2, (abs(y2 - y1) + WALL_THICKNESS) / 2)) for x1, y1, x2, y2 in WALLS
        )
        self._collider = Compound(self.boxes)

    @property
    def collider(self) -> Shape:
        return self._collider

    def tick(self, clock: Clock, world: World) -> None:
        pass
