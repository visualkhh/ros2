"""nav2_core::GlobalPlanner 흉내: planner_server에 끼우는 경로 계획 알고리즘의 공통 모양과 도우미."""
import math
from collections.abc import Callable

from kros.msgs.nav2_msgs import ComputePathToPoseResult as R
from kros.nodes.nav2.costmap_2d import INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION, Costmap
from kros.nodes.node import Node

Cell = tuple[int, int]
NEIGHBORS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))


class PlannerError(Exception):
    """계획 실패. code는 ComputePathToPoseResult의 오류 코드."""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code


Pose2 = tuple[float, float, float]  # (x, y, yaw)


class GlobalPlanner:
    """플러그인: configure(node, name)에서 "<name>.<파라미터>"를 선언하고, create_plan이 world 좌표 점들을 돌려준다 (실패면 PlannerError).
    start/goal은 (x, y, yaw): 격자 플래너는 x, y만 보고, Hybrid-A*처럼 방향을 아는 플래너는 yaw도 본다."""

    def configure(self, node: Node, name: str) -> None:
        self.node, self.name = node, name

    def create_plan(self, costmap: Costmap, start: Pose2, goal: Pose2) -> list[tuple[float, float]]:
        raise NotImplementedError


def passable(cost: int, allow_unknown: bool) -> bool:
    """로봇 중심이 이 칸에 와도 되나: 몸이 닿는 칸(253 이상)은 안 되고, 모름은 allow_unknown이면 된다."""
    return cost < INSCRIBED_INFLATED_OBSTACLE or (cost == NO_INFORMATION and allow_unknown)


def endpoints(costmap: Costmap, start: Pose2 | tuple[float, float], goal: Pose2 | tuple[float, float], tolerance: float, allow_unknown: bool) -> tuple[Cell, Cell]:
    """시작/목표 칸. 목표가 못 가는 칸이면 tolerance(m) 안에서 가장 가까운 갈 수 있는 칸으로 (nav2 navfn의 tolerance)."""
    s, g = costmap.world_to_map(start[0], start[1]), costmap.world_to_map(goal[0], goal[1])
    if s is None:
        raise PlannerError(R.START_OUTSIDE_MAP, f"시작 {start}이 지도 밖이다")
    if g is None:
        raise PlannerError(R.GOAL_OUTSIDE_MAP, f"목표 {goal}가 지도 밖이다")
    if costmap.cost(*s) == LETHAL_OBSTACLE:
        raise PlannerError(R.START_OCCUPIED, f"시작 {start}이 벽 안이다")
    if not passable(costmap.cost(*g), allow_unknown):
        reach = int(tolerance / costmap.resolution)
        candidates = [
            (math.hypot(di, dj), (g[0] + di, g[1] + dj))
            for di in range(-reach, reach + 1)
            for dj in range(-reach, reach + 1)
            if 0 <= g[0] + di < costmap.width and 0 <= g[1] + dj < costmap.height and passable(costmap.cost(g[0] + di, g[1] + dj), allow_unknown) and math.hypot(di, dj) <= reach
        ]
        if not candidates:
            raise PlannerError(R.GOAL_OCCUPIED, f"목표 {goal}가 벽 안이고 {tolerance}m 안에 갈 곳이 없다")
        g = min(candidates)[1]
    return s, g


ESCAPE_RADIUS = 0.3  # m: 로봇이 벽에 붙어 있으면(시작 칸이 253) 이 안에서는 벽(254)만 아니면 지나가 빠져나온다 (nav2 navfn은 시작 칸을 비운다)


def make_passable(costmap: Costmap, start: Cell, allow_unknown: bool) -> Callable[[int, int], bool]:
    """칸 (i, j)에 로봇 중심이 가도 되나 (지도 안인지도 본다)."""
    reach2 = (ESCAPE_RADIUS / costmap.resolution) ** 2
    stuck = not passable(costmap.cost(*start), allow_unknown)

    def check(i: int, j: int) -> bool:
        if not (0 <= i < costmap.width and 0 <= j < costmap.height):
            return False
        cost = costmap.cost(i, j)
        if passable(cost, allow_unknown):
            return True
        return stuck and cost != LETHAL_OBSTACLE and (i - start[0]) ** 2 + (j - start[1]) ** 2 <= reach2

    return check


def step_cost(costmap: Costmap, cell: Cell, neutral_cost: float, cost_factor: float) -> float:
    """칸 하나를 지나는 값 (nav2 navfn처럼 기본값 neutral_cost + cost_factor * 비용): 벽 가까운 길을 꺼린다."""
    cost = costmap.cost(*cell)
    return neutral_cost + cost_factor * (INSCRIBED_INFLATED_OBSTACLE - 1 if cost == NO_INFORMATION else cost)


def to_world(costmap: Costmap, cells: list[Cell]) -> list[tuple[float, float]]:
    return [costmap.map_to_world(*cell) for cell in cells]


def walk_back(parents: dict[Cell, Cell], end: Cell) -> list[Cell]:
    cells = [end]
    while cells[-1] in parents:
        cells.append(parents[cells[-1]])
    return cells[::-1]

