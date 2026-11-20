"""nav2_theta_star_planner::ThetaStarPlanner 흉내: Theta* (any-angle). A*처럼 펼치되, 이웃 칸이 할아버지 칸에서 벽에 막히지 않고 직선으로 보이면
할아버지에 바로 잇는다. 그래서 칸 모양의 지그재그가 아니라 모서리에서만 꺾는 직선 경로가 나온다."""
import heapq
import math
from collections.abc import Callable

from kros.msgs.nav2_msgs import ComputePathToPoseResult
from kros.nodes.nav2.costmap_2d import INSCRIBED_INFLATED_OBSTACLE, NO_INFORMATION, Costmap
from kros.nodes.node import Node

from .global_planner import NEIGHBORS, Pose2, Cell, GlobalPlanner, PlannerError, endpoints, make_passable, walk_back


class ThetaStarPlanner(GlobalPlanner):
    """파라미터 ("<이름>." 아래, nav2와 같다): w_euc_cost (거리의 무게), w_traversal_cost (벽 가까운 칸 비용의 무게), allow_unknown, tolerance."""

    def configure(self, node: Node, name: str) -> None:
        super().configure(node, name)
        non_negative = lambda value: value >= 0  # noqa: E731
        self.w_euc_cost = node.declare_parameter(f"{name}.w_euc_cost", 1.0, validator=non_negative)
        self.w_traversal_cost = node.declare_parameter(f"{name}.w_traversal_cost", 2.0, validator=non_negative)
        self.allow_unknown = node.declare_parameter(f"{name}.allow_unknown", True)
        self.tolerance = node.declare_parameter(f"{name}.tolerance", 0.5, validator=non_negative)
        self.expanded = 0

    def create_plan(self, costmap: Costmap, start: Pose2, goal: Pose2) -> list[tuple[float, float]]:
        s, g = endpoints(costmap, start, goal, self.tolerance.value, self.allow_unknown.value)
        ok = make_passable(costmap, s, self.allow_unknown.value)
        w_euc, w_trav = self.w_euc_cost.value, self.w_traversal_cost.value

        def edge(a: Cell, b: Cell) -> float | None:
            """a에서 b까지 직선의 값. 중간에 못 가는 칸이 있으면 None (시야가 막혔다)."""
            cells = _line(a, b)
            if not all(ok(*c) for c in cells):
                return None
            costs = [costmap.cost(*c) for c in cells]
            mean = sum(INSCRIBED_INFLATED_OBSTACLE - 1 if c == NO_INFORMATION else c for c in costs) / len(costs)
            return math.hypot(b[0] - a[0], b[1] - a[1]) * (w_euc + w_trav * mean / 252)

        h: Callable[[Cell], float] = lambda c: w_euc * math.hypot(c[0] - g[0], c[1] - g[1])  # noqa: E731
        best: dict[Cell, float] = {s: 0.0}
        parents: dict[Cell, Cell] = {}
        frontier = [(h(s), 0.0, s)]
        self.expanded = 0
        while frontier:
            _, cost, cell = heapq.heappop(frontier)
            if cost > best.get(cell, math.inf):
                continue
            self.expanded += 1
            if cell == g:
                return _densify(costmap, walk_back(parents, g))
            grandparent = parents.get(cell)
            for di, dj, _ in NEIGHBORS:
                nxt = (cell[0] + di, cell[1] + dj)
                if not ok(*nxt):
                    continue
                via, value = cell, None
                if grandparent is not None:  # Theta*: 할아버지에서 바로 보이면 그쪽으로 잇는다
                    through = edge(grandparent, nxt)
                    if through is not None:
                        via, value = grandparent, best[grandparent] + through
                if value is None:
                    step = edge(cell, nxt)
                    if step is None:
                        continue
                    value = cost + step
                if value < best.get(nxt, math.inf):
                    best[nxt], parents[nxt] = value, via
                    heapq.heappush(frontier, (value + h(nxt), value, nxt))
        raise PlannerError(ComputePathToPoseResult.NO_VALID_PATH, f"{start} -> {goal} 길이 없다")


def _line(a: Cell, b: Cell) -> list[Cell]:
    """a와 b를 포함한 브레젠험 직선의 칸들."""
    (x, y), (x1, y1) = a, b
    dx, dy = abs(x1 - x), -abs(y1 - y)
    sx, sy = (1 if x < x1 else -1), (1 if y < y1 else -1)
    error = dx + dy
    cells = [(x, y)]
    while (x, y) != (x1, y1):
        e2 = 2 * error
        if e2 >= dy:
            error += dy
            x += sx
        if e2 <= dx:
            error += dx
            y += sy
        cells.append((x, y))
    return cells


def _densify(costmap: Costmap, corners: list[Cell]) -> list[tuple[float, float]]:
    """꺾이는 점들 사이를 칸 간격으로 채운다 (nav2 theta star의 interpolation: controller는 촘촘한 경로를 따라간다)."""
    points = [costmap.map_to_world(*corners[0])]
    for a, b in zip(corners, corners[1:]):
        (ax, ay), (bx, by) = costmap.map_to_world(*a), costmap.map_to_world(*b)
        n = max(1, round(math.hypot(bx - ax, by - ay) / costmap.resolution))
        points.extend((ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(1, n + 1))
    return points
