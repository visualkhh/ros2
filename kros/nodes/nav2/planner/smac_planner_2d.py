"""nav2_smac_planner::SmacPlanner2D 흉내: 비용을 아는 8방향 A* + 경로 펴기(smoother).
칸 값은 1 + cost_penalty * 비용/252라 벽에서 먼 길을 고르고, 찾은 칸 경로는 smoother가 벽에 닿지 않는 한도에서 매끄럽게 편다."""
import heapq
import math

from kros.msgs.nav2_msgs import ComputePathToPoseResult
from kros.nodes.nav2.costmap_2d import INSCRIBED_INFLATED_OBSTACLE, NO_INFORMATION, Costmap
from kros.nodes.node import Node

from .global_planner import NEIGHBORS, Pose2, Cell, GlobalPlanner, PlannerError, endpoints, make_passable, to_world, walk_back


class SmacPlanner2D(GlobalPlanner):
    """파라미터 ("<이름>." 아래, nav2와 같은 이름): tolerance, allow_unknown, cost_travel_multiplier (벽 가까운 칸을 얼마나 꺼리나),
    max_iterations (펼칠 칸 수 한계), smooth_path (smoother를 쓸지), smoother.w_smooth / smoother.w_data (매끄러움과 원래 경로의 무게), smoother.max_iterations."""

    def configure(self, node: Node, name: str) -> None:
        super().configure(node, name)
        non_negative = lambda value: value >= 0  # noqa: E731
        positive = lambda value: value > 0  # noqa: E731
        self.tolerance = node.declare_parameter(f"{name}.tolerance", 0.125, validator=non_negative)
        self.allow_unknown = node.declare_parameter(f"{name}.allow_unknown", True)
        self.cost_travel_multiplier = node.declare_parameter(f"{name}.cost_travel_multiplier", 2.0, validator=non_negative)
        self.max_iterations = node.declare_parameter(f"{name}.max_iterations", 1000000, validator=positive)
        self.smooth_path = node.declare_parameter(f"{name}.smooth_path", True)
        self.w_smooth = node.declare_parameter(f"{name}.smoother.w_smooth", 0.3, validator=non_negative)
        self.w_data = node.declare_parameter(f"{name}.smoother.w_data", 0.2, validator=non_negative)
        self.smoother_iterations = node.declare_parameter(f"{name}.smoother.max_iterations", 1000, validator=positive)
        self.expanded = 0

    def create_plan(self, costmap: Costmap, start: Pose2, goal: Pose2) -> list[tuple[float, float]]:
        s, g = endpoints(costmap, start, goal, self.tolerance.value, self.allow_unknown.value)
        ok = make_passable(costmap, s, self.allow_unknown.value)
        penalty = self.cost_travel_multiplier.value

        def travel(cell: Cell) -> float:
            cost = costmap.cost(*cell)
            return 1.0 + penalty * (INSCRIBED_INFLATED_OBSTACLE - 1 if cost == NO_INFORMATION else cost) / 252

        best: dict[Cell, float] = {s: 0.0}
        parents: dict[Cell, Cell] = {}
        frontier = [(math.hypot(s[0] - g[0], s[1] - g[1]), 0.0, s)]
        self.expanded = 0
        while frontier and self.expanded < self.max_iterations.value:
            _, cost, cell = heapq.heappop(frontier)
            if cost > best.get(cell, math.inf):
                continue
            self.expanded += 1
            if cell == g:
                points = to_world(costmap, walk_back(parents, g))
                return self._smooth(costmap, points) if self.smooth_path.value else points
            for di, dj, length in NEIGHBORS:
                nxt = (cell[0] + di, cell[1] + dj)
                if not ok(*nxt) or (di and dj and not (ok(cell[0] + di, cell[1]) and ok(cell[0], cell[1] + dj))):
                    continue
                new = cost + length * travel(nxt)
                if new < best.get(nxt, math.inf):
                    best[nxt], parents[nxt] = new, cell
                    heapq.heappush(frontier, (new + math.hypot(nxt[0] - g[0], nxt[1] - g[1]), new, nxt))
        code = ComputePathToPoseResult.TIMEOUT if frontier else ComputePathToPoseResult.NO_VALID_PATH
        raise PlannerError(code, f"{start} -> {goal} 길이 없다 (펼친 칸 {self.expanded})")

    def _smooth(self, costmap: Costmap, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """nav2 smac의 Smoother처럼 경사 하강: 각 점을 이웃 둘의 가운데 쪽(w_smooth)과 원래 자리 쪽(w_data)으로 당긴다.
        옮긴 점이 몸이 닿는 칸이면 그 점은 옮기지 않는다. 처음과 끝은 그대로."""
        if len(points) < 3:
            return points
        original = list(points)
        smooth = list(points)
        w_smooth, w_data = self.w_smooth.value, self.w_data.value
        for _ in range(self.smoother_iterations.value):
            change = 0.0
            for k in range(1, len(smooth) - 1):
                x, y = smooth[k]
                nx = x + w_data * (original[k][0] - x) + w_smooth * (smooth[k - 1][0] + smooth[k + 1][0] - 2 * x)
                ny = y + w_data * (original[k][1] - y) + w_smooth * (smooth[k - 1][1] + smooth[k + 1][1] - 2 * y)
                if costmap.cost_at(nx, ny) < INSCRIBED_INFLATED_OBSTACLE:
                    change += abs(nx - x) + abs(ny - y)
                    smooth[k] = (nx, ny)
            if change < 1e-6:
                break
        return smooth
