"""nav2_navfn_planner::NavfnPlanner 흉내: 격자 위 최단 경로. use_astar가 거짓이면 Dijkstra(모든 방향으로 고르게 퍼진다), 참이면 A*(목표 쪽으로 먼저 뻗는다)."""
import heapq
import math

from kros.msgs.nav2_msgs import ComputePathToPoseResult
from kros.nodes.nav2.costmap_2d import Costmap
from kros.nodes.node import Node

from .global_planner import NEIGHBORS, Pose2, Cell, GlobalPlanner, PlannerError, endpoints, make_passable, step_cost, to_world, walk_back

NEUTRAL_COST, COST_FACTOR = 50.0, 0.8  # nav2 navfn의 COST_NEUTRAL, COST_FACTOR


class NavfnPlanner(GlobalPlanner):
    """파라미터 ("<이름>." 아래): tolerance (목표가 벽이면 이 안에서 갈 곳을 찾는다, m), use_astar, allow_unknown (모르는 칸을 지나가도 되나).
    shortcut: navfn은 퍼텐셜 장에서 기울기를 따라 매끈한 경로를 뽑지만, 여기서는 8방향 칸 경로를 그대로 낸다 (controller가 매끈하게 따라간다)."""

    def configure(self, node: Node, name: str) -> None:
        super().configure(node, name)
        self.tolerance = node.declare_parameter(f"{name}.tolerance", 0.5, validator=lambda value: value >= 0)
        self.use_astar = node.declare_parameter(f"{name}.use_astar", False)
        self.allow_unknown = node.declare_parameter(f"{name}.allow_unknown", True)
        self.expanded = 0  # 지난 계획에서 펼친 칸 수 (Dijkstra와 A*의 차이를 본다)

    def create_plan(self, costmap: Costmap, start: Pose2, goal: Pose2) -> list[tuple[float, float]]:
        allow_unknown = self.allow_unknown.value
        s, g = endpoints(costmap, start, goal, self.tolerance.value, allow_unknown)
        ok = make_passable(costmap, s, allow_unknown)
        astar = self.use_astar.value
        def h(c: Cell) -> float:
            """A*의 휴리스틱: 남은 거리 x 가장 싼 칸 값 (실제보다 크지 않아 최단을 보장한다). Dijkstra는 0."""
            return math.hypot(c[0] - g[0], c[1] - g[1]) * NEUTRAL_COST if astar else 0.0

        best: dict[Cell, float] = {s: 0.0}
        parents: dict[Cell, Cell] = {}
        frontier = [(h(s), 0.0, s)]
        self.expanded = 0
        while frontier:
            _, cost, cell = heapq.heappop(frontier)
            if cost > best.get(cell, math.inf):
                continue  # 더 싼 길로 이미 펼쳤다
            self.expanded += 1
            if cell == g:
                return to_world(costmap, walk_back(parents, g))
            for di, dj, length in NEIGHBORS:
                nxt = (cell[0] + di, cell[1] + dj)
                if not ok(*nxt):
                    continue
                if di and dj and not (ok(cell[0] + di, cell[1]) and ok(cell[0], cell[1] + dj)):
                    continue  # 대각선으로 벽 모서리 사이를 빠져나가지 않는다
                new = cost + length * step_cost(costmap, nxt, NEUTRAL_COST, COST_FACTOR)
                if new < best.get(nxt, math.inf):
                    best[nxt], parents[nxt] = new, cell
                    heapq.heappush(frontier, (new + h(nxt), new, nxt))
        raise PlannerError(ComputePathToPoseResult.NO_VALID_PATH, f"{start} -> {goal} 길이 없다")
