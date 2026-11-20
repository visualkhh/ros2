"""nav2_smac_planner::SmacPlannerHybrid 흉내: Hybrid-A*. 상태가 칸 (x, y)와 방향(angle_quantization_bins칸)이고,
직진/왼쪽 원호/오른쪽 원호(최소 회전 반지름)로만 펼쳐서 로봇이 실제로 달릴 수 있는 경로가 나온다 (REEDS_SHEPP이면 후진도).
휴리스틱은 직선 거리와 "벽을 돌아가는 2D 거리"(목표에서 퍼뜨린 Dijkstra) 중 큰 것."""
import heapq
import math
from collections.abc import Callable

from kros.msgs.nav2_msgs import ComputePathToPoseResult
from kros.nodes.nav2.costmap_2d import INSCRIBED_INFLATED_OBSTACLE, NO_INFORMATION, Costmap
from kros.nodes.node import Node

from .global_planner import NEIGHBORS, Cell, GlobalPlanner, PlannerError, Pose2, endpoints, make_passable

State = tuple[int, int, int]  # (칸 x, 칸 y, 방향 칸)


class SmacPlannerHybrid(GlobalPlanner):
    """파라미터 ("<이름>." 아래, nav2와 같은 이름): motion_model_for_search ("DUBIN" 앞으로만, "REEDS_SHEPP" 후진도), minimum_turning_radius (m),
    angle_quantization_bins, cost_penalty (벽 가까움), non_straight_penalty (도는 동작), reverse_penalty (후진), tolerance, allow_unknown, max_iterations.
    shortcut: nav2의 해석적 확장(Dubins/Reeds-Shepp 곡선으로 목표 방향까지 정확히 들어가기)은 없다: 목표 칸에 닿으면 끝 (목표 방향은 controller가 도착해서 맞춘다)."""

    def configure(self, node: Node, name: str) -> None:
        super().configure(node, name)
        non_negative = lambda value: value >= 0  # noqa: E731
        positive = lambda value: value > 0  # noqa: E731
        self.motion_model = node.declare_parameter(f"{name}.motion_model_for_search", "DUBIN", validator=lambda value: value in ("DUBIN", "REEDS_SHEPP"))
        self.minimum_turning_radius = node.declare_parameter(f"{name}.minimum_turning_radius", 0.25, validator=positive)
        self.angle_bins = node.declare_parameter(f"{name}.angle_quantization_bins", 72, validator=positive)
        self.cost_penalty = node.declare_parameter(f"{name}.cost_penalty", 2.0, validator=non_negative)
        self.non_straight_penalty = node.declare_parameter(f"{name}.non_straight_penalty", 1.2, validator=lambda value: value >= 1)
        self.reverse_penalty = node.declare_parameter(f"{name}.reverse_penalty", 2.0, validator=lambda value: value >= 1)
        self.tolerance = node.declare_parameter(f"{name}.tolerance", 0.25, validator=non_negative)
        self.allow_unknown = node.declare_parameter(f"{name}.allow_unknown", True)
        self.max_iterations = node.declare_parameter(f"{name}.max_iterations", 200000, validator=positive)
        self.expanded = 0

    def create_plan(self, costmap: Costmap, start: Pose2, goal: Pose2) -> list[tuple[float, float]]:
        s, g = endpoints(costmap, start, goal, self.tolerance.value, self.allow_unknown.value)
        ok = make_passable(costmap, s, self.allow_unknown.value)
        res, bins = costmap.resolution, self.angle_bins.value
        step = math.sqrt(2) * res  # 한 동작으로 칸을 벗어날 만큼
        turn = step / self.minimum_turning_radius.value  # 원호 한 동작에 도는 각
        directions = (1, -1) if self.motion_model.value == "REEDS_SHEPP" else (1,)
        primitives = [(d, k * turn) for d in directions for k in (0, 1, -1)]  # (앞/뒤, 회전각)
        to_goal = _distance_map(costmap, g, ok)  # 벽을 돌아가는 2D 거리 (칸)
        cost_penalty = self.cost_penalty.value

        def heuristic(x: float, y: float) -> float:
            cell = costmap.world_to_map(x, y)
            return max(math.hypot(x - goal[0], y - goal[1]), to_goal.get(cell, math.inf) * res if cell else math.inf)

        def state(x: float, y: float, yaw: float) -> State | None:
            cell = costmap.world_to_map(x, y)
            return None if cell is None else (cell[0], cell[1], round(yaw / (2 * math.pi) * bins) % bins)

        start_pose = (start[0], start[1], start[2])
        first = state(*start_pose)
        assert first is not None  # endpoints가 지도 안임을 확인했다
        poses: dict[State, Pose2] = {first: start_pose}
        best: dict[State, float] = {first: 0.0}
        parents: dict[State, State] = {}
        frontier = [(heuristic(start[0], start[1]), 0.0, first)]
        self.expanded = 0
        while frontier and self.expanded < self.max_iterations.value:
            _, cost, current = heapq.heappop(frontier)
            if cost > best.get(current, math.inf):
                continue
            self.expanded += 1
            x, y, yaw = poses[current]
            if (current[0], current[1]) == g or math.hypot(x - goal[0], y - goal[1]) < res:
                return self._trace(parents, poses, current)
            for direction, dtheta in primitives:
                nx, ny, nyaw = _move(x, y, yaw, direction * step, direction * dtheta)
                nxt = state(nx, ny, nyaw)
                if nxt is None or not ok(nxt[0], nxt[1]):
                    continue
                c = costmap.cost(nxt[0], nxt[1])
                weight = 1.0 + cost_penalty * (INSCRIBED_INFLATED_OBSTACLE - 1 if c == NO_INFORMATION else c) / 252
                if dtheta:
                    weight *= self.non_straight_penalty.value
                if direction < 0:
                    weight *= self.reverse_penalty.value
                new = cost + step * weight
                if new < best.get(nxt, math.inf):
                    best[nxt], parents[nxt], poses[nxt] = new, current, (nx, ny, nyaw)
                    heapq.heappush(frontier, (new + heuristic(nx, ny), new, nxt))
        code = ComputePathToPoseResult.TIMEOUT if frontier else ComputePathToPoseResult.NO_VALID_PATH
        raise PlannerError(code, f"{start} -> {goal} 길이 없다 (펼친 상태 {self.expanded})")

    @staticmethod
    def _trace(parents: dict[State, State], poses: dict[State, Pose2], end: State) -> list[tuple[float, float]]:
        states = [end]
        while states[-1] in parents:
            states.append(parents[states[-1]])
        return [poses[st][:2] for st in reversed(states)]


def _move(x: float, y: float, yaw: float, length: float, dtheta: float) -> Pose2:
    """length만큼 직진하거나 (dtheta가 있으면) 원호를 따라간 자리."""
    if dtheta == 0.0:
        return x + length * math.cos(yaw), y + length * math.sin(yaw), yaw
    radius = length / dtheta
    nyaw = yaw + dtheta
    return x + radius * (math.sin(nyaw) - math.sin(yaw)), y - radius * (math.cos(nyaw) - math.cos(yaw)), math.atan2(math.sin(nyaw), math.cos(nyaw))


def _distance_map(costmap: Costmap, goal: Cell, passable: Callable[[int, int], bool]) -> dict[Cell, float]:
    """목표 칸에서 갈 수 있는 칸으로 퍼뜨린 8방향 거리(칸). 벽에 막힌 곳을 돌아가는 거리라 직선 거리보다 좋은 휴리스틱이다."""
    distance: dict[Cell, float] = {goal: 0.0}
    frontier = [(0.0, goal)]
    while frontier:
        d, cell = heapq.heappop(frontier)
        if d > distance[cell]:
            continue
        for di, dj, length in NEIGHBORS:
            nxt = (cell[0] + di, cell[1] + dj)
            if passable(*nxt) and d + length < distance.get(nxt, math.inf):
                distance[nxt] = d + length
                heapq.heappush(frontier, (d + length, nxt))
    return distance
