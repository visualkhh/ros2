"""dwb_core::DWBLocalPlanner 흉내 (DWA 계열): 지금 속도에서 가속 한계 안에 갈 수 있는 (v, w)를 여러 개 뽑아 sim_time 동안 굴려 보고,
critic들의 점수(낮을수록 좋다)를 더해 가장 좋은 것을 고른다. 벽에 닿는 자취는 버린다."""
import math

from kros.msgs.nav2_msgs import FollowPathResult
from kros.nodes.nav2.costmap_2d import INSCRIBED_INFLATED_OBSTACLE, Costmap
from kros.nodes.node import Node

from .controller import Controller, ControllerError, Pose2, Velocity, blocked, closest_index, simulate, wrap

CRITICS = ("RotateToGoal", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist")
SCALES = {"RotateToGoal": 32.0, "BaseObstacle": 0.02, "GoalAlign": 24.0, "PathAlign": 32.0, "PathDist": 32.0, "GoalDist": 24.0}  # nav2 기본 설정의 scale


class DWBLocalPlanner(Controller):
    """파라미터 ("<이름>." 아래, nav2와 같은 이름): min_vel_x, max_vel_x, max_vel_theta, acc_lim_x, acc_lim_theta, decel_lim_x, decel_lim_theta,
    vx_samples, vtheta_samples, sim_time, prune_distance (경로를 이만큼만 보고 그 끝을 지역 목표로), xy_goal_tolerance (RotateToGoal이 켜지는 거리),
    "<critic>.scale" (critic 무게), "PathAlign.forward_point_distance"/"GoalAlign.forward_point_distance".
    critic (점수가 낮을수록 좋다): BaseObstacle(지나가는 칸 비용 합, 닿으면 버림), PathDist(자취 끝과 경로 사이 거리), GoalDist(자취 끝과 지역 목표 거리),
    PathAlign/GoalAlign(자취 끝에서 앞으로 조금 나간 점의 거리: 방향이 맞는지), RotateToGoal(목표 근처에서는 서서 방향만 맞춘다).
    거리는 nav2처럼 칸 수로 잰다. 기본 샘플 수는 nav2(20x20)보다 적다: 순수 파이썬으로 20Hz에 돌게.
    """

    def configure(self, node: Node, name: str) -> None:
        super().configure(node, name)
        positive = lambda value: value > 0  # noqa: E731
        non_negative = lambda value: value >= 0  # noqa: E731
        self.min_vel_x = node.declare_parameter(f"{name}.min_vel_x", 0.0, validator=non_negative)
        self.max_vel_x = node.declare_parameter(f"{name}.max_vel_x", 0.22, validator=positive)
        self.max_vel_theta = node.declare_parameter(f"{name}.max_vel_theta", 1.0, validator=positive)
        self.acc_lim_x = node.declare_parameter(f"{name}.acc_lim_x", 2.5, validator=positive)
        self.acc_lim_theta = node.declare_parameter(f"{name}.acc_lim_theta", 3.2, validator=positive)
        self.decel_lim_x = node.declare_parameter(f"{name}.decel_lim_x", -2.5, validator=lambda value: value < 0)
        self.decel_lim_theta = node.declare_parameter(f"{name}.decel_lim_theta", -3.2, validator=lambda value: value < 0)
        self.vx_samples = node.declare_parameter(f"{name}.vx_samples", 6, validator=positive)
        self.vtheta_samples = node.declare_parameter(f"{name}.vtheta_samples", 15, validator=positive)
        self.sim_time = node.declare_parameter(f"{name}.sim_time", 1.7, validator=positive)
        self.prune_distance = node.declare_parameter(f"{name}.prune_distance", 1.0, validator=positive)
        self.xy_goal_tolerance = node.declare_parameter(f"{name}.xy_goal_tolerance", 0.2, validator=positive)
        self.scales = {critic: node.declare_parameter(f"{name}.{critic}.scale", SCALES[critic], validator=non_negative) for critic in CRITICS}
        self.path_align_forward = node.declare_parameter(f"{name}.PathAlign.forward_point_distance", 0.325, validator=non_negative)
        self.goal_align_forward = node.declare_parameter(f"{name}.GoalAlign.forward_point_distance", 0.325, validator=non_negative)
        self.best: list[Pose2] = []  # 지난 주기에 고른 자취 (화면/시험용)

    def compute_velocity_commands(self, pose: Pose2, velocity: Velocity, path: list[Pose2], costmap: Costmap, dt: float) -> Velocity:
        goal = path[-1]
        if math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < self.xy_goal_tolerance.value:  # RotateToGoal: 다 왔으면 서서 목표 방향으로만
            angle = wrap(goal[2] - pose[2])
            w = math.copysign(min(self.max_vel_theta.value, abs(velocity[1]) + self.acc_lim_theta.value * dt, abs(angle) / max(dt, 1e-6)), angle)
            return 0.0, w
        local = self._local_plan(pose, path)
        local_goal = local[-1]
        near_end = math.hypot(goal[0] - pose[0], goal[1] - pose[1])  # 경로 끝까지 남은 직선 거리
        res = costmap.resolution
        v0, w0 = velocity
        vs = _samples(max(self.min_vel_x.value, v0 + self.decel_lim_x.value * dt), min(self.max_vel_x.value, v0 + self.acc_lim_x.value * dt), self.vx_samples.value)
        limit = self.max_vel_theta.value
        ws = _samples(max(-limit, w0 + self.decel_lim_theta.value * dt), min(limit, w0 + self.acc_lim_theta.value * dt), self.vtheta_samples.value)
        best: tuple[float, Velocity, list[Pose2]] | None = None
        step = 0.1
        for v in vs:
            for w in ws:
                trajectory = simulate(pose, v, w, self.sim_time.value, step)
                obstacle = 0.0
                for x, y, _ in trajectory:
                    if math.hypot(x - pose[0], y - pose[1]) > 0.1 and blocked(costmap, x, y):
                        break  # 벽에 닿는다: 버린다
                    obstacle += min(costmap.cost_at(x, y), INSCRIBED_INFLATED_OBSTACLE - 1)
                else:
                    end = trajectory[-1]
                    score = self.scales["BaseObstacle"].value * obstacle
                    score += self.scales["PathDist"].value * _distance_to_path(end, local) / res
                    score += self.scales["GoalDist"].value * math.hypot(end[0] - local_goal[0], end[1] - local_goal[1]) / res
                    if near_end > self.path_align_forward.value:  # nav2처럼 끝이 앞으로 내다보는 거리 안이면 정렬 critic은 끈다 (넘어선 점이 벌점을 받아 멈춘다)
                        path_ahead = _ahead(end, self.path_align_forward.value)
                        score += self.scales["PathAlign"].value * _distance_to_path(path_ahead, local) / res
                    if near_end > self.goal_align_forward.value:
                        goal_ahead = _ahead(end, self.goal_align_forward.value)
                        score += self.scales["GoalAlign"].value * math.hypot(goal_ahead[0] - local_goal[0], goal_ahead[1] - local_goal[1]) / res
                    if best is None or score < best[0]:
                        best = (score, (v, w), trajectory)
        if best is None:
            raise ControllerError(FollowPathResult.NO_VALID_CONTROL, "모든 자취가 벽에 닿는다")
        self.best = best[2]
        return best[1]

    def _local_plan(self, pose: Pose2, path: list[Pose2]) -> list[Pose2]:
        """가장 가까운 점부터 prune_distance 안의 경로 (그 끝이 지역 목표)."""
        rest = path[closest_index(pose, path):]
        limit = self.prune_distance.value
        local = [p for p in rest if math.hypot(p[0] - pose[0], p[1] - pose[1]) <= limit]
        return local or rest[:1]


def _samples(low: float, high: float, count: int) -> list[float]:
    if high <= low or count == 1:
        return [max(low, min(high, 0.0)) if low <= 0.0 <= high else low]
    return [low + (high - low) * k / (count - 1) for k in range(count)]


def _ahead(pose: Pose2, distance: float) -> tuple[float, float]:
    return pose[0] + distance * math.cos(pose[2]), pose[1] + distance * math.sin(pose[2])


def _distance_to_path(point: tuple[float, float] | Pose2, path: list[Pose2]) -> float:
    return min(math.hypot(point[0] - p[0], point[1] - p[1]) for p in path)
