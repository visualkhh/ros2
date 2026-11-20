"""nav2_mppi_controller::MPPIController 흉내 (Model Predictive Path Integral): 지난 제어 시퀀스에 잡음을 섞은 후보 batch_size개를
time_steps 동안 굴려 critic 비용을 매기고, exp(-비용/temperature)로 가중 평균한 시퀀스의 첫 명령을 낸다. 시퀀스는 한 칸 밀어 다음 주기에 이어 쓴다."""
import math
import random

from kros.msgs.nav2_msgs import FollowPathResult
from kros.nodes.nav2.costmap_2d import INSCRIBED_INFLATED_OBSTACLE, Costmap
from kros.nodes.node import Node

from .controller import Controller, ControllerError, Pose2, Velocity, closest_index, wrap

CRITIC_WEIGHTS = {  # nav2 기본 설정의 cost_weight
    "CostCritic": 3.81,
    "GoalCritic": 5.0,
    "GoalAngleCritic": 3.0,
    "PathAlignCritic": 14.0,
    "PathFollowCritic": 5.0,
    "PathAngleCritic": 2.0,
    "PreferForwardCritic": 5.0,
}
COLLISION_COST = 1_000_000.0  # nav2 CostCritic의 collision_cost


class MPPIController(Controller):
    """파라미터 ("<이름>." 아래, nav2와 같은 이름): batch_size, time_steps, model_dt, vx_std, wz_std, vx_max, vx_min, wz_max, temperature,
    "<critic>.cost_weight", GoalCritic/GoalAngleCritic/PathFollowCritic.threshold_to_consider (목표에서 이 거리 안이면 켜진다/꺼진다),
    PathFollowCritic.offset_from_furthest, PathAngleCritic.offset_from_furthest / max_angle_to_furthest.
    critic (낮을수록 좋다): CostCritic(지나는 칸 비용, 벽에 닿으면 사실상 버림), GoalCritic/GoalAngleCritic(목표 가까이에서 목표 자리/방향),
    PathAlignCritic(자취가 경로에서 떨어진 정도), PathFollowCritic(경로 앞쪽 점까지 거리: 앞으로 나아가게), PathAngleCritic(경로 앞쪽 점을 보게), PreferForwardCritic(후진 벌점).
    기본 batch_size/time_steps는 nav2(2000 x 56)보다 훨씬 작다: 순수 파이썬으로 20Hz에 돌게. seed가 같으면 같은 잡음.
    """

    def configure(self, node: Node, name: str) -> None:
        super().configure(node, name)
        positive = lambda value: value > 0  # noqa: E731
        non_negative = lambda value: value >= 0  # noqa: E731
        self.batch_size = node.declare_parameter(f"{name}.batch_size", 80, validator=positive)
        self.time_steps = node.declare_parameter(f"{name}.time_steps", 20, validator=positive)
        self.model_dt = node.declare_parameter(f"{name}.model_dt", 0.1, validator=positive)
        self.vx_std = node.declare_parameter(f"{name}.vx_std", 0.2, validator=non_negative)
        self.wz_std = node.declare_parameter(f"{name}.wz_std", 0.4, validator=non_negative)
        self.vx_max = node.declare_parameter(f"{name}.vx_max", 0.22, validator=positive)
        self.vx_min = node.declare_parameter(f"{name}.vx_min", -0.1, validator=lambda value: value <= 0)
        self.wz_max = node.declare_parameter(f"{name}.wz_max", 1.5, validator=positive)
        self.temperature = node.declare_parameter(f"{name}.temperature", 0.3, validator=positive)
        self.seed = node.declare_parameter(f"{name}.seed", 0)
        self.weights = {critic: node.declare_parameter(f"{name}.{critic}.cost_weight", weight, validator=non_negative) for critic, weight in CRITIC_WEIGHTS.items()}
        self.goal_threshold = node.declare_parameter(f"{name}.GoalCritic.threshold_to_consider", 1.0, validator=positive)
        self.goal_angle_threshold = node.declare_parameter(f"{name}.GoalAngleCritic.threshold_to_consider", 0.2, validator=positive)  # 도착 허용(0.25)보다 작게: 먼저 자리, 그다음 방향 (차동 구동은 옆으로 못 가서, 멀리서 방향부터 맞추면 멈춘다)
        self.path_follow_threshold = node.declare_parameter(f"{name}.PathFollowCritic.threshold_to_consider", 1.0, validator=positive)
        self.path_follow_offset = node.declare_parameter(f"{name}.PathFollowCritic.offset_from_furthest", 6, validator=positive)
        self.path_angle_offset = node.declare_parameter(f"{name}.PathAngleCritic.offset_from_furthest", 4, validator=positive)
        self.max_angle_to_furthest = node.declare_parameter(f"{name}.PathAngleCritic.max_angle_to_furthest", 0.785, validator=positive)
        self._rng = random.Random(self.seed.value)
        self.control: list[Velocity] = []  # 지난 주기에 고른 제어 시퀀스 (warm start)
        self.best: list[Pose2] = []  # 고른 시퀀스를 굴린 자취 (화면/시험용)

    def compute_velocity_commands(self, pose: Pose2, velocity: Velocity, path: list[Pose2], costmap: Costmap, dt: float) -> Velocity:
        steps, model_dt = self.time_steps.value, self.model_dt.value
        if len(self.control) != steps:
            self.control = [velocity] * steps
        vx_min, vx_max, wz_max = self.vx_min.value, self.vx_max.value, self.wz_max.value
        start = closest_index(pose, path)
        reach = int(vx_max * steps * model_dt / max(costmap.resolution, 1e-6)) * 2 + 10
        local = path[start : start + reach][::2] or path[-1:]  # 자취가 닿을 만한 경로 앞부분 (거리 계산을 줄이려고 하나 건너)
        goal = path[-1]
        to_goal = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
        follow = path[min(len(path) - 1, start + self.path_follow_offset.value)]
        ahead = path[min(len(path) - 1, start + self.path_angle_offset.value)]
        rng, vx_std, wz_std = self._rng, self.vx_std.value, self.wz_std.value
        candidates: list[tuple[float, list[Velocity], list[Pose2]]] = []
        for k in range(self.batch_size.value):
            noisy = self.control if k == 0 else [  # 0번은 지난 시퀀스 그대로 (잡음 없이)
                (min(vx_max, max(vx_min, v + rng.gauss(0.0, vx_std))), min(wz_max, max(-wz_max, w + rng.gauss(0.0, wz_std)))) for v, w in self.control
            ]
            trajectory = _rollout(pose, noisy, model_dt)
            candidates.append((self._score(trajectory, noisy, costmap, local, goal, to_goal, follow, ahead, pose, model_dt), noisy, trajectory))
        lowest = min(c[0] for c in candidates)
        if lowest >= COLLISION_COST:
            self.control = []
            raise ControllerError(FollowPathResult.NO_VALID_CONTROL, "모든 후보가 벽에 닿는다")
        temperature = self.temperature.value
        weights = [math.exp(-(c[0] - lowest) / temperature) for c in candidates]
        total = sum(weights)
        self.control = [
            (sum(wt * c[1][t][0] for wt, c in zip(weights, candidates)) / total, sum(wt * c[1][t][1] for wt, c in zip(weights, candidates)) / total) for t in range(steps)
        ]
        self.best = self.local_plan = _rollout(pose, self.control, model_dt)
        command = self.control[0]
        self.control = self.control[1:] + self.control[-1:]  # 한 칸 밀어 다음 주기의 출발점으로
        return command

    def _score(
        self, trajectory: list[Pose2], controls: list[Velocity], costmap: Costmap, local: list[Pose2], goal: Pose2, to_goal: float, follow: Pose2, ahead: Pose2, pose: Pose2, model_dt: float
    ) -> float:
        w = {name: parameter.value for name, parameter in self.weights.items()}
        n = len(trajectory)
        score = 0.0
        cost_sum = 0.0
        for x, y, _ in trajectory:  # CostCritic
            c = costmap.cost_at(x, y)
            if c >= INSCRIBED_INFLATED_OBSTACLE and math.hypot(x - pose[0], y - pose[1]) > 0.1:
                return COLLISION_COST
            cost_sum += min(c, INSCRIBED_INFLATED_OBSTACLE - 1) / 252
        score += w["CostCritic"] * cost_sum / n
        end = trajectory[-1]
        if to_goal < self.goal_threshold.value:  # GoalCritic: 목표 가까이에서는 목표 자리로
            score += w["GoalCritic"] * sum(math.hypot(x - goal[0], y - goal[1]) for x, y, _ in trajectory) / n
        if to_goal < self.goal_angle_threshold.value:  # GoalAngleCritic
            score += w["GoalAngleCritic"] * sum(abs(wrap(yaw - goal[2])) for _, _, yaw in trajectory) / n
        if to_goal >= self.goal_angle_threshold.value:  # PathAlignCritic: 목표 가까이에서는 끈다 (경로 끝을 넘어서려 하지 않게)
            score += w["PathAlignCritic"] * sum(min(math.hypot(x - p[0], y - p[1]) for p in local) for x, y, _ in trajectory[1::2]) / max(1, n // 2)
        if to_goal >= self.path_follow_threshold.value:  # PathFollowCritic: 경로 앞쪽 점까지 가까이 (앞으로 나아가게)
            score += w["PathFollowCritic"] * math.hypot(end[0] - follow[0], end[1] - follow[1])
        angle_to_ahead = math.atan2(ahead[1] - pose[1], ahead[0] - pose[0])
        if to_goal >= self.goal_angle_threshold.value and abs(wrap(angle_to_ahead - pose[2])) > self.max_angle_to_furthest.value:  # PathAngleCritic
            score += w["PathAngleCritic"] * sum(abs(wrap(math.atan2(ahead[1] - y, ahead[0] - x) - yaw)) for x, y, yaw in trajectory) / n
        score += w["PreferForwardCritic"] * sum(max(0.0, -v) * model_dt for v, _ in controls)  # PreferForwardCritic
        return score


def _rollout(pose: Pose2, controls: list[Velocity], dt: float) -> list[Pose2]:
    x, y, yaw = pose
    points = []
    for v, w in controls:
        yaw += w * dt
        x, y = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
        points.append((x, y, yaw))
    return points
