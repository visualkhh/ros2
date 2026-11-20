"""nav2_regulated_pure_pursuit_controller 흉내: 경로 위 lookahead 거리의 점(carrot)으로 가는 원호를 그리고, 그 곡률로 회전 속도를 정한다.
"regulated": 급커브(곡률 반지름이 작을 때)와 목표 가까이에서는 앞 속도를 줄인다. carrot이 옆/뒤에 있으면 먼저 제자리에서 돈다."""
import math

from kros.msgs.nav2_msgs import FollowPathResult
from kros.nodes.nav2.costmap_2d import Costmap
from kros.nodes.node import Node

from .controller import Controller, ControllerError, Pose2, Velocity, blocked, closest_index, simulate, to_robot, wrap


class RegulatedPurePursuitController(Controller):
    """파라미터 ("<이름>." 아래, nav2와 같은 이름): desired_linear_vel, lookahead_dist, min_lookahead_dist, max_lookahead_dist, lookahead_time,
    use_velocity_scaled_lookahead_dist, use_rotate_to_heading, rotate_to_heading_angular_vel, rotate_to_heading_min_angle, max_angular_accel,
    use_regulated_linear_velocity_scaling, regulated_linear_scaling_min_radius, regulated_linear_scaling_min_speed,
    approach_velocity_scaling_dist, min_approach_linear_velocity, use_collision_detection, max_allowed_time_to_collision_up_to_carrot,
    goal_dist_tol (경로 끝에서 이만큼 가까우면 앞으로 가지 않고 목표 방향으로 돈다).
    shortcut: cost-regulated 속도(벽 가까우면 감속)는 없다, 좁은 곳에서 더 조심해야 하면 추가."""

    def configure(self, node: Node, name: str) -> None:
        super().configure(node, name)
        positive = lambda value: value > 0  # noqa: E731
        self.desired_linear_vel = node.declare_parameter(f"{name}.desired_linear_vel", 0.22, validator=positive)  # TurtleBot3 최고 0.26
        self.lookahead_dist = node.declare_parameter(f"{name}.lookahead_dist", 0.6, validator=positive)
        self.min_lookahead_dist = node.declare_parameter(f"{name}.min_lookahead_dist", 0.3, validator=positive)
        self.max_lookahead_dist = node.declare_parameter(f"{name}.max_lookahead_dist", 0.9, validator=positive)
        self.lookahead_time = node.declare_parameter(f"{name}.lookahead_time", 1.5, validator=positive)
        self.use_velocity_scaled_lookahead_dist = node.declare_parameter(f"{name}.use_velocity_scaled_lookahead_dist", False)
        self.use_rotate_to_heading = node.declare_parameter(f"{name}.use_rotate_to_heading", True)
        self.rotate_to_heading_angular_vel = node.declare_parameter(f"{name}.rotate_to_heading_angular_vel", 1.8, validator=positive)
        self.rotate_to_heading_min_angle = node.declare_parameter(f"{name}.rotate_to_heading_min_angle", 0.785, validator=positive)
        self.max_angular_accel = node.declare_parameter(f"{name}.max_angular_accel", 3.2, validator=positive)
        self.use_regulated_linear_velocity_scaling = node.declare_parameter(f"{name}.use_regulated_linear_velocity_scaling", True)
        self.regulated_linear_scaling_min_radius = node.declare_parameter(f"{name}.regulated_linear_scaling_min_radius", 0.9, validator=positive)
        self.regulated_linear_scaling_min_speed = node.declare_parameter(f"{name}.regulated_linear_scaling_min_speed", 0.05, validator=positive)
        self.approach_velocity_scaling_dist = node.declare_parameter(f"{name}.approach_velocity_scaling_dist", 0.6, validator=positive)
        self.min_approach_linear_velocity = node.declare_parameter(f"{name}.min_approach_linear_velocity", 0.05, validator=positive)
        self.use_collision_detection = node.declare_parameter(f"{name}.use_collision_detection", True)
        self.max_allowed_time_to_collision_up_to_carrot = node.declare_parameter(f"{name}.max_allowed_time_to_collision_up_to_carrot", 1.0, validator=positive)
        self.goal_dist_tol = node.declare_parameter(f"{name}.goal_dist_tol", 0.2, validator=positive)
        self.carrot: tuple[float, float] | None = None  # 지난 주기의 carrot (global, 화면/시험용)

    def compute_velocity_commands(self, pose: Pose2, velocity: Velocity, path: list[Pose2], costmap: Costmap, dt: float) -> Velocity:
        rest = path[closest_index(pose, path):]
        goal = path[-1]
        self.local_plan = []
        if math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < self.goal_dist_tol.value:  # 거의 왔다: 목표 방향으로만 돈다
            return 0.0, self._rotate(wrap(goal[2] - pose[2]), velocity[1], dt)
        lookahead = self.lookahead_dist.value
        if self.use_velocity_scaled_lookahead_dist.value:
            lookahead = min(self.max_lookahead_dist.value, max(self.min_lookahead_dist.value, abs(velocity[0]) * self.lookahead_time.value))
        carrot = next((p for p in rest if math.hypot(p[0] - pose[0], p[1] - pose[1]) >= lookahead), rest[-1])
        self.carrot = carrot[:2]
        cx, cy = to_robot(pose, carrot[:2])
        angle = math.atan2(cy, cx)
        if self.use_rotate_to_heading.value and abs(angle) > self.rotate_to_heading_min_angle.value:  # 옆/뒤: 제자리에서 그쪽을 본다
            return 0.0, self._rotate(angle, velocity[1], dt)
        distance2 = cx * cx + cy * cy
        curvature = 2.0 * cy / distance2 if distance2 > 1e-9 else 0.0
        v = self.desired_linear_vel.value
        if self.use_regulated_linear_velocity_scaling.value and abs(curvature) > 1e-9:
            radius = 1.0 / abs(curvature)
            if radius < self.regulated_linear_scaling_min_radius.value:  # 급커브: 반지름에 비례해 늦춘다
                v = max(v * radius / self.regulated_linear_scaling_min_radius.value, self.regulated_linear_scaling_min_speed.value)
        remaining = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(rest, rest[1:])) + math.hypot(rest[0][0] - pose[0], rest[0][1] - pose[1])
        if remaining < self.approach_velocity_scaling_dist.value:  # 목표 가까이: 남은 거리에 비례해 늦춘다
            v = max(v * remaining / self.approach_velocity_scaling_dist.value, self.min_approach_linear_velocity.value)
        w = v * curvature
        seconds = min(self.max_allowed_time_to_collision_up_to_carrot.value, math.sqrt(distance2) / max(v, 1e-6))
        self.local_plan = simulate(pose, v, w, seconds, costmap.resolution / max(v, 1e-6))  # carrot까지의 원호
        if self.use_collision_detection.value:
            for x, y, _ in self.local_plan:
                if math.hypot(x - pose[0], y - pose[1]) > 0.1 and blocked(costmap, x, y):  # 바로 둘레는 빼고 (벽에 붙어 있어도 빠져나간다)
                    raise ControllerError(FollowPathResult.NO_VALID_CONTROL, f"앞 경로 ({x:.2f}, {y:.2f})에서 벽에 닿는다")
        return v, w

    def _rotate(self, angle: float, current: float, dt: float) -> float:
        """angle 쪽으로 rotate_to_heading_angular_vel로 돈다. 가속은 max_angular_accel까지, 다 돌기 전에 줄인다."""
        target = math.copysign(min(self.rotate_to_heading_angular_vel.value, math.sqrt(2 * self.max_angular_accel.value * abs(angle))), angle)
        step = self.max_angular_accel.value * dt
        return min(current + step, max(current - step, target))
