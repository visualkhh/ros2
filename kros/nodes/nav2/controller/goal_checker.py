"""nav2_controller의 SimpleGoalChecker / SimpleProgressChecker 흉내."""
import math

from kros.nodes.node import Node

from .controller import Pose2, wrap


class SimpleGoalChecker:
    """경로 끝에서 xy_goal_tolerance(m) 안이고 방향이 yaw_goal_tolerance(rad) 안이면 도착."""

    def __init__(self, node: Node, name: str = "general_goal_checker") -> None:
        positive = lambda value: value > 0  # noqa: E731
        self.xy_goal_tolerance = node.declare_parameter(f"{name}.xy_goal_tolerance", 0.25, validator=positive)
        self.yaw_goal_tolerance = node.declare_parameter(f"{name}.yaw_goal_tolerance", 0.25, validator=positive)

    def is_goal_reached(self, pose: Pose2, goal: Pose2) -> bool:
        return math.hypot(goal[0] - pose[0], goal[1] - pose[1]) <= self.xy_goal_tolerance.value and abs(wrap(goal[2] - pose[2])) <= self.yaw_goal_tolerance.value


class SimpleProgressChecker:
    """movement_time_allowance(초) 동안 required_movement_radius(m)도 못 움직이면 막힌 것."""

    def __init__(self, node: Node, name: str = "progress_checker") -> None:
        positive = lambda value: value > 0  # noqa: E731
        self.required_movement_radius = node.declare_parameter(f"{name}.required_movement_radius", 0.5, validator=positive)
        self.movement_time_allowance = node.declare_parameter(f"{name}.movement_time_allowance", 10.0, validator=positive)
        # 제자리 회전도 진행으로 친다 (목표 방향/경로 회전에서 false 105가 나지 않게)
        self.required_angle = node.declare_parameter(f"{name}.required_angle", 0.5, validator=positive)
        self._baseline: tuple[Pose2, float] | None = None

    def reset(self) -> None:
        self._baseline = None

    def check(self, pose: Pose2, now: float) -> bool:
        """아직 괜찮으면 True."""
        baseline = self._baseline
        if baseline is None:
            self._baseline = (pose, now)
            return True
        moved = math.hypot(pose[0] - baseline[0][0], pose[1] - baseline[0][1]) > self.required_movement_radius.value
        turned = abs(wrap(pose[2] - baseline[0][2])) > self.required_angle.value
        if moved or turned:
            self._baseline = (pose, now)
            return True
        return now - baseline[1] <= self.movement_time_allowance.value
