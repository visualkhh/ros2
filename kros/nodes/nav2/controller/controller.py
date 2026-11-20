"""nav2_core::Controller 흉내: controller_server에 끼우는 경로 따라가기 알고리즘의 공통 모양과 도우미."""
import math

from kros.nodes.nav2.costmap_2d import INSCRIBED_INFLATED_OBSTACLE, Costmap
from kros.nodes.node import Node

Pose2 = tuple[float, float, float]  # (x, y, yaw) global_frame
Velocity = tuple[float, float]  # (앞 m/s, 회전 rad/s)


class ControllerError(Exception):
    """제어 실패. code는 FollowPathResult의 오류 코드."""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code


class Controller:
    """플러그인: configure(node, name)에서 "<name>.<파라미터>"를 선언하고, compute_velocity_commands가 이번 주기의 (v, w)를 돌려준다 (실패면 ControllerError).
    path는 global_frame의 (x, y, yaw) 점들, velocity는 지금 로봇 속도(odom), dt는 제어 주기(초)."""

    def configure(self, node: Node, name: str) -> None:
        self.node, self.name = node, name

    def compute_velocity_commands(self, pose: Pose2, velocity: Velocity, path: list[Pose2], costmap: Costmap, dt: float) -> Velocity:
        raise NotImplementedError


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def closest_index(pose: Pose2, path: list[Pose2]) -> int:
    """경로에서 로봇에 가장 가까운 점 (지나온 점은 버리고 여기부터 본다)."""
    return min(range(len(path)), key=lambda k: (path[k][0] - pose[0]) ** 2 + (path[k][1] - pose[1]) ** 2)


def to_robot(pose: Pose2, point: tuple[float, float]) -> tuple[float, float]:
    """global 점을 로봇 축(앞 x, 왼쪽 y)으로."""
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


def blocked(costmap: Costmap, x: float, y: float) -> bool:
    """로봇 중심이 여기 오면 몸이 벽에 닿나 (원형 로봇: 중심 칸 비용이 inscribed 이상). 모름은 막힌 것으로 본다."""
    return costmap.cost_at(x, y) >= INSCRIBED_INFLATED_OBSTACLE  # 253, 254, 모름(255)


def simulate(pose: Pose2, v: float, w: float, seconds: float, step: float) -> list[Pose2]:
    """(v, w)로 seconds 동안 굴러간 자취 (step초 간격, 출발점 제외)."""
    x, y, yaw = pose
    points = []
    for _ in range(max(1, round(seconds / step))):
        x, y, yaw = x + v * math.cos(yaw) * step, y + v * math.sin(yaw) * step, yaw + w * step
        points.append((x, y, yaw))
    return points
