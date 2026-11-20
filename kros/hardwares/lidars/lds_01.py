"""ROBOTIS LDS-01 2D 라이다 (TurtleBot3 Burger/Waffle Pi): 360도, 1도 간격 360샘플, 0.12~3.5m, 5Hz."""
import dataclasses
import math
import random

from kros.msgs.geometry_msgs import Pose2D
from kros.msgs.sensor_msgs import LaserScan
from kros.msgs.std_msgs import Header
from kros.worlds.clock import Clock
from kros.hardwares.lidars.lidar import Lidar
from kros.hardwares.noise import Noise
from kros.hardwares.simulated_body import simulated_body
from kros.worlds.world import World
from kros.worlds.objects.world_object import WorldObject

SAMPLES = 360  # 1도 간격
RANGE_MIN, RANGE_MAX = 0.12, 3.5  # m
SCAN_PERIOD = 0.2  # 초 (5Hz)


def make_scan(sensor: Pose2D, height: float, world: World, header: Header = Header(0.0, "base_scan")) -> LaserScan:
    """sensor 위치에서 바닥 위 height 높이로 한 바퀴를 쏴서 LDS-01의 스캔을 만든다 (범위 밖은 inf)."""
    inc = 2 * math.pi / SAMPLES
    hits = world.ray_distances(sensor.x, sensor.y, [sensor.theta + i * inc for i in range(SAMPLES)], height)
    ranges = tuple(r if RANGE_MIN <= r <= RANGE_MAX else math.inf for r in hits)
    return LaserScan(header, 0.0, 2 * math.pi - inc, inc, RANGE_MIN, RANGE_MAX, ranges)


class Lds01(Lidar):
    """LDS-01 라이다의 공통부: 5Hz. 스캔을 만드는 sample()은 구현(시뮬레이션/더미/진짜)이 한다."""

    model = "LDS-01"

    def __init__(self, mount_x: float = 0.0, mount_height: float = 0.0, period: float | None = SCAN_PERIOD) -> None:
        super().__init__(mount_x, mount_height, period)


class SimulatedLds01(Lds01):
    """월드에 레이를 쏴서 측정한다: owner(로봇)의 위치에서. 오차(기본은 없음): range는 거리마다 붙는 Noise(m), dropout은 측정 하나가 실패해 inf가 될 확률. seed가 같으면 같은 오차."""

    def __init__(
        self, mount_x: float = 0.0, mount_height: float = 0.0, period: float | None = SCAN_PERIOD, range: Noise = Noise(), dropout: float = 0.0, seed: int | None = None
    ) -> None:
        super().__init__(mount_x, mount_height, period)
        self.range_noise, self.dropout = range, dropout
        self._rng = random.Random(seed)

    def sample(self, clock: Clock) -> LaserScan:
        body, world = simulated_body(self)
        self.scan_pose = self.sensor_pose(body.pose)
        scan = make_scan(self.scan_pose, self.mount_height, world, Header(clock.time, self.frame_id))
        rng = self._rng
        ranges = tuple(r if math.isinf(r) else math.inf if rng.random() < self.dropout else self.range_noise.apply(r, rng) for r in scan.ranges)
        return dataclasses.replace(scan, ranges=ranges)


class DummyLds01(Lds01):
    """월드와 상관없이 모든 방향이 같은 거리인 가짜 스캔을 보낸다 (월드 없이 파이프라인을 시험할 때)."""

    def __init__(self, distance: float = 1.0, mount_x: float = 0.0, mount_height: float = 0.0, period: float | None = SCAN_PERIOD) -> None:
        super().__init__(mount_x, mount_height, period)
        self.distance = distance

    def sample(self, clock: Clock) -> LaserScan:
        inc = 2 * math.pi / SAMPLES
        body = self.owner
        if isinstance(body, WorldObject):
            self.scan_pose = self.sensor_pose(body.pose)
        return LaserScan(Header(clock.time, self.frame_id), 0.0, 2 * math.pi - inc, inc, RANGE_MIN, RANGE_MAX, (self.distance,) * SAMPLES)
