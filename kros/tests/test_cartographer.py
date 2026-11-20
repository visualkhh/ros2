import math
import unittest

from kros import Kros
from kros.context import Context
from kros.examples.turtlebot3.main import IMU_CONFIG, ODOM_CONFIG
from kros.examples.turtlebot3.robot_bridge import RobotBridge
from kros.msgs.geometry_msgs import Point, Pose, Pose2D, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion, Twist, Vector3
from kros.msgs.nav_msgs import UNKNOWN, OccupancyGrid
from kros.msgs.std_msgs import Header
from kros.nodes import Cartographer, DiffDriveOdometry, Ekf, Node, RobotStatePublisher
from kros.nodes.cartographer import ProbabilityGrid, match
from kros.nodes.cartographer.cartographer import compose, inverse
from kros.parameter import ParameterRejectedError
from kros.tf import Buffer, TransformListener
from kros.timer import SimulatedTimeSource
from kros.topic import QOS_LATCHED
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3


def room_scan(sensor: tuple[float, float, float], size: float = 2.0, rays: int = 180, wall: float = 0.013) -> list[tuple[float, float]]:
    """벽이 wall과 wall+size에 있는 방을 sensor 위치에서 본 끝점들 (센서 좌표).
    벽을 격자선(0.05의 배수)에서 비켜 둔다: 격자선 위의 벽은 끝점이 늘 바깥 칸에 들어가 지도가 반 칸 밀린 것처럼 보인다 (진짜 벽은 격자에 맞춰 있지 않다)."""
    x, y, yaw = sensor[0] - wall, sensor[1] - wall, sensor[2]
    points = []
    for i in range(rays):
        a = yaw + 2 * math.pi * i / rays
        c, s = math.cos(a), math.sin(a)
        r = min(d for d in ((size - x) / c if c > 1e-9 else math.inf, -x / c if c < -1e-9 else math.inf, (size - y) / s if s > 1e-9 else math.inf, -y / s if s < -1e-9 else math.inf))
        local = a - yaw
        points.append((r * math.cos(local), r * math.sin(local)))
    return points


def to_map(sensor: tuple[float, float, float], points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [compose(sensor, (px, py, 0.0))[:2] for px, py in points]


class TestProbabilityGrid(unittest.TestCase):
    def test_ray_end_becomes_wall_and_the_way_there_free(self) -> None:
        grid = ProbabilityGrid(0.1)
        grid.insert((0.05, 0.05), [(0.55, 0.05)])
        self.assertGreater(grid.probability(0.55, 0.05), 0.6)
        self.assertLess(grid.probability(0.25, 0.05), 0.45)
        self.assertEqual(grid.probability(0.05, 3.0), 0.5)  # 모르는 곳

    def test_message_has_probabilities_and_unknown_and_reads_back(self) -> None:
        grid = ProbabilityGrid(0.1)
        grid.insert((0.05, 0.05), [(0.35, 0.05), (0.05, 0.35)])
        message = grid.to_message(Header(1.0, "map"))
        info = message.info
        self.assertEqual((info.width, info.height), (4, 4))
        self.assertIn(UNKNOWN, message.data)  # (3, 3) 같은 칸은 레이가 안 지나갔다
        self.assertGreater(message.data[3], 50)  # (3, 0) = 벽
        back = ProbabilityGrid.from_message(message)
        self.assertAlmostEqual(back.probability(0.35, 0.05), grid.probability(0.35, 0.05), places=2)


class TestScanMatcher(unittest.TestCase):
    def test_finds_the_true_pose_from_a_wrong_guess(self) -> None:
        grid = ProbabilityGrid(0.05)
        truth = (0.8, 0.9, 0.2)
        for sensor in ((0.8, 0.9, 0.2), (1.2, 0.7, -0.5), (0.5, 1.4, 1.0)):  # 몇 군데서 본 방으로 지도를 만든다
            grid.insert(sensor[:2], to_map(sensor, room_scan(sensor)))
        found, value = match(grid, room_scan(truth), (0.86, 0.85, 0.25))  # 6cm, 3도쯤 틀린 예상
        self.assertAlmostEqual(found[0], truth[0], delta=0.015)
        self.assertAlmostEqual(found[1], truth[1], delta=0.015)
        self.assertAlmostEqual(found[2], truth[2], delta=math.radians(1))
        self.assertGreater(value, 0.7)


class Simulation:
    """노이즈 TurtleBot3 + EKF(odom) + Cartographer(map). 맨 끝에 벽으로 밀어붙이면 바퀴가 헛돌아 odom은 틀리지만 라이다는 안다."""

    def __init__(self, test: unittest.TestCase, mode: str, given_map: OccupancyGrid | None = None, offset: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.world, self.robot = MazeWorld(puck_count=0), TurtleBot3(noisy=True, seed=1)
        start = self.world.start_pose
        self.start = (start.x, start.y, start.theta)  # map 원점 = 지도를 만들 때 로봇이 켜진 곳
        self.world.add(self.robot, Pose2D(*compose(self.start, offset)))  # offset: 그 원점에서 본 이번 시작 위치
        self.source = SimulatedTimeSource()
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["watcher"], made["slam"] = Node(ctx, "Watcher"), Cartographer(ctx, parameters={"mode": mode})
            ekf = Ekf(ctx, parameters={"odom0": "odom", "odom0_config": ODOM_CONFIG, "imu0": "imu", "imu0_config": IMU_CONFIG})
            description = RobotStatePublisher(ctx, parameters={"robot_description": TurtleBot3.urdf_path.read_text()})
            return [made["watcher"], RobotBridge(ctx, self.robot), description, DiffDriveOdometry(ctx, parameters={"publish_tf": False}), ekf, made["slam"]]

        self.kros = Kros(create, time_source=self.source)
        test.addCleanup(self.kros.shutdown)
        slam = made["slam"]
        assert isinstance(slam, Cartographer)
        self.slam, self.watcher = slam, made["watcher"]
        self.maps: list[OccupancyGrid] = []
        self.watcher.subscribe("map", OccupancyGrid, self.maps.append, QOS_LATCHED)
        if given_map is not None:
            self.watcher.publish("map", given_map, QOS_LATCHED)  # map_server 자리
        self.buffer = Buffer()
        TransformListener(self.buffer, self.watcher)

    def run(self, seconds: float, v: float = 0.0, w: float = 0.0) -> None:
        self.robot.command = Twist(Vector3(x=v), Vector3(z=w))
        for _ in range(round(seconds / 0.02)):
            self.world.step(0.02)
            self.source.set(self.world.time)
            self.kros.context.timer_manager.join()
        self.kros.context.topic_manager.join()

    def error(self, frame: str) -> float:
        t = self.buffer.lookup_transform(frame, "base_footprint").transform
        truth = compose(inverse(self.start), (self.robot.pose.x, self.robot.pose.y, self.robot.pose.theta))
        return math.hypot(t.translation.x - truth[0], t.translation.y - truth[1])

    def drive_into_the_wall(self) -> None:
        self.run(3.0)  # IMU 보정 동안 서 있는다
        self.run(math.pi / 0.6, w=0.6)  # 반 바퀴 돌며 둘레를 본다
        self.run(math.pi / 0.6, w=0.6)
        self.run(9.0, v=0.2)  # 앞 벽까지 1.1m: 막힌 뒤에도 바퀴는 돈다
        self.run(1.0)  # 바퀴가 도는 동안엔 스캔 사이(0.2초)에 odom이 미끄러진 만큼 map 위치도 잠깐 밀린다: 멈추고 다음 스캔이 고친 뒤에 잰다


class TestCartographer(unittest.TestCase):
    def test_mapping_keeps_the_true_pose_when_odometry_slips_on_a_wall(self) -> None:
        sim = Simulation(self, "mapping")
        sim.drive_into_the_wall()
        self.assertGreater(sim.error("odom"), 0.3)  # 바퀴(+EKF)는 벽을 뚫고 간 줄 안다
        self.assertLess(sim.error("map"), 0.05)  # 지도에 맞춘 위치는 벽 앞에 있다
        self.assertTrue(sim.maps)
        self.assertEqual(sim.maps[-1].header.frame_id, "map")
        self.assertGreater(sum(1 for v in sim.maps[-1].data if v > 65), 50)  # 벽이 그려졌다

    def test_localization_uses_the_given_map_and_does_not_change_it(self) -> None:
        mapping = Simulation(self, "mapping")
        mapping.run(3.0)
        mapping.run(2 * math.pi / 0.6, w=0.6)
        mapping.run(1.5)  # 지도를 낼 시간
        saved = mapping.maps[-1]
        sim = Simulation(self, "localization", given_map=saved)
        sim.drive_into_the_wall()
        self.assertLess(sim.error("map"), 0.05)
        self.assertGreater(sim.error("odom"), 0.3)
        self.assertEqual(sim.maps, [saved])  # 지도는 map_server(여기서는 Watcher)가 낸 것뿐: 고치지 않는다

    def test_initialpose_puts_the_robot_on_the_map_and_matching_refines_it(self) -> None:
        mapping = Simulation(self, "mapping")
        mapping.run(3.0)
        mapping.run(2 * math.pi / 0.6, w=0.6)
        mapping.run(1.5)
        offset = (0.2, 0.0, 0.25)  # 지도 원점에서 20cm 앞, 14도 돈 곳에서 켠다: 그냥 두면 원점에 있다고 믿는다
        sim = Simulation(self, "localization", given_map=mapping.maps[-1], offset=offset)
        sim.run(3.0)
        self.assertGreater(sim.error("map"), 0.15)
        x, y, yaw = 0.25, 0.03, 0.2  # 사람이 대충 찍은 위치 (5cm, 3도쯤 틀림)
        sim.watcher.publish("initialpose", PoseWithCovarianceStamped(Header(0.0, "map"), PoseWithCovariance(Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))))))
        sim.run(1.0)
        self.assertLess(sim.error("map"), 0.03)  # 지도에 맞춰 진짜 위치로 다듬었다
        with self.assertLogs("kros.node.cartographer", level="WARNING"):
            sim.watcher.publish("initialpose", PoseWithCovarianceStamped(Header(0.0, "odom")))  # map 좌표계가 아니면 버린다
            sim.run(0.1)

    def test_idle_does_nothing_and_mode_is_validated(self) -> None:
        sim = Simulation(self, "idle")
        sim.run(1.0)
        self.assertEqual(sim.slam.scans, 0)
        self.assertFalse(sim.buffer.can_transform("map", "odom"))
        with self.assertRaises(ParameterRejectedError):
            sim.slam.set_parameter("mode", "dance")
        sim.slam.set_parameter("mode", "mapping")  # 실행 중에 켠다
        sim.run(1.0)
        self.assertGreater(sim.slam.scans, 0)
        self.assertTrue(sim.buffer.can_transform("map", "odom"))


if __name__ == "__main__":
    unittest.main()
