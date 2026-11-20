import math
import unittest

from kros import Kros
from kros.context import Context
from kros.examples.turtlebot3.robot_bridge import RobotBridge
from kros.msgs.geometry_msgs import PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3
from kros.msgs.nav_msgs import Odometry
from kros.msgs.sensor_msgs import Imu
from kros.msgs.std_msgs import Header
from kros.nodes import DiffDriveOdometry, Ekf, Node, RobotStatePublisher
from kros.nodes.ekf import _inverse, _mul
from kros.tf import Buffer, TransformListener
from kros.tf.transform_math import to_rpy
from kros.timer import SimulatedTimeSource
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3

V_ONLY = (False,) * 6 + (True,) + (False,) * 8  # vx
W_ONLY = (False,) * 11 + (True,) + (False,) * 3  # vyaw
YAW_ONLY = (False,) * 5 + (True,) + (False,) * 9  # yaw
TWIST_COV = tuple(0.001 if i % 7 == 0 else 0.0 for i in range(36))


def yaw_of(q: Quaternion) -> float:
    return to_rpy(q)[2]


class TestMatrix(unittest.TestCase):
    def test_inverse_times_matrix_is_identity(self) -> None:
        a = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]]
        for i, row in enumerate(_mul(a, _inverse(a))):
            for j, value in enumerate(row):
                self.assertAlmostEqual(value, 1.0 if i == j else 0.0)

    def test_inverse_needs_pivoting(self) -> None:
        self.assertEqual(_inverse([[0.0, 2.0], [1.0, 0.0]]), [[0.0, 1.0], [0.5, 0.0]])


class Setup(unittest.TestCase):
    def make(self, **parameters: bool | float | str | tuple[bool, ...]) -> None:
        self.source = SimulatedTimeSource()
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["ekf"], made["user"] = Ekf(ctx, parameters={"frequency": 10.0, "odom0": "odom", "odom0_config": V_ONLY, "imu0": "imu", **parameters}), Node(ctx, "User")
            return list(made.values())

        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        ekf = made["ekf"]
        assert isinstance(ekf, Ekf)
        self.ekf, self.user = ekf, made["user"]
        self.buffer = Buffer()
        TransformListener(self.buffer, self.user)
        self.filtered: list[Odometry] = []
        self.user.subscribe("odometry/filtered", Odometry, self.filtered.append)

    def tick(self, t: float) -> None:
        self.kros.context.topic_manager.join()
        self.source.set(t)
        self.kros.context.timer_manager.join()
        self.kros.context.topic_manager.join()

    def odom(self, stamp: float, v: float, w: float = 0.0) -> None:
        twist = TwistWithCovariance(Twist(Vector3(x=v), Vector3(z=w)), TWIST_COV)
        self.user.publish("odom", Odometry(Header(stamp, "odom"), "base_footprint", PoseWithCovariance(), twist))

    def drive(self, v: float, seconds: float, start: float = 0.0) -> float:
        t = start
        for _ in range(round(seconds * 10)):
            t += 0.1
            self.odom(t, v)
            self.tick(t)
        return t


class TestEkf(Setup):
    def test_nothing_is_published_before_the_first_measurement(self) -> None:
        self.make()
        self.tick(1.0)
        self.assertEqual(self.filtered, [])

    def test_integrates_wheel_speed_into_position_and_tf(self) -> None:
        self.make()
        self.drive(0.2, 3.0)
        last = self.filtered[-1]
        self.assertAlmostEqual(last.pose.pose.position.x, 0.6, delta=0.05)  # 처음 몇 단계는 속도를 아직 덜 믿는다
        self.assertAlmostEqual(last.twist.twist.linear.x, 0.2, places=3)
        self.assertEqual((last.header.frame_id, last.child_frame_id), ("odom", "base_footprint"))
        t = self.buffer.lookup_transform("odom", "base_footprint").transform
        self.assertAlmostEqual(t.translation.x, last.pose.pose.position.x)
        self.assertGreater(last.pose.covariance[0], 0.0)  # 위치는 직접 재지 않으니 불확실성이 쌓인다

    def test_outlier_is_rejected_by_mahalanobis_distance(self) -> None:
        self.make(odom0_twist_rejection_threshold=3.0)
        t = self.drive(0.2, 2.0)
        before = self.ekf.rejected  # 처음엔 초기 공분산(1e-9)이 작아 0.2m/s도 이상값으로 버린다 (robot_localization과 같다)
        self.odom(t + 0.1, 5.0)  # 한 번 튄 값
        self.tick(t + 0.1)
        self.assertEqual(self.ekf.rejected, before + 1)
        self.assertAlmostEqual(self.ekf.state[3], 0.2, places=2)

    def test_old_measurement_after_newer_ones_is_dropped(self) -> None:
        self.make()
        t = self.drive(0.2, 1.0)
        self.odom(t - 0.5, 9.0)
        self.tick(t + 0.1)
        self.assertEqual(self.ekf.late, 1)
        self.assertAlmostEqual(self.ekf.state[3], 0.2, places=2)

    def test_measurements_in_one_cycle_are_processed_in_time_order(self) -> None:
        self.make()
        self.odom(0.2, 0.2)
        self.odom(0.1, 0.2)  # 늦게 도착했지만 같은 주기 안이면 순서대로 처리한다
        self.tick(0.3)
        self.assertEqual(self.ekf.late, 0)
        self.assertEqual(self.ekf.time, 0.2)

    def test_imu_gyro_and_relative_yaw(self) -> None:
        self.make(odom0="", imu0_config=tuple(a or b for a, b in zip(W_ONLY, YAW_ONLY)), imu0_relative=True)
        cov = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-4)
        for i in range(1, 11):
            yaw = 1.0 + 0.05 * i  # 월드 기준 방향은 1.0에서 시작한다
            self.user.publish("imu", Imu(Header(i * 0.1), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), Vector3(z=0.5), Vector3(), cov, cov, cov))
            self.tick(i * 0.1)
        self.assertAlmostEqual(yaw_of(self.filtered[-1].pose.pose.orientation), 0.45, delta=0.01)  # 첫 방향을 0으로 친다
        self.assertAlmostEqual(self.ekf.state[4], 0.5, places=2)

    def test_tf_can_be_turned_off(self) -> None:
        self.make(publish_tf=False)
        self.drive(0.2, 0.3)
        self.assertTrue(self.filtered)
        self.assertFalse(self.buffer.can_transform("odom", "base_footprint"))


class TestAgainstTheSimulation(unittest.TestCase):
    """노이즈 낀 TurtleBot3: 바퀴만 쓴 오도메트리보다 바퀴 + 자이로를 합친 EKF가 진짜 위치에 더 가깝다 (seed로 고정)."""

    def run_robot(self, seed: int) -> tuple[float, float]:
        source = SimulatedTimeSource()
        world, robot = MazeWorld(puck_count=0), TurtleBot3(noisy=True, seed=seed)
        world.add(robot)
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["watcher"] = Node(ctx, "Watcher")
            ekf = Ekf(ctx, parameters={"odom0": "odom", "odom0_config": tuple(a or b for a, b in zip(V_ONLY, W_ONLY)), "imu0": "imu", "imu0_config": W_ONLY})
            wheels = DiffDriveOdometry(ctx, parameters={"publish_tf": False})
            return [made["watcher"], RobotBridge(ctx, robot), RobotStatePublisher(ctx, parameters={"robot_description": TurtleBot3.urdf_path.read_text()}), wheels, ekf]

        kros = Kros(create, time_source=source)
        self.addCleanup(kros.shutdown)
        odom: list[Odometry] = []
        filtered: list[Odometry] = []
        made["watcher"].subscribe("odom", Odometry, odom.append)
        made["watcher"].subscribe("odometry/filtered", Odometry, filtered.append)
        robot.command = Twist(Vector3(x=0.15), Vector3(z=0.3))
        for _ in range(round(8.0 / 0.02)):
            world.step(0.02)
            source.set(world.time)
            kros.context.timer_manager.join()
        kros.context.topic_manager.join()
        truth = robot.pose

        def error(m: Odometry) -> float:
            return math.hypot(m.pose.pose.position.x - truth.x, m.pose.pose.position.y - truth.y)

        return error(odom[-1]), error(filtered[-1])

    def test_fusing_the_gyro_beats_wheels_alone(self) -> None:
        for seed in (1, 2, 3):
            wheels, fused = self.run_robot(seed)
            self.assertLess(fused, wheels, f"seed {seed}: 바퀴만 {wheels:.3f}m, EKF {fused:.3f}m")
