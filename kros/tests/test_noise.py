import math
import random
import unittest

from kros.hardwares.lidars.lds_01 import SimulatedLds01
from kros.hardwares.noise import Noise
from kros.msgs.geometry_msgs import Pose2D, Twist, Vector3
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.hardwares.imus.mpu9250 import SimulatedMpu9250
from kros.worlds.robots.turtlebot3.turtlebot3 import ACCEL, GYRO, GYRO_CALIBRATION

DT = 1 / 60


def drive(robot: TurtleBot3, seconds: float, command: Twist) -> MazeWorld:
    world = MazeWorld(puck_count=0)
    world.add(robot, Pose2D(5.0, 5.0, 0.0))
    robot.command = command
    for _ in range(round(seconds / DT)):
        world.step(DT)
    return world


class TestNoise(unittest.TestCase):
    def test_noise_formula_and_zero_is_exact(self) -> None:
        rng = random.Random(1)
        self.assertEqual(Noise().apply(2.5, rng), 2.5)
        self.assertAlmostEqual(Noise(bias=0.5, scale=0.1).apply(2.0, rng), 2.7)
        samples = [Noise(sigma=0.2).apply(0.0, rng) for _ in range(4000)]
        self.assertAlmostEqual(sum(samples) / len(samples), 0.0, delta=0.02)
        self.assertAlmostEqual(math.sqrt(sum(x * x for x in samples) / len(samples)), 0.2, delta=0.02)

    def test_default_robot_has_no_error(self) -> None:
        robot = TurtleBot3()
        drive(robot, 2.0, Twist(Vector3(x=0.2), Vector3()))
        self.assertAlmostEqual(robot.pose.x, 5.4, places=2)
        self.assertEqual(robot.pose.y, 5.0)

    def test_same_seed_same_error_different_seed_different(self) -> None:
        def run(seed: int) -> tuple[float, float]:
            robot = TurtleBot3(noisy=True, seed=seed)
            drive(robot, 3.0, Twist(Vector3(x=0.2), Vector3()))
            return robot.pose.x, robot.pose.y

        self.assertEqual(run(1), run(1))
        self.assertNotEqual(run(1), run(2))

    def test_wheel_error_bends_the_path_but_the_encoders_say_straight(self) -> None:
        robot = TurtleBot3(noisy=True, seed=3)
        drive(robot, 5.0, Twist(Vector3(x=0.2), Vector3()))
        left, right = robot.joint_positions()["wheel_left_joint"], robot.joint_positions()["wheel_right_joint"]
        self.assertAlmostEqual(left, right, places=2)  # 엔코더로는 똑바로 갔다
        self.assertGreater(abs(robot.pose.y - 5.0) + abs(robot.pose.theta), 0.005)  # 실제로는 휘었다

    def test_encoder_is_quantized(self) -> None:
        robot = TurtleBot3(noisy=True, seed=1)
        drive(robot, 1.0, Twist(Vector3(x=0.1), Vector3()))
        step = 2 * math.pi / 4096
        for angle in robot.joint_positions().values():
            self.assertAlmostEqual(angle / step, round(angle / step), places=6)

    def test_gyro_calibration_at_boot_keeps_orientation_still_at_rest(self) -> None:
        def yaw_after_rest(calibration: float) -> float:
            robot = TurtleBot3(noisy=True, seed=1)
            assert isinstance(robot.imu, SimulatedMpu9250)
            robot.imu.calibration_time = calibration
            drive(robot, 60.0, Twist())
            data = robot.imu.read()
            assert data is not None
            return 2 * math.atan2(data.orientation.z, data.orientation.w)  # 시작 방향은 0

        self.assertGreater(abs(yaw_after_rest(0)), 0.2)  # 보정 없이: 편향(0.005rad/s)이 1분이면 0.3rad 쌓인다
        self.assertLess(abs(yaw_after_rest(GYRO_CALIBRATION)), math.radians(2))  # OpenCR처럼 켜질 때 보정하면 1분 서 있어도 거의 그대로

    def test_imu_covariance_comes_from_the_noise_and_orientation_grows_honestly(self) -> None:
        robot = TurtleBot3(noisy=True, seed=1)
        drive(robot, 5.0, Twist())
        early = robot.imu.read()
        drive(robot, 15.0, Twist())  # 이어서 (같은 로봇, 새 월드지만 IMU는 계속 쌓는다)
        data = robot.imu.read()
        assert early is not None and data is not None
        samples = round(GYRO_CALIBRATION * 60)  # 보정한 2초 동안의 값 수 (이 테스트는 60Hz로 돈다)
        gyro = GYRO.sigma**2 * (1 + 1 / samples)  # 보정 뒤: 잡음 + 평균의 오차로 남은 편향
        for i, value in enumerate(data.angular_velocity_covariance):
            self.assertAlmostEqual(value, gyro if i % 4 == 0 else 0.0, delta=gyro * 0.02)
        self.assertEqual(data.linear_acceleration_covariance[0], ACCEL.variance)
        self.assertGreater(data.orientation_covariance[8], early.orientation_covariance[8])  # 오래 쌓을수록 덜 믿는다
        yaw = 2 * math.atan2(data.orientation.z, data.orientation.w)
        self.assertLess(abs(yaw), 3 * math.sqrt(data.orientation_covariance[8]))  # 실제로 틀어진 양이 알려 준 크기 안에 있다
        clean = TurtleBot3()
        drive(clean, 1.0, Twist())
        assert (zero := clean.imu.read()) is not None
        self.assertEqual(zero.orientation_covariance + zero.angular_velocity_covariance, (0.0,) * 18)  # 오차 없는 시뮬레이션: ROS의 "모름"

    def test_lidar_noise_and_dropout(self) -> None:
        clean, noisy = TurtleBot3(), TurtleBot3(noisy=True, seed=1)
        assert isinstance(noisy.lidar, SimulatedLds01)
        noisy.lidar.dropout = 0.5  # 확률이 낮아도 시험에서 보이게
        for robot in (clean, noisy):
            drive(robot, 0.5, Twist())
        a, b = clean.lidar.read(), noisy.lidar.read()
        assert a is not None and b is not None
        both = [(x, y) for x, y in zip(a.ranges, b.ranges) if math.isfinite(x) and math.isfinite(y)]
        self.assertTrue(any(x != y for x, y in both))
        self.assertLess(max(abs(x - y) for x, y in both), 0.1)
        self.assertGreater(sum(1 for x, y in zip(a.ranges, b.ranges) if math.isfinite(x) and math.isinf(y)), 0)


class TestLatency(unittest.TestCase):
    def test_delayed_values_are_old_when_they_arrive_and_stay_in_order(self) -> None:
        robot = TurtleBot3()
        robot.imu.latency, robot.imu.jitter, robot.imu.delay_seed = 0.05, 0.02, 1
        arrived: list[tuple[float, float]] = []  # (도착 시각, 측정 시각)
        world = MazeWorld(puck_count=0)
        world.add(robot, Pose2D(5.0, 5.0, 0.0))
        robot.imu.subscribe(lambda m: arrived.append((world.time, m.header.stamp)))
        for _ in range(120):
            world.step(DT)
        self.assertGreater(len(arrived), 50)
        ages = [now - stamp for now, stamp in arrived]
        self.assertGreater(min(ages), 0.0)
        self.assertAlmostEqual(sum(ages) / len(ages), 0.05, delta=0.02)
        stamps = [stamp for _, stamp in arrived]
        self.assertEqual(stamps, sorted(stamps))

    def test_no_latency_delivers_in_the_same_tick(self) -> None:
        robot = TurtleBot3()
        arrived: list[float] = []
        world = MazeWorld(puck_count=0)
        world.add(robot)
        robot.imu.subscribe(lambda m: arrived.append(world.time - m.header.stamp))
        for _ in range(30):
            world.step(DT)
        self.assertEqual(max(arrived), 0.0)

    def test_power_off_drops_values_in_flight(self) -> None:
        robot = TurtleBot3()
        robot.imu.latency = 1.0
        world = MazeWorld(puck_count=0)
        world.add(robot)
        for _ in range(30):
            world.step(DT)
        robot.imu.power_off()
        world.step(DT)
        self.assertIsNone(robot.imu.read())


if __name__ == "__main__":
    unittest.main()
