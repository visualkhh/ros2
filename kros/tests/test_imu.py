import math
import unittest

from kros import Kros
from kros.hardwares import Imu
from kros.hardwares.controllers.diff_drive_board import DiffDriveBoard
from kros.hardwares.controllers.opencr import OpenCr
from kros.hardwares.motors.xl430_w250 import SimulatedXl430W250
from kros.hardwares.imus.mpu9250 import DummyMpu9250, SimulatedMpu9250
from kros.msgs.geometry_msgs import Pose2D, Twist, Vector3
from kros.msgs.sensor_msgs import Imu as ImuMessage
from kros.nodes import Node
from kros.timer import SimulatedTimeSource
from kros.worlds.clock import Clock
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.world import EARTH_GRAVITY

DT = 1 / 60


def run(robot: TurtleBot3, world: MazeWorld, seconds: float) -> None:
    for _ in range(round(seconds / DT)):
        world.step(DT)


def make(pose: Pose2D = Pose2D(0.0, 0.0, 0.0), gravity: float = EARTH_GRAVITY) -> tuple[MazeWorld, TurtleBot3]:
    world, robot = MazeWorld(puck_count=0, gravity=gravity), TurtleBot3()
    world.add(robot, pose)
    return world, robot


def latest(robot: TurtleBot3) -> ImuMessage:
    data = robot.imu.read()
    assert data is not None
    return data


class TestSimulatedImu(unittest.TestCase):
    def test_robot_has_an_imu_that_is_hardware_of_the_mpu9250_model(self) -> None:
        robot = TurtleBot3()
        self.assertIsInstance(robot.imu, Imu)
        self.assertEqual((robot.imu.model, robot.imu.name, robot.imu.frame_id), ("MPU-9250", "imu", "imu_link"))

    def test_at_rest_it_feels_only_gravity(self) -> None:
        world, robot = make()
        run(robot, world, 0.5)
        m = latest(robot)
        self.assertEqual((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z), (0.0, 0.0, 0.0))
        self.assertEqual((m.linear_acceleration.x, m.linear_acceleration.y), (0.0, 0.0))
        self.assertAlmostEqual(m.linear_acceleration.z, world.gravity)  # 달 월드면 1.62다
        self.assertEqual(m.header.frame_id, "imu_link")

    def test_gravity_follows_the_world(self) -> None:
        world, robot = make(gravity=1.62)
        run(robot, world, 0.2)
        self.assertAlmostEqual(latest(robot).linear_acceleration.z, 1.62)

    def test_spinning_gives_yaw_rate(self) -> None:
        world, robot = make()
        robot.command = Twist(Vector3(), Vector3(z=1.0))
        run(robot, world, 2.0)
        self.assertAlmostEqual(latest(robot).angular_velocity.z, 1.0, places=1)
        robot.command = Twist(Vector3(), Vector3(z=-0.5))
        run(robot, world, 2.0)
        self.assertAlmostEqual(latest(robot).angular_velocity.z, -0.5, places=1)

    def test_yaw_rate_is_right_across_the_pi_wrap(self) -> None:
        world, robot = make(Pose2D(0.0, 0.0, math.pi - 0.02))
        robot.command = Twist(Vector3(), Vector3(z=1.0))
        rates: list[float] = []
        robot.imu.subscribe(lambda m: rates.append(m.angular_velocity.z))
        run(robot, world, 1.0)
        self.assertLess(max(abs(r - 1.0) for r in rates[5:]), 0.15)  # 각이 -pi로 넘어가는 순간에도 튀지 않는다

    def test_accelerating_forward_feels_it_along_its_own_x_axis_even_when_turned(self) -> None:
        for heading in (0.0, math.pi / 2, -2.0):
            world, robot = make(Pose2D(0.0, 0.0, heading))
            samples: list[Vector3] = []
            run(robot, world, 0.2)  # 먼저 가만히 서서 첫 값을 낸다 (변화율은 연속된 값이 있어야 구한다)
            robot.imu.subscribe(lambda m: samples.append(m.linear_acceleration))
            robot.command = Twist(Vector3(x=0.2), Vector3())
            run(robot, world, 0.6)
            peak = max(samples, key=lambda a: math.hypot(a.x, a.y))
            self.assertGreater(peak.x, 0.5, heading)  # 앞으로 가속: 센서 x가 +
            self.assertLess(abs(peak.y), 0.2 * peak.x, heading)

    def test_orientation_is_the_yaw_quaternion(self) -> None:
        world, robot = make(Pose2D(0.0, 0.0, math.pi / 2))
        run(robot, world, 0.1)
        q = latest(robot).orientation
        self.assertAlmostEqual(q.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(q.w, math.cos(math.pi / 4))

    def test_after_a_power_gap_it_does_not_report_a_fake_acceleration(self) -> None:
        world, robot = make()
        robot.command = Twist(Vector3(x=0.2), Vector3())
        run(robot, world, 1.0)  # 일정한 속도
        robot.imu.power_off()
        run(robot, world, 1.0)
        robot.imu.power_on()
        accelerations: list[Vector3] = []
        robot.imu.subscribe(lambda m: accelerations.append(m.linear_acceleration))
        run(robot, world, 0.3)
        self.assertLess(max(math.hypot(a.x, a.y) for a in accelerations), 0.5)  # 꺼진 동안의 이동이 가속도로 보이지 않는다

    def test_read_failures_are_recorded_like_any_sensor(self) -> None:
        world, robot = make()
        robot.imu.fault = lambda clock: clock.time > 0.2
        run(robot, world, 0.5)
        self.assertGreater(robot.imu.error_count, 0)


class DummyImuBot(TurtleBot3):
    def make_opencr(self, serial_port: str | None) -> DiffDriveBoard:
        return OpenCr(SimulatedXl430W250(), SimulatedXl430W250(), DummyMpu9250(), 0.033, 0.288, self.radius)


class TestDummyAndBridge(unittest.TestCase):
    def test_dummy_imu_reports_a_still_sensor_without_a_world_object(self) -> None:
        robot = DummyImuBot()
        world = MazeWorld(puck_count=0)
        world.add(robot)
        run(robot, world, 0.1)
        self.assertEqual(latest(robot).linear_acceleration, Vector3(0.0, 0.0, 9.80665))

    def test_bridge_publishes_imu_topic(self) -> None:
        from kros.examples.turtlebot3.robot_bridge import RobotBridge

        world, robot = make()
        source = SimulatedTimeSource()
        made: dict[str, Node] = {}
        kros = Kros(lambda ctx: [RobotBridge(ctx, robot), made.setdefault("watcher", Node(ctx, "Watcher"))], time_source=source)
        self.addCleanup(kros.shutdown)
        got: list[ImuMessage] = []
        made["watcher"].subscribe("imu", ImuMessage, got.append)
        run(robot, world, 0.2)
        kros.context.topic_executor.join()
        self.assertTrue(got)
        self.assertEqual(got[-1].header.frame_id, "imu_link")

    def test_simulated_mpu_needs_a_world_object_owner(self) -> None:
        with self.assertRaises(AssertionError):
            SimulatedMpu9250().sample(Clock(0.0, DT))


if __name__ == "__main__":
    unittest.main()
