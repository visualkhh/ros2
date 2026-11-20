import math
import unittest
from unittest import mock

from kros.msgs.geometry_msgs import Pose2D, Twist, Vector3
from kros.msgs.sensor_msgs import LaserScan
from kros.worlds.clock import Clock
from kros.hardwares import Actuator, Hardware, HardwareError, Lidar, Motor, Sensor
from kros.hardwares.lidars.lds_01 import RANGE_MAX, SAMPLES, DummyLds01, SimulatedLds01, make_scan
from kros.hardwares.motors.xl430_w250 import SimulatedXl430W250
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.maze.track import WALL_THICKNESS
from kros.worlds.objects import Puck, PuckShape
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.robots.turtlebot3.turtlebot3 import LIDAR_MOUNT_HEIGHT, LIDAR_MOUNT_X
from kros.worlds.world import World
from kros.worlds.objects.world_object import WorldObject

WORLD = MazeWorld(puck_count=0)
START = WORLD.start_pose


def make() -> tuple[TurtleBot3, MazeWorld]:
    world, robot = MazeWorld(puck_count=0), TurtleBot3()
    world.add(robot)
    return robot, world


def drive(robot: TurtleBot3, v: float = 0.0, w: float = 0.0) -> None:
    robot.command = Twist(Vector3(x=v), Vector3(z=w))


class TestTurtleBot3Body(unittest.TestCase):
    def test_placed_at_world_start(self) -> None:
        robot, _ = make()
        self.assertEqual(robot.pose, START)

    def test_forward(self) -> None:
        robot, world = make()
        drive(robot, v=0.26)
        world.step(1.0)
        self.assertAlmostEqual(robot.pose.x - START.x, 0.26)
        self.assertAlmostEqual(robot.pose.y, START.y)

    def test_turn_in_place(self) -> None:
        robot, world = make()
        drive(robot, w=1.5)
        world.step(1.0)
        self.assertAlmostEqual(robot.pose.theta, 1.5)
        self.assertAlmostEqual(robot.pose.x, START.x)

    def test_no_command_stays(self) -> None:
        robot, world = make()
        world.step(1.0)
        self.assertEqual(robot.pose, START)

    def test_outer_wall_stops_robot(self) -> None:
        robot, world = make()
        robot.pose = Pose2D(0.0, -2.6, 0.0)  # 아래쪽 복도: 동쪽으로 막힘 없이 바깥벽까지
        drive(robot, v=0.26)
        for _ in range(100):
            world.step(1.0)
        self.assertAlmostEqual(robot.pose.x, 2.24 - WALL_THICKNESS / 2 - robot.radius)

    def test_inner_wall_blocks_robot(self) -> None:
        robot, world = make()
        robot.pose = Pose2D(-0.8, -2.6, math.pi / 2)  # 안쪽 벽 (-1.19..-0.41, y=-1.95) 남쪽에서 북쪽으로
        drive(robot, v=0.26)
        for _ in range(20):
            world.step(1.0)
        self.assertLess(robot.pose.y, -1.95)

    def test_theta_wraps(self) -> None:
        robot, world = make()
        drive(robot, w=1.5)
        for _ in range(10):
            world.step(1.0)
        self.assertLessEqual(abs(robot.pose.theta), math.pi)


class TestRobotAndPucks(unittest.TestCase):
    def test_robot_pushes_puck_instead_of_passing_through(self) -> None:
        robot, world = make()
        puck = Puck(PuckShape.CIRCLE, (1, 0, 0))
        world.add(puck, Pose2D(0.4, 0.0, 0.0))  # 로봇 바로 앞
        drive(robot, v=0.26)
        for _ in range(20):
            world.step(0.1)  # 2초: 0.52m 전진
        gap = math.hypot(puck.pose.x - robot.pose.x, puck.pose.y - robot.pose.y)
        self.assertGreater(puck.pose.x, 0.4)  # 앞으로 밀렸다
        self.assertGreaterEqual(gap, robot.radius + puck.radius - 1e-6)  # 겹치지 않는다

    def test_robot_is_not_pushed_back_by_puck(self) -> None:
        robot, world = make()
        world.add(Puck(PuckShape.CIRCLE, (1, 0, 0)), Pose2D(0.4, 0.0, 0.0))
        drive(robot, v=0.26)
        for _ in range(20):
            world.step(0.1)
        self.assertAlmostEqual(robot.pose.x, 0.26 * 2.0, places=3)  # 퍽을 밀면서도 속도는 그대로

    def test_robot_ignores_scan_of_its_own_body(self) -> None:
        robot, world = make()
        scans: list[LaserScan] = []
        robot.lidar.subscribe(scans.append)
        world.step(0.0)
        self.assertTrue(all(r > 0.5 for r in scans[0].ranges))  # 자기 몸체(반지름 0.2)가 가리지 않는다

    def test_lidar_passes_over_low_puck_but_sees_tall_object(self) -> None:
        robot, world = make()
        scans: list[LaserScan] = []
        robot.lidar.subscribe(scans.append)
        puck = Puck(PuckShape.CIRCLE, (1, 0, 0))  # 높이 0.04m < 라이다 0.132m
        world.add(puck, Pose2D(0.6, 0.0, 0.0))
        world.step(0.0)
        self.assertAlmostEqual(scans[0].ranges[0], 1.145 - (START.x + LIDAR_MOUNT_X))  # 퍽을 지나 벽까지
        tall = Puck(PuckShape.CIRCLE, (0, 1, 0), height=0.3)
        world.add(tall, Pose2D(0.6, 0.0, 0.0))
        world.step(0.2)
        self.assertAlmostEqual(scans[1].ranges[0], 0.6 - tall.radius - (START.x + LIDAR_MOUNT_X), places=2)  # 센서는 바퀴축 뒤에 있다


def inside_map(p: Puck) -> bool:
    return -2.215 < p.pose.x < 2.215 and -3.175 < p.pose.y < 3.145  # 바깥벽 안쪽 (벽 두께 반 + 퍽 반지름 여유 포함)


class TestPushingPuckIntoWalls(unittest.TestCase):
    def push(self, v: float, dt: float, steps: int, robot_at: tuple[float, float], puck_at: tuple[float, float], heading: float) -> tuple[TurtleBot3, Puck]:
        robot, world = make()
        robot.pose = Pose2D(*robot_at, heading)
        puck = Puck(PuckShape.CIRCLE, (1, 0, 0))
        world.add(puck, Pose2D(*puck_at, 0.0))
        drive(robot, v=v)
        for _ in range(steps):
            world.step(dt)
            self.assertTrue(inside_map(puck), f"퍽이 지도 밖으로: {puck.pose}")
            self.assertTrue(-2.215 < robot.pose.x < 2.215 and -3.175 < robot.pose.y < 3.145, f"로봇이 지도 밖으로: {robot.pose}")
        return robot, puck

    def test_keep_pushing_puck_against_a_wall(self) -> None:
        _, puck = self.push(0.26, 1 / 60, 600, (1.2, -2.6), (1.7, -2.6), 0.0)
        self.assertEqual(puck.velocity, (0.0, 0.0))  # 벽에 눌려 가만히 있다 (속도가 쌓이지 않는다)

    def test_keep_pushing_puck_into_a_corner(self) -> None:
        self.push(0.26, 1 / 60, 900, (1.6, -2.55), (2.0, -2.95), -0.7)

    def test_fast_robot_big_dt_does_not_throw_puck_through_wall(self) -> None:
        self.push(1.04, 0.1, 60, (1.6, -2.55), (2.0, -2.95), -0.7)
        self.push(1.04, 0.1, 60, (1.2, -2.6), (1.7, -2.6), 0.0)

    def test_robot_yields_when_puck_is_pinned(self) -> None:
        robot, puck = self.push(0.26, 1 / 60, 600, (1.2, -2.6), (1.7, -2.6), 0.0)
        self.assertGreaterEqual(math.hypot(puck.pose.x - robot.pose.x, puck.pose.y - robot.pose.y), robot.radius + puck.radius - 1e-6)

    def test_driving_around_with_ten_pucks_keeps_all_inside(self) -> None:
        world, robot = MazeWorld(seed=5), TurtleBot3()
        world.add(robot)
        pucks = [o for o in world.objects if isinstance(o, Puck)]
        for i in range(1200):  # 20초: 직진하다 주기적으로 돈다
            drive(robot, v=0.26, w=1.2 if (i // 120) % 3 == 0 else 0.0)
            world.step(1 / 60)
        for puck in pucks:
            self.assertTrue(inside_map(puck), puck.pose)


class RecordingHardware(Hardware):
    def __init__(self, name: str, log: list[str]) -> None:
        super().__init__()
        self.name, self.log = name, log

    def tick(self, clock: Clock, world: World) -> None:
        self.log.append(self.name)
        super().tick(clock, world)


class TestHardware(unittest.TestCase):
    def test_motor_lidar_and_robot_are_all_hardware(self) -> None:
        robot = TurtleBot3()
        self.assertIsInstance(robot, Hardware)
        self.assertIsInstance(robot.drive, Hardware)
        self.assertIsInstance(robot.drive.left, Hardware)
        self.assertIsInstance(robot.lidar, Hardware)
        self.assertIsInstance(robot, WorldObject)  # 로봇은 월드에 놓이는 객체이기도 하다
        self.assertEqual(robot.parts, [robot.drive, robot.lidar])
        self.assertEqual(robot.drive.parts, [robot.drive.left, robot.drive.right])  # 구동부의 부품은 바퀴 모터 둘
        self.assertIs(robot.drive.owner, robot)
        self.assertIs(robot.drive.left.owner, robot.drive)
        self.assertIs(robot.lidar.owner, robot)

    def test_tick_goes_down_the_tree_in_order(self) -> None:
        log: list[str] = []
        top = RecordingHardware("top", log)
        a, b = RecordingHardware("a", log), RecordingHardware("b", log)
        top.add(a)
        a.add(RecordingHardware("a1", log))
        top.add(b)
        top.tick(Clock(0.1, 0.1), WORLD)
        self.assertEqual(log, ["top", "a", "a1", "b"])

    def test_world_step_ticks_robot_parts(self) -> None:
        robot, world = make()
        log: list[str] = []
        extra = RecordingHardware("extra", log)
        robot.add(extra)  # 부품을 더 달면 같은 신호를 받는다
        world.step(0.1)
        self.assertEqual(log, ["extra"])
        self.assertAlmostEqual(robot.pose.x, START.x)  # 명령이 없으니 모터는 가만히


class CountingSensor(Sensor[int]):
    def __init__(self, period: float | None = None) -> None:
        super().__init__(period)
        self.samples = 0

    def sample(self, clock: Clock, world: World) -> int | None:
        self.samples += 1
        return self.samples


class RecordingActuator(Actuator[str]):
    def __init__(self, period: float | None = None) -> None:
        super().__init__("idle", period)
        self.applied: list[tuple[str, float]] = []

    def apply(self, command: str, clock: Clock, world: World) -> None:
        self.applied.append((command, round(clock.dt, 6)))


class TestHardwareInterface(unittest.TestCase):
    def test_each_hardware_has_its_own_period(self) -> None:
        robot, world = make()
        fast, slow, every = CountingSensor(0.05), CountingSensor(0.5), CountingSensor()
        for h in (fast, slow, every):
            robot.add(h)
        for _ in range(100):  # 0.01초 x 100 = 1.0초
            world.step(0.01)
        self.assertEqual(every.samples, 100)  # period=None: 매 시계 신호
        self.assertEqual(fast.samples, 21)  # 첫 신호(0.01)에 한 번, 이후 0.05초마다 (0.05, 0.10, ... 1.00)
        self.assertEqual(slow.samples, 3)  # 첫 신호 0.01, 0.5, 1.0

    def test_sensor_read_and_subscribe(self) -> None:
        robot, world = make()
        sensor = CountingSensor(0.1)
        robot.add(sensor)
        seen: list[int] = []
        sensor.subscribe(seen.append)
        self.assertIsNone(sensor.read())  # 아직 값이 없다
        for _ in range(30):
            world.step(0.01)
        self.assertEqual(seen, [1, 2, 3, 4])  # 0.01(첫 신호), 0.1, 0.2, 0.3
        self.assertEqual(sensor.read(), 4)

    def test_sensor_sample_hook_can_skip_a_period(self) -> None:
        class Sometimes(Sensor[int]):
            def sample(self, clock: Clock, world: World) -> int | None:
                return None  # 이번엔 데이터 없음

        robot, world = make()
        sensor = Sometimes(0.1)
        robot.add(sensor)
        seen: list[int] = []
        sensor.subscribe(seen.append)
        world.step(0.2)
        self.assertEqual((seen, sensor.read()), ([], None))

    def test_actuator_write_then_apply_hook_with_elapsed_time(self) -> None:
        robot, world = make()
        actuator = RecordingActuator(0.1)
        robot.add(actuator)
        actuator.write("go")
        for _ in range(30):
            world.step(0.01)
        self.assertEqual(actuator.applied, [("go", 0.01), ("go", 0.09), ("go", 0.1), ("go", 0.1)])  # dt는 지난 호출 이후 지난 시간: 0.01, 0.1, 0.2, 0.3에 불림

    def test_motor_with_its_own_period_still_moves_the_right_distance(self) -> None:
        world, robot = MazeWorld(puck_count=0), TurtleBot3(left_motor=SimulatedXl430W250(period=0.1), right_motor=SimulatedXl430W250(period=0.1))
        world.add(robot)
        drive(robot, v=0.2)
        for _ in range(100):
            world.step(0.01)
        self.assertAlmostEqual(robot.pose.x, 0.2 * 1.0, delta=0.025)  # 주기 사이의 시간만큼 이동 (마지막 주기 끝 오차 이내)

    def test_dummy_lidar_replaces_simulation_without_touching_the_world(self) -> None:
        robot = TurtleBot3(lidar=DummyLds01(distance=2.0))
        world = MazeWorld(puck_count=0)
        world.add(robot)
        scans: list[LaserScan] = []
        robot.lidar.subscribe(scans.append)
        with mock.patch.object(World, "ray_distance", side_effect=AssertionError("더미는 월드에 레이를 쏘지 않는다")):
            world.step(0.0)
            world.step(0.2)
        self.assertEqual(len(scans), 2)
        self.assertEqual(set(scans[0].ranges), {2.0})
        self.assertIsInstance(robot.lidar, DummyLds01)
        self.assertNotIsInstance(robot.lidar, SimulatedLds01)

    def test_default_hardware_is_simulated(self) -> None:
        robot = TurtleBot3()
        self.assertIsInstance(robot.lidar, SimulatedLds01)
        self.assertIsInstance(robot.drive.left, SimulatedXl430W250)
        self.assertIsInstance(robot.drive.right, SimulatedXl430W250)
        self.assertIsInstance(robot.lidar, Lidar)  # 인터페이스를 만족한다
        self.assertIsInstance(robot.drive.left, Motor)


class FailingSensor(Sensor[int]):
    def __init__(self, period: float | None = None) -> None:
        super().__init__(period)
        self.broken = False

    def sample(self, clock: Clock, world: World) -> int | None:
        if self.broken:
            raise HardwareError("타임아웃")
        return int(clock.time * 100)


class TestHardwarePowerAndFailure(unittest.TestCase):
    def test_power_off_stops_the_hardware_and_on_resumes(self) -> None:
        robot, world = make()
        sensor = CountingSensor(0.1)
        robot.add(sensor)
        sensor.power_off()
        for _ in range(30):
            world.step(0.01)
        self.assertEqual(sensor.samples, 0)
        self.assertFalse(sensor.is_on)
        sensor.power_on()
        world.step(0.01)
        self.assertEqual(sensor.samples, 1)

    def test_powering_off_the_robot_powers_off_all_its_parts(self) -> None:
        robot, world = make()
        robot.power_off()
        self.assertFalse(robot.drive.is_on)
        self.assertFalse(robot.drive.left.is_on)  # 부모가 꺼지면 바퀴 모터도 꺼진다
        self.assertFalse(robot.lidar.is_on)
        scans: list[LaserScan] = []
        robot.lidar.subscribe(scans.append)
        drive(robot, v=0.26)
        for _ in range(20):
            world.step(0.1)
        self.assertEqual(scans, [])  # 센서도 안 나오고
        self.assertEqual(robot.pose, START)  # 모터도 안 움직인다
        robot.power_on()
        world.step(0.1)
        self.assertEqual(len(scans), 1)

    def test_motor_off_then_on_does_not_jump_by_the_time_it_was_off(self) -> None:
        robot, world = make()
        drive(robot, v=0.26)
        robot.drive.power_off()
        for _ in range(50):  # 5초 동안 꺼져 있다
            world.step(0.1)
        robot.drive.power_on()
        world.step(0.1)
        self.assertAlmostEqual(robot.pose.x, 0.026, places=3)  # 켠 뒤 0.1초만큼만 움직인다 (5초치가 한꺼번에 가지 않는다)

    def test_read_failure_is_recorded_and_data_goes_stale(self) -> None:
        robot, world = make()
        sensor = FailingSensor(0.1)
        robot.add(sensor)
        seen: list[int] = []
        sensor.subscribe(seen.append)
        for _ in range(20):
            world.step(0.01)
        good_stamp, good_value = sensor.stamp, sensor.read()
        self.assertEqual((sensor.error_count, sensor.last_error), (0, None))
        sensor.broken = True
        for _ in range(30):
            world.step(0.01)
        self.assertEqual(sensor.error_count, 3)  # 실패한 읽기가 기록된다
        self.assertEqual(str(sensor.last_error), "타임아웃")
        self.assertEqual(sensor.read(), good_value)  # 새 값은 없고 옛 값이 그대로 남는다
        assert sensor.stamp is not None and good_stamp is not None
        self.assertEqual(sensor.stamp, good_stamp)  # 그래서 쓰는 쪽이 (지금 - stamp)로 오래됐음을 안다
        self.assertGreater(world.time - sensor.stamp, 0.25)
        self.assertEqual(len(seen), 3)  # 구독자는 실패한 읽기에 불리지 않는다
        sensor.broken = False
        world.step(0.1)
        assert sensor.stamp is not None
        self.assertGreater(sensor.stamp, good_stamp)  # 회복하면 다시 새 값
        self.assertEqual(sensor.error_count, 3)  # 실패 기록은 누적으로 남는다

    def test_other_exceptions_are_bugs_and_propagate(self) -> None:
        class Buggy(Sensor[int]):
            def sample(self, clock: Clock, world: World) -> int | None:
                raise ZeroDivisionError

        robot, world = make()
        robot.add(Buggy(0.1))
        with self.assertRaises(ZeroDivisionError):
            world.step(0.1)

    def test_injected_fault_makes_reads_fail_like_real_hardware(self) -> None:
        robot, world = make()
        scans: list[LaserScan] = []
        robot.lidar.subscribe(scans.append)
        robot.lidar.fault = lambda clock: 0.35 <= clock.time <= 0.75  # 이 사이엔 통신이 끊겼다
        for _ in range(20):
            world.step(0.05)  # 0.05 ~ 1.0
        # 측정 시각은 0.05(첫 신호), 0.2, 0.4, 0.6, 0.8, 1.0. 고장 구간(0.35~0.75)의 0.4와 0.6만 실패한다
        self.assertEqual(len(scans), 4)
        self.assertEqual(robot.lidar.error_count, 2)
        assert robot.lidar.stamp is not None
        self.assertAlmostEqual(robot.lidar.stamp, 1.0)  # 마지막 성공은 1.0

    def test_actuator_failure_is_recorded_and_command_is_not_executed(self) -> None:
        class DeadMotor(Actuator[str]):
            def apply(self, command: str, clock: Clock, world: World) -> None:
                raise HardwareError("모터 응답 없음")

        robot, world = make()
        motor = DeadMotor("idle")
        robot.add(motor)
        motor.write("go")
        world.step(0.1)
        world.step(0.1)
        self.assertEqual(motor.error_count, 2)
        self.assertEqual(motor.command, "go")  # 명령은 받은 그대로 남아 있다 (실행을 못 했을 뿐)


class TestMotor(unittest.TestCase):
    def tick(self, motor: SimulatedXl430W250, dt: float, n: int) -> None:
        for i in range(n):
            motor.tick(Clock((i + 1) * dt, dt), WORLD)

    def test_single_motor_integrates_position_at_commanded_velocity(self) -> None:
        motor = SimulatedXl430W250()
        motor.write(2.0)  # rad/s
        self.tick(motor, 0.01, 100)
        self.assertAlmostEqual(motor.velocity, 2.0)
        self.assertAlmostEqual(motor.position, 2.0)  # 1초 x 2 rad/s

    def test_motor_velocity_is_limited(self) -> None:
        motor = SimulatedXl430W250(max_velocity=1.5)
        motor.write(10.0)
        self.tick(motor, 0.01, 100)
        self.assertAlmostEqual(motor.velocity, 1.5)
        motor.write(-10.0)
        self.tick(motor, 0.01, 1)
        self.assertAlmostEqual(motor.velocity, -1.5)

    def test_powered_off_motor_stops(self) -> None:
        motor = SimulatedXl430W250()
        motor.write(2.0)
        self.tick(motor, 0.01, 10)
        motor.power_off()
        self.tick(motor, 0.01, 10)
        self.assertEqual(motor.velocity, 0.0)

    def test_robot_command_goes_to_drive(self) -> None:
        robot = TurtleBot3()
        drive(robot, v=0.1)
        self.assertEqual(robot.drive.command.linear.x, 0.1)


class TestDiffDrive(unittest.TestCase):
    def test_inverse_kinematics_gives_wheel_speeds(self) -> None:
        robot, world = make()
        drive(robot, v=0.2)
        world.step(0.01)
        wheel = 0.2 / 0.033  # 직진: 두 바퀴가 같은 속도
        self.assertAlmostEqual(robot.drive.left.velocity, wheel)
        self.assertAlmostEqual(robot.drive.right.velocity, wheel)
        drive(robot, w=1.5)
        world.step(0.01)
        spin = 1.5 * 0.144 / 0.033  # 제자리 회전: 두 바퀴가 반대 방향
        self.assertAlmostEqual(robot.drive.left.velocity, -spin)
        self.assertAlmostEqual(robot.drive.right.velocity, spin)

    def test_body_follows_what_the_wheels_really_did(self) -> None:
        robot, world = make()
        drive(robot, v=0.2)
        for _ in range(100):
            world.step(0.01)
        self.assertAlmostEqual(robot.pose.x, 0.2, places=3)
        # 바퀴 위치로 계산한 이동 거리와 같다 (바퀴 반지름 x 회전각)
        self.assertAlmostEqual(robot.drive.left.position * 0.033, 0.2, places=3)

    def test_slower_wheel_motors_limit_the_robot(self) -> None:
        world, robot = MazeWorld(puck_count=0), TurtleBot3(left_motor=SimulatedXl430W250(max_velocity=3.0), right_motor=SimulatedXl430W250(max_velocity=3.0))
        world.add(robot)
        drive(robot, v=0.26)  # 명령은 0.26 m/s지만 바퀴가 3 rad/s까지만 돈다
        for _ in range(100):
            world.step(0.01)
        self.assertAlmostEqual(robot.pose.x, 3.0 * 0.033, places=3)  # 1초 동안 0.099 m

    def test_one_wheel_off_makes_the_robot_curve(self) -> None:
        robot, world = make()
        robot.drive.left.power_off()  # 왼쪽 바퀴 모터만 꺼졌다
        drive(robot, v=0.2)
        for _ in range(100):
            world.step(0.01)
        self.assertGreater(robot.pose.theta, 0.1)  # 오른쪽 바퀴만 돌아 왼쪽으로 휜다
        self.assertGreater(robot.pose.x, 0.02)


class TestLidar(unittest.TestCase):
    def test_shape_and_ros_conventions(self) -> None:
        scan = make_scan(Pose2D(0.0, 0.0, 0.0), 0.132, WORLD)
        self.assertEqual(len(scan.ranges), SAMPLES)
        self.assertAlmostEqual(scan.angle_min + (SAMPLES - 1) * scan.angle_increment, scan.angle_max)

    def test_ranges_follow_sensor_heading(self) -> None:
        scan = make_scan(Pose2D(0.0, 0.0, math.pi / 2), 0.132, WORLD)  # 북쪽(+y)을 보고 있음
        self.assertAlmostEqual(scan.ranges[0], 1.785)  # 앞 = 북쪽 안쪽 벽
        self.assertAlmostEqual(scan.ranges[90], 2.215)  # 왼쪽 = 서쪽 바깥벽
        self.assertAlmostEqual(scan.ranges[270], 1.145)  # 오른쪽 = 동쪽 안쪽 벽

    def test_out_of_range_is_inf(self) -> None:
        scan = make_scan(Pose2D(-2.0, -3.0, 0.0), 0.132, WORLD)  # 동쪽 벽까지 4.2m > RANGE_MAX
        self.assertGreater(WORLD.ray_distance(-2.0, -3.0, 0.0), RANGE_MAX)
        self.assertEqual(scan.ranges[0], math.inf)
        self.assertAlmostEqual(scan.ranges[180], 0.215)  # 서쪽 벽
        self.assertEqual(make_scan(Pose2D(-2.2, -3.0, 0.0), 0.132, WORLD).ranges[180], math.inf)  # 0.015m: RANGE_MIN 미만

    def test_sensor_is_behind_wheel_axis(self) -> None:
        lidar = TurtleBot3().lidar
        self.assertEqual((lidar.mount_x, lidar.mount_height), (LIDAR_MOUNT_X, LIDAR_MOUNT_HEIGHT))
        self.assertAlmostEqual(lidar.sensor_pose(Pose2D(2.0, 2.0, 0.0)).x, 2.0 + LIDAR_MOUNT_X)
        self.assertAlmostEqual(lidar.sensor_pose(Pose2D(2.0, 2.0, math.pi / 2)).y, 2.0 + LIDAR_MOUNT_X)

    def test_scans_at_5hz_of_sim_time(self) -> None:
        robot, world = make()
        scans: list[LaserScan] = []
        robot.lidar.subscribe(scans.append)
        for _ in range(20):  # 0.05초 간격으로 1.0초
            world.step(0.05)
        self.assertEqual(len(scans), 6)  # 첫 신호(0.05초)에 바로 한 번, 이후 0.2, 0.4, 0.6, 0.8, 1.0
        self.assertIsNotNone(robot.lidar.read())

    def test_big_dt_scans_once(self) -> None:
        robot, world = make()
        scans: list[LaserScan] = []
        robot.lidar.subscribe(scans.append)
        world.step(0.0)
        world.step(1.0)  # 5주기를 건너뛰어도 한 번만
        world.step(0.05)
        self.assertEqual(len(scans), 2)

    def test_world_tick_makes_lidar_scan_via_callback(self) -> None:
        robot, world = make()
        scans: list[LaserScan] = []
        robot.lidar.subscribe(scans.append)
        world.step(0.0)
        self.assertEqual(len(scans), 1)
        self.assertAlmostEqual(scans[0].ranges[0], 1.145 - (START.x + LIDAR_MOUNT_X))
        world.step(0.05)
        self.assertEqual(len(scans), 1)  # 다음 스캔은 0.2초 뒤


if __name__ == "__main__":
    unittest.main()
