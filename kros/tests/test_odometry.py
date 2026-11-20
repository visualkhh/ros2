import math
import unittest

from kros import Kros
from kros.context import Context
from kros.examples.turtlebot3.robot_bridge import RobotBridge
from kros.msgs.geometry_msgs import Twist, Vector3
from kros.msgs.nav_msgs import Odometry
from kros.msgs.sensor_msgs import JointState
from kros.msgs.std_msgs import Header
from kros.nodes import DiffDriveOdometry, Node, RobotStatePublisher
from kros.tf import JOINT_STATES_TOPIC_NAME, Buffer, TransformListener
from kros.tf.transform_math import to_rpy
from kros.timer import SimulatedTimeSource
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3

R, SEP = 0.033, 0.288
LEFT, RIGHT = "wheel_left_joint", "wheel_right_joint"


def settle(kros: Kros) -> None:
    kros.context.timer_manager.join()
    kros.context.topic_manager.join()


class Setup(unittest.TestCase):
    def make(self, parameters: dict[str, bool | int | float | str] | None = None) -> None:
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["odom"], made["user"] = DiffDriveOdometry(ctx, parameters=parameters), Node(ctx, "User")
            return list(made.values())

        self.kros = Kros(create, time_source=SimulatedTimeSource())
        self.addCleanup(self.kros.shutdown)
        odom = made["odom"]
        assert isinstance(odom, DiffDriveOdometry)
        self.odom = odom
        self.user = made["user"]
        self.buffer = Buffer()
        TransformListener(self.buffer, self.user)
        self.messages: list[Odometry] = []
        self.user.subscribe("odom", Odometry, self.messages.append)

    def wheels(self, stamp: float, left: float, right: float, names: tuple[str, str] = (LEFT, RIGHT)) -> None:
        self.user.publish(JOINT_STATES_TOPIC_NAME, JointState(Header(stamp), names, (left, right)))
        settle(self.kros)

    def move(self, start: float, left: float, right: float, from_left: float = 0.0, from_right: float = 0.0, seconds: float = 1.0) -> float:
        """바퀴를 (left, right) rad만큼 돌린다: 펌웨어처럼 한 번에 크게 튀지 않게 0.5rad 이하로 잘게 나눠 joint_states를 낸다. 끝 시각을 돌려준다."""
        steps = max(1, math.ceil(max(abs(left), abs(right)) / 0.5))
        for i in range(1, steps + 1):
            self.wheels(start + seconds * i / steps, from_left + left * i / steps, from_right + right * i / steps)
        return start + seconds

    def assertPose(self, expected: tuple[float, float, float]) -> None:
        for got, want in zip(self.pose(), expected):
            self.assertAlmostEqual(got, want, places=6)

    def pose(self) -> tuple[float, float, float]:
        t = self.buffer.lookup_transform("odom", "base_footprint").transform
        return t.translation.x, t.translation.y, to_rpy(t.rotation)[2]


class TestIntegration(Setup):
    def setUp(self) -> None:
        self.make()

    def test_first_joint_states_gives_the_origin(self) -> None:
        self.wheels(1.0, 5.0, 5.0)  # 바퀴 각이 0이 아니어도 시작점이 원점이다
        self.assertEqual(self.pose(), (0.0, 0.0, 0.0))
        self.assertEqual((self.messages[0].header.frame_id, self.messages[0].child_frame_id), ("odom", "base_footprint"))

    def test_both_wheels_forward_moves_straight(self) -> None:
        self.wheels(1.0, 0.0, 0.0)
        self.move(1.0, 10.0, 10.0)
        self.assertPose((R * 10.0, 0.0, 0.0))
        self.assertAlmostEqual(self.messages[-1].twist.linear.x, R * 10.0)  # 1초에 10rad 도는 속도

    def test_opposite_wheels_turn_in_place(self) -> None:
        self.wheels(1.0, 0.0, 0.0)
        self.move(1.0, -2.0, 2.0)
        self.assertPose((0.0, 0.0, R * 4.0 / SEP))  # 오른쪽이 더 가면 왼쪽(반시계)으로 돈다
        self.assertAlmostEqual(self.messages[-1].twist.angular.z, R * 4.0 / SEP)  # 1초에 이만큼 돈다

    def test_a_quarter_turn_then_forward_goes_along_the_new_heading(self) -> None:
        quarter = (math.pi / 2) * SEP / (2 * R)  # 제자리에서 90도 도는 데 필요한 바퀴 각
        self.wheels(1.0, 0.0, 0.0)
        t = self.move(1.0, -quarter, quarter)
        self.move(t, 10.0, 10.0, -quarter, quarter)
        self.assertPose((0.0, R * 10.0, math.pi / 2))

    def test_one_step_uses_the_middle_heading(self) -> None:
        # 한 번에 휘어 가는 조각은 시작 방향이 아니라 중간 방향으로 간 것으로 본다 (그래야 호를 따라간 것과 같다)
        self.wheels(1.0, 0.0, 0.0)
        self.wheels(2.0, 0.0, 1.0)
        distance, turn = R * 0.5, R * 1.0 / SEP
        self.assertPose((distance * math.cos(turn / 2), distance * math.sin(turn / 2), turn))

    def test_an_arc_is_integrated_along_the_arc_not_the_chord(self) -> None:
        # 반지름 1m의 원을 반 바퀴: 지름만큼(2m) 떨어진 곳에 반대 방향으로 선다. 한 번에 반 바퀴를 주면 오차가 크므로 잘게 나눈다
        steps, radius = 2000, 1.0
        total_turn = math.pi
        for i in range(steps + 1):
            f = i / steps
            turn, distance = total_turn * f, radius * total_turn * f
            self.wheels(float(i), (distance - turn * SEP / 2) / R, (distance + turn * SEP / 2) / R)
        x, y, yaw = self.pose()
        self.assertAlmostEqual(x, 0.0, delta=0.01)
        self.assertAlmostEqual(y, 2.0, delta=0.01)
        self.assertAlmostEqual(abs(yaw), math.pi, delta=0.01)

    def test_a_wheel_jump_is_dropped_not_driven(self) -> None:
        self.wheels(1.0, 0.0, 0.0)
        self.wheels(2.0, 0.5, 0.5)
        before = self.pose()
        with self.assertLogs("kros.node.DiffDriveOdometry", level="WARNING"):
            self.user.publish(JOINT_STATES_TOPIC_NAME, JointState(Header(3.0), (LEFT, RIGHT), (50.0, 50.0)))  # 재부팅으로 각이 튄 것처럼
            settle(self.kros)
        self.assertEqual((self.pose(), self.odom.jumps), (before, 1))
        self.wheels(4.0, 50.5, 50.5)  # 튄 뒤의 값을 새 기준으로 이어 간다
        self.assertAlmostEqual(self.pose()[0], before[0] + R * 0.5)

    def test_other_robots_joint_states_are_ignored(self) -> None:
        self.wheels(1.0, 0.0, 0.0, names=("arm_joint_1", "arm_joint_2"))
        self.assertEqual(self.messages, [])
        self.assertFalse(self.buffer.can_transform("odom", "base_footprint"))

    def test_time_going_backwards_does_not_divide_by_zero_or_move(self) -> None:
        self.wheels(2.0, 0.0, 0.0)
        self.wheels(2.0, 0.4, 0.4)  # 같은 시각
        self.assertEqual(self.pose(), (0.0, 0.0, 0.0))


class TestParameters(Setup):
    def test_frames_and_tf_publishing_are_parameters(self) -> None:
        self.make({"odom_frame": "world", "base_frame": "base_link", "publish_tf": False})
        self.wheels(1.0, 0.0, 0.0)
        self.assertEqual((self.messages[0].header.frame_id, self.messages[0].child_frame_id), ("world", "base_link"))
        self.assertFalse(self.buffer.can_transform("world", "base_link"))  # TF는 안 낸다

    def test_wheel_size_changes_the_distance_live(self) -> None:
        self.make()
        self.wheels(1.0, 0.0, 0.0)
        self.odom.set_parameter("wheel_radius", 0.066)
        self.move(1.0, 10.0, 10.0)
        self.assertAlmostEqual(self.pose()[0], 0.66)


class TestAgainstTheSimulation(unittest.TestCase):
    """시뮬레이션이 아는 정답(로봇의 진짜 위치)과 오도메트리의 추정이 같은가: RobotBridge -> joint_states -> DiffDriveOdometry."""

    def drive(self, command: Twist, seconds: float) -> tuple[Odometry, TurtleBot3]:
        source = SimulatedTimeSource()
        world, robot = MazeWorld(puck_count=0), TurtleBot3()
        world.add(robot)
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["watcher"] = Node(ctx, "Watcher")
            return [made["watcher"], RobotBridge(ctx, robot), RobotStatePublisher(ctx, parameters={"robot_description": TurtleBot3.urdf_path.read_text()}), DiffDriveOdometry(ctx)]

        kros = Kros(create, time_source=source)
        self.addCleanup(kros.shutdown)
        got: list[Odometry] = []
        made["watcher"].subscribe("odom", Odometry, got.append)
        robot.command = command
        for _ in range(round(seconds / 0.05)):
            world.step(0.05)
            source.set(world.time)
            kros.context.timer_manager.join()
        settle(kros)
        return got[-1], robot

    def test_odometry_follows_the_true_pose_on_a_curve(self) -> None:
        odom, robot = self.drive(Twist(Vector3(x=0.2), Vector3(z=0.4)), 3.0)
        # 시뮬레이션에서는 바퀴가 미끄러지지 않아서 거의 같다 (joint_states가 20Hz 샘플이라 마지막 한 샘플 이내의 차이)
        self.assertAlmostEqual(odom.pose.position.x, robot.pose.x, delta=0.03)
        self.assertAlmostEqual(odom.pose.position.y, robot.pose.y, delta=0.03)
        self.assertAlmostEqual(to_rpy(odom.pose.orientation)[2], robot.pose.theta, delta=0.05)
        self.assertGreater(math.hypot(odom.pose.position.x, odom.pose.position.y), 0.3)  # 실제로 움직였다

    def test_when_the_robot_is_stopped_by_a_wall_odometry_follows_the_wheels_not_the_wall(self) -> None:
        # 벽에 막히면 몸체는 안 가는데 바퀴 모터는 계속 돈다고 시뮬레이션이 센다: 오도메트리는 바퀴만 보므로 벽을 뚫고 간 것으로 추정한다 (진짜 로봇의 바퀴 슬립과 같은 한계)
        odom, robot = self.drive(Twist(Vector3(x=0.26)), 12.0)  # 시작점 앞 벽까지 1.1m: 12초면 3m를 가려 한다
        self.assertGreater(odom.pose.position.x, robot.pose.x + 1.0)


if __name__ == "__main__":
    unittest.main()
