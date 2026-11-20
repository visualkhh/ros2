import math
import unittest

from kros import Kros
from kros.context import Context
from kros.examples.turtlebot3.robot_bridge import RobotBridge
from kros.msgs.geometry_msgs import Transform, TransformStamped, Vector3
from kros.msgs.sensor_msgs import JointState
from kros.msgs.std_msgs import Header, String
from kros.node import Node
from kros.parameter import ParameterRejectedError
from kros.tf import (
    JOINT_STATES_TOPIC_NAME,
    ROBOT_DESCRIPTION_TOPIC_NAME,
    ExtrapolationException,
    RobotStatePublisher,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformListener,
)
from kros.tf.transform_math import from_axis_angle, rotate
from kros.timer import SimulatedTimeSource
from kros.topic import QOS_LATCHED
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.robots.turtlebot3.turtlebot3 import LIDAR_MOUNT_HEIGHT, LIDAR_MOUNT_X

URDF = TurtleBot3.urdf_path.read_text()


def stamped(parent: str, child: str, stamp: float, x: float = 0.0) -> TransformStamped:
    return TransformStamped(Header(stamp, parent), child, Transform(Vector3(x, 0, 0)))


def settle(kros: Kros) -> None:
    kros.context.timer_manager.join()
    kros.context.topic_manager.join()


class TestBroadcasterAndListener(unittest.TestCase):
    def setUp(self) -> None:
        self.source = SimulatedTimeSource()
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["pub"], made["sub"] = Node(ctx, "Pub"), Node(ctx, "Sub")
            return list(made.values())

        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        self.pub, self.sub = made["pub"], made["sub"]
        self.listener = TransformListener(self.sub)

    def test_moving_transform_reaches_the_listener_buffer(self) -> None:
        TransformBroadcaster(self.pub).send_transform([stamped("odom", "base", 1.0, 1.0), stamped("odom", "base", 2.0, 3.0)])
        settle(self.kros)
        self.assertAlmostEqual(self.listener.buffer.lookup_transform("odom", "base", 1.5).transform.translation.x, 2.0)

    def test_single_transform_is_accepted_too(self) -> None:
        TransformBroadcaster(self.pub).send_transform(stamped("a", "b", 1.0, 5.0))
        settle(self.kros)
        self.assertTrue(self.listener.buffer.can_transform("a", "b"))

    def test_static_transform_reaches_a_listener_that_started_later(self) -> None:
        StaticTransformBroadcaster(self.pub).send_transform(stamped("base_link", "base_scan", 0.0, 0.5))  # 리스너가 뜨기 전에 한 번만 낸다
        late = Node(self.kros.context, "Late")
        late_listener = TransformListener(late)  # 나중에 뜬 노드
        settle(self.kros)
        self.assertEqual(late_listener.buffer.lookup_transform("base_link", "base_scan").transform.translation.x, 0.5)
        self.assertEqual(self.listener.buffer.lookup_transform("base_link", "base_scan").transform.translation.x, 0.5)

    def test_static_and_moving_transforms_chain_together(self) -> None:
        StaticTransformBroadcaster(self.pub).send_transform(stamped("base", "lidar", 0.0, 0.5))
        TransformBroadcaster(self.pub).send_transform(stamped("odom", "base", 4.0, 10.0))
        settle(self.kros)
        self.assertAlmostEqual(self.listener.buffer.lookup_transform("odom", "lidar").transform.translation.x, 10.5)

    def test_bad_transform_is_logged_and_does_not_block_the_rest(self) -> None:
        with self.assertLogs("kros.node.Sub", level="WARNING"):  # 발행도 캡처 안에서 해야 한다 (밖에서 하면 처리가 캡처 시작 전에 끝날 수 있다)
            TransformBroadcaster(self.pub).send_transform([stamped("a", "a", 1.0), stamped("x", "y", 1.0, 2.0)])  # 자기 자신의 자식은 잘못이다
            settle(self.kros)
        self.assertTrue(self.listener.buffer.can_transform("x", "y"))

    def test_empty_send_publishes_nothing(self) -> None:
        TransformBroadcaster(self.pub).send_transform([])
        settle(self.kros)
        self.assertEqual(self.listener.buffer.all_frames(), [])


class TestRobotStatePublisher(unittest.TestCase):
    def setUp(self) -> None:
        self.source = SimulatedTimeSource()
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["state"] = RobotStatePublisher(ctx, parameters={"robot_description": URDF})
            made["user"] = Node(ctx, "User")
            return list(made.values())

        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        state, user = made["state"], made["user"]
        assert isinstance(state, RobotStatePublisher)
        self.state, self.user = state, user
        self.listener = TransformListener(self.user)
        settle(self.kros)

    def joint_states(self, stamp: float, left: float, right: float) -> None:
        self.user.publish(JOINT_STATES_TOPIC_NAME, JointState(Header(stamp), ("wheel_left_joint", "wheel_right_joint"), (left, right)))
        settle(self.kros)

    def test_fixed_joints_become_static_transforms(self) -> None:
        buffer = self.listener.buffer
        self.assertEqual(self.state.robot.root if self.state.robot else None, "base_footprint")
        for frame in ("base_link", "base_scan", "imu_link", "camera_link", "caster_back_left_link"):
            self.assertTrue(buffer.can_transform("base_footprint", frame), frame)

    def test_lidar_position_from_the_urdf_matches_what_the_robot_class_uses(self) -> None:
        """두 출처의 대조: URDF의 관절 값으로 계산한 라이다 위치 == TurtleBot3가 코드에 적어 둔 LIDAR_MOUNT_X, LIDAR_MOUNT_HEIGHT."""
        t = self.listener.buffer.lookup_transform("base_footprint", "base_scan").transform.translation
        self.assertAlmostEqual(t.x, LIDAR_MOUNT_X)
        self.assertAlmostEqual(t.y, 0.0)
        self.assertAlmostEqual(t.z, LIDAR_MOUNT_HEIGHT)

    def test_wheel_axis_is_the_robots_y_axis_and_wheels_are_where_the_urdf_says(self) -> None:
        self.joint_states(1.0, 0.0, 0.0)
        for frame, y in (("wheel_left_link", 0.144), ("wheel_right_link", -0.144)):
            t = self.listener.buffer.lookup_transform("base_link", frame).transform
            self.assertAlmostEqual(t.translation.y, y)
            self.assertAlmostEqual(t.translation.z, 0.023)
            self.assertAlmostEqual(rotate(t.rotation, Vector3(0, 0, 1)).y, 1.0, places=3)  # 회전축이 로봇의 +y

    def test_wheel_rotation_follows_the_joint_state(self) -> None:
        self.joint_states(1.0, 0.0, 0.0)
        self.joint_states(2.0, math.pi / 2, -math.pi / 2)
        wheel = self.listener.buffer.lookup_transform("base_link", "wheel_left_link", 2.0).transform
        rest = self.listener.buffer.lookup_transform("base_link", "wheel_left_link", 1.0).transform
        # 바퀴 둘레의 점이 90도 돌았다: 기준 자세에서의 점 (r, 0, 0)이 로봇 좌표에서 y축(바퀴 축) 둘레로 90도 돈 위치
        point = Vector3(0.033, 0.0, 0.0)
        after, before = rotate(wheel.rotation, point), rotate(rest.rotation, point)
        axis = rotate(rest.rotation, Vector3(0, 0, 1))
        dot = after.x * before.x + after.y * before.y + after.z * before.z
        self.assertAlmostEqual(dot, 0.0, places=3)  # 90도 돌았으니 처음 위치와 직교한다
        self.assertAlmostEqual(after.x * axis.x + after.y * axis.y + after.z * axis.z, before.x * axis.x + before.y * axis.y + before.z * axis.z, places=6)  # 축 방향 성분은 그대로
        right = self.listener.buffer.lookup_transform("base_link", "wheel_right_link", 2.0).transform
        r_after = rotate(right.rotation, point)
        self.assertGreater(abs(r_after.x - after.x) + abs(r_after.z - after.z), 1e-3)  # 오른쪽은 반대로 돌았다

    def test_wheel_interpolates_between_joint_states(self) -> None:
        self.joint_states(1.0, 0.0, 0.0)
        self.joint_states(2.0, math.pi / 2, 0.0)
        quarter = self.listener.buffer.lookup_transform("base_link", "wheel_left_link", 1.5).transform.rotation
        full = self.listener.buffer.lookup_transform("base_link", "wheel_left_link", 2.0).transform.rotation
        rest = self.listener.buffer.lookup_transform("base_link", "wheel_left_link", 1.0).transform.rotation
        for a, b in ((rest, quarter), (quarter, full)):  # 45도씩
            dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
            self.assertAlmostEqual(2 * math.acos(min(1.0, dot)), math.pi / 4, places=3)

    def test_wheels_have_no_transform_until_their_first_joint_state(self) -> None:
        with self.assertRaises(Exception):
            self.listener.buffer.lookup_transform("base_link", "wheel_left_link")
        self.joint_states(1.0, 0.0, 0.0)
        self.assertTrue(self.listener.buffer.can_transform("base_link", "wheel_left_link"))

    def test_unknown_joint_names_are_ignored_and_known_ones_still_work(self) -> None:
        self.user.publish(JOINT_STATES_TOPIC_NAME, JointState(Header(1.0), ("arm_joint", "wheel_left_joint"), (9.0, 0.5)))
        settle(self.kros)
        self.assertTrue(self.listener.buffer.can_transform("base_link", "wheel_left_link"))
        self.assertFalse(self.listener.buffer.can_transform("base_link", "arm_link"))

    def test_mismatched_name_and_position_lengths_are_reported_not_crashed(self) -> None:
        with self.assertLogs("kros.node.RobotStatePublisher", level="ERROR"):  # 발행도 캡처 안에서 (밖에서 하면 처리가 캡처 시작 전에 끝날 수 있다)
            self.user.publish(JOINT_STATES_TOPIC_NAME, JointState(Header(1.0), ("wheel_left_joint", "wheel_right_joint"), (0.1,)))
            settle(self.kros)
        self.assertFalse(self.listener.buffer.can_transform("base_link", "wheel_left_link"))

    def test_robot_description_topic_is_latched_for_late_subscribers(self) -> None:
        got: list[String] = []
        late = Node(self.kros.context, "Late")
        late.subscribe(ROBOT_DESCRIPTION_TOPIC_NAME, String, got.append, QOS_LATCHED)  # URDF가 나간 한참 뒤에 구독
        settle(self.kros)
        self.assertEqual([m.data for m in got], [URDF])

    def test_invalid_urdf_is_rejected_and_the_old_robot_stays(self) -> None:
        with self.assertLogs("kros.tf.robot_state_publisher", level="WARNING"):
            with self.assertRaises(ParameterRejectedError):
                self.kros.set_parameter(self.state, "robot_description", "<robot><link name='a'/><link name='b'/></robot>")  # 뿌리가 둘
        self.assertEqual(self.state.robot.root if self.state.robot else None, "base_footprint")

    def test_changing_robot_description_while_running_publishes_the_new_robot(self) -> None:
        other = '<robot name="arm"><link name="shoulder"/><link name="hand"/><joint name="reach" type="fixed"><parent link="shoulder"/><child link="hand"/><origin xyz="0 0 0.3"/></joint></robot>'
        self.kros.set_parameter(self.state, "robot_description", other)
        settle(self.kros)
        self.assertEqual(self.listener.buffer.lookup_transform("shoulder", "hand").transform.translation.z, 0.3)
        self.assertEqual(self.state.robot.name if self.state.robot else None, "arm")

    def test_prismatic_and_revolute_joints_move_along_and_about_their_axis(self) -> None:
        urdf = (
            '<robot name="r"><link name="base"/><link name="slider"/><link name="arm"/>'
            '<joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/><origin xyz="1 0 0"/><axis xyz="0 1 0"/></joint>'
            '<joint name="swing" type="revolute"><parent link="slider"/><child link="arm"/><axis xyz="0 0 1"/></joint></robot>'
        )
        kros = Kros(lambda ctx: [RobotStatePublisher(ctx, "Arm", {"robot_description": urdf}), Node(ctx, "U")], time_source=SimulatedTimeSource())
        self.addCleanup(kros.shutdown)
        user = kros.nodes[1]
        listener = TransformListener(user)
        user.publish(JOINT_STATES_TOPIC_NAME, JointState(Header(1.0), ("slide", "swing"), (0.4, math.pi / 2)))
        settle(kros)
        slider = listener.buffer.lookup_transform("base", "slider").transform.translation
        self.assertEqual((slider.x, slider.y, slider.z), (1.0, 0.4, 0.0))  # origin + 축(0,1,0)을 따라 0.4
        arm = listener.buffer.lookup_transform("base", "arm").transform
        tip = rotate(arm.rotation, Vector3(1, 0, 0))
        self.assertAlmostEqual(tip.y, 1.0)  # z축으로 90도 돌았다
        self.assertAlmostEqual(arm.rotation.z, from_axis_angle((0, 0, 1), math.pi / 2).z)

    def test_without_a_description_it_publishes_nothing(self) -> None:
        kros = Kros(lambda ctx: [RobotStatePublisher(ctx), Node(ctx, "U")], time_source=SimulatedTimeSource())
        self.addCleanup(kros.shutdown)
        listener = TransformListener(kros.nodes[1])
        settle(kros)
        self.assertEqual(listener.buffer.all_frames(), [])


class TestRobotIntegration(unittest.TestCase):
    """실제 로봇(월드에서 달리는 바퀴) -> RobotBridge의 joint_states -> RobotStatePublisher -> TF."""

    def run_robot(self, seconds: float, v: float) -> tuple[Kros, TransformListener, TurtleBot3, MazeWorld]:
        from kros.msgs.geometry_msgs import Twist

        source = SimulatedTimeSource()
        world, robot = MazeWorld(puck_count=0), TurtleBot3()
        world.add(robot)
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["watcher"] = Node(ctx, "Watcher")
            return [made["watcher"], RobotBridge(ctx, robot), RobotStatePublisher(ctx, parameters={"robot_description": URDF})]

        kros = Kros(create, time_source=source)
        self.addCleanup(kros.shutdown)
        listener = TransformListener(made["watcher"])
        robot.command = Twist(Vector3(v, 0, 0), Vector3())
        for _ in range(round(seconds / 0.05)):
            world.step(0.05)
            source.set(world.time)
            kros.context.timer_manager.join()  # 시간을 sleep 없이 몰아서 밀면 타이머 워커가 못 따라가 주기를 건너뛴다 (missed). 한 스텝마다 기다린다
        settle(kros)
        return kros, listener, robot, world

    def wheel_angle_from_tf(self, listener: TransformListener, time: float) -> float:
        """TF가 알려 주는 바퀴 회전각(기준 자세에서 바퀴 축 둘레로 돈 각)을 구한다. 기준 자세는 joint=0일 때의 변환이다."""
        rest = listener.buffer.lookup_transform("base_link", "wheel_left_link", self.first_time).transform.rotation
        now = listener.buffer.lookup_transform("base_link", "wheel_left_link", time).transform.rotation
        # 상대 회전 = now * rest^-1 의 각도
        from kros.tf.transform_math import quaternion_conjugate, quaternion_multiply

        q = quaternion_multiply(now, quaternion_conjugate(rest))
        return 2 * math.atan2(math.sqrt(q.x**2 + q.y**2 + q.z**2), abs(q.w))

    def test_wheel_frames_follow_the_real_wheel_motors(self) -> None:
        kros, listener, robot, world = self.run_robot(1.0, v=0.1)  # 0.1 m/s / 0.033 m = 3.03 rad/s: 1초에 약 3 rad (한 바퀴 미만이라 접힘 없이 비교한다)
        motor_angle = robot.drive.left.position
        self.assertTrue(2.0 < motor_angle < math.pi, motor_angle)
        latest = listener.buffer.lookup_transform("base_link", "wheel_left_link").header.stamp
        self.first_time = min(t for t in (0.05, 0.1, 0.15) if listener.buffer.can_transform("base_link", "wheel_left_link", t))
        measured = self.wheel_angle_from_tf(listener, latest)
        # TF의 마지막 시각은 모터의 마지막 값보다 최대 한 주기(0.05초 * 3 rad/s = 0.15 rad) 전이고, 첫 시각 이후에 돈 만큼이 빠진다
        self.assertAlmostEqual(measured, motor_angle, delta=0.5)
        self.assertGreater(measured, 1.5)  # 바퀴가 실제로 돌았다는 것이 TF에 보인다

    def test_a_stopped_robot_has_wheels_that_do_not_turn_in_tf(self) -> None:
        kros, listener, robot, world = self.run_robot(1.0, v=0.0)
        self.first_time = 0.05
        latest = listener.buffer.lookup_transform("base_link", "wheel_left_link").header.stamp
        self.assertAlmostEqual(self.wheel_angle_from_tf(listener, latest), 0.0, places=6)
        self.assertEqual(robot.drive.left.position, 0.0)

    def test_the_lidar_frame_matches_the_robot_class_constants(self) -> None:
        kros, listener, robot, world = self.run_robot(0.5, v=0.0)
        t = listener.buffer.lookup_transform("base_footprint", "base_scan").transform.translation
        self.assertAlmostEqual(t.x, LIDAR_MOUNT_X)
        self.assertAlmostEqual(t.z, LIDAR_MOUNT_HEIGHT)

    def test_asking_for_a_time_after_the_latest_is_an_extrapolation_error(self) -> None:
        kros, listener, robot, world = self.run_robot(0.5, v=0.1)
        with self.assertRaises(ExtrapolationException):
            listener.buffer.lookup_transform("base_link", "wheel_left_link", world.time + 5.0)

if __name__ == "__main__":
    unittest.main()
