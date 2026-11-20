import unittest
from typing import Any

from kros.context import Context
from kros.examples.turtlebot3.robot_bridge import RobotBridge
from kros.examples.turtlebot3.topics import CMD_VEL_TOPIC_NAME, SCAN_TOPIC_NAME
from kros.nodes import Node
from kros.msgs.geometry_msgs import Twist, Vector3
from kros.msgs.sensor_msgs import LaserScan
from kros.interactions import Interaction, Keyboard, KeyState
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.robot import Robot
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.robots.turtlebot3.turtlebot3 import ANGULAR_SPEED, LINEAR_SPEED


class TestInteraction(unittest.TestCase):
    def test_feed_updates_state_and_notifies_subscribers(self) -> None:
        class Dial(Interaction[int]):
            pass

        dial = Dial()
        seen: list[int] = []
        dial.subscribe(seen.append)
        self.assertIsNone(dial.state)
        dial.feed(3)
        dial.feed(5)
        self.assertEqual((dial.state, seen), (5, [3, 5]))

    def test_keyboard_feeds_only_when_the_pressed_keys_change(self) -> None:
        keyboard = Keyboard()
        seen: list[KeyState] = []
        keyboard.subscribe(seen.append)
        keyboard.set_keys({"w"})
        keyboard.set_keys({"w"})  # 같은 키: 새 입력이 아니다
        keyboard.set_keys({"w", "a"})
        keyboard.set_keys(set())
        self.assertEqual([s.keys for s in seen], [frozenset({"w"}), frozenset({"w", "a"}), frozenset()])


class TestRobotReceivesInteraction(unittest.TestCase):
    def setUp(self) -> None:
        self.world, self.robot, self.keyboard = MazeWorld(puck_count=0), TurtleBot3(), Keyboard()
        self.world.add(self.robot)
        self.robot.receive(self.keyboard)

    def test_pressing_w_drives_the_robot_forward(self) -> None:
        self.keyboard.set_keys({"w"})
        self.world.step(1.0)
        self.assertAlmostEqual(self.robot.pose.x - self.world.start_pose.x, LINEAR_SPEED)

    def test_releasing_keys_stops_the_robot(self) -> None:
        self.keyboard.set_keys({"w"})
        self.keyboard.set_keys(set())
        self.assertEqual(self.robot.command.linear.x, 0.0)

    def test_robot_decides_what_the_input_means(self) -> None:
        class Ignoring(TurtleBot3):
            def on_interaction(self, interaction: Interaction[Any], state: Any) -> None:
                pass  # 이 로봇은 키보드를 무시한다

        robot, keyboard = Ignoring(), Keyboard()
        robot.receive(keyboard)
        keyboard.set_keys({"w"})
        self.assertEqual(robot.command.linear.x, 0.0)

    def test_base_robot_ignores_input_by_default(self) -> None:
        class Bare(Robot):
            def tick(self, clock: Any, world: Any) -> None:
                pass

        robot, keyboard = Bare(), Keyboard()
        robot.receive(keyboard)
        keyboard.set_keys({"w"})  # 기본 훅은 아무것도 하지 않는다: 예외 없이 지나간다
        self.assertEqual(keyboard.state, KeyState(frozenset({"w"})))


class TestTurtleBot3Keys(unittest.TestCase):
    """키보드 해석 (TurtleBot3 안): 눌린 키 -> robot.command."""

    def setUp(self) -> None:
        self.robot, self.keyboard = TurtleBot3(), Keyboard()
        self.robot.receive(self.keyboard)

    def press(self, *keys: str) -> Twist:
        self.keyboard.set_keys(set(keys))
        return self.robot.command

    def test_forward_turn_and_cancel(self) -> None:
        self.assertAlmostEqual(self.press("w").linear.x, LINEAR_SPEED)
        self.assertAlmostEqual(self.press("a").angular.z, ANGULAR_SPEED)
        self.assertAlmostEqual(self.press("d").angular.z, -ANGULAR_SPEED)
        cmd = self.press("up", "left")
        self.assertGreater(cmd.linear.x, 0)
        self.assertGreater(cmd.angular.z, 0)
        cmd = self.press("w", "s", "a", "d")
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0.0, 0.0))
        self.assertEqual(self.press().linear.x, 0.0)

    def test_q_z_change_speed_once_per_press(self) -> None:
        self.press("q")
        self.press("q", "w")  # 누르고 있는 q는 다시 세지 않는다
        self.assertAlmostEqual(self.robot.speed_scale, 1.25)
        self.assertAlmostEqual(self.robot.command.linear.x, LINEAR_SPEED * 1.25)

    def test_speed_scale_is_clamped(self) -> None:
        for _ in range(20):
            self.press("q")
            self.press()
        self.assertEqual(self.robot.speed_scale, 4.0)
        for _ in range(40):
            self.press("z")
            self.press()
        self.assertEqual(self.robot.speed_scale, 0.25)


class TestRobotBridge(unittest.TestCase):
    """kros 쪽: cmd_vel 토픽 -> robot.command, 라이다 측정 -> scan 토픽."""

    def test_cmd_vel_drives_the_robot_and_scan_comes_back(self) -> None:
        ctx = Context()
        self.addCleanup(ctx.shutdown)
        world, robot = MazeWorld(puck_count=0), TurtleBot3()
        world.add(robot)
        RobotBridge(ctx, robot)
        listener = Node(ctx, "listener")
        scans: list[LaserScan] = []
        listener.subscribe(SCAN_TOPIC_NAME, LaserScan, scans.append)
        listener.publish(CMD_VEL_TOPIC_NAME, Twist(Vector3(x=0.2), Vector3()))
        ctx.topic_manager.join()
        x = robot.pose.x
        world.step(1.0)
        self.assertAlmostEqual(robot.pose.x - x, 0.2)
        ctx.topic_manager.join()
        self.assertEqual(len(scans), 1)


if __name__ == "__main__":
    unittest.main()
