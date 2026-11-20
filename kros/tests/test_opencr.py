import unittest

from kros.hardwares import Controller, Hardware
from kros.hardwares.controllers.opencr import OpenCr
from kros.hardwares.imus.mpu9250 import DummyMpu9250
from kros.msgs.geometry_msgs import Pose2D, Twist, Vector3
from kros.worlds.clock import Clock
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.objects.world_object import WorldObject


class TestOpenCr(unittest.TestCase):
    def test_board_holds_the_drive_and_the_imu_but_not_the_lidar(self) -> None:
        robot = TurtleBot3()
        board = robot.opencr
        self.assertIsInstance(board, Controller)
        self.assertEqual((board.model, board.name), ("OpenCR1.0", "opencr"))
        self.assertEqual(board.parts, [board.drive, board.imu])
        self.assertNotIn(robot.lidar, board.parts)  # 라이다는 라즈베리파이에 꽂힌다

    def test_parts_find_the_robot_through_the_board(self) -> None:
        robot = TurtleBot3()
        self.assertIs(robot.imu.find_owner(WorldObject), robot)
        self.assertIs(robot.drive.left.find_owner(OpenCr), robot.opencr)
        self.assertIsNone(robot.find_owner(Hardware))  # 최상위

    def test_powering_off_the_board_turns_everything_on_it_off_but_not_the_lidar(self) -> None:
        robot = TurtleBot3()
        robot.opencr.power_off()
        for part in (robot.drive, robot.drive.left, robot.drive.right, robot.imu):
            self.assertFalse(part.is_on)
        self.assertTrue(robot.lidar.is_on)

    def test_board_off_stops_the_robot_and_the_imu(self) -> None:
        world, robot = MazeWorld(puck_count=0), TurtleBot3()
        world.add(robot, Pose2D(0.0, 0.0, 0.0))
        robot.command = Twist(Vector3(x=0.2), Vector3())
        robot.opencr.power_off()
        for _ in range(60):
            world.step(1 / 60)
        self.assertEqual(robot.pose.x, 0.0)
        self.assertIsNone(robot.imu.read())  # 꺼진 센서는 값을 내지 않는다

    def test_hardware_runs_on_a_clock_alone_without_any_world(self) -> None:
        # 진짜 하드웨어를 돌리는 실시간 루프처럼: 월드 없이 시계 신호만 내려준다
        board = TurtleBot3.make_opencr(imu=DummyMpu9250())
        for i in range(1, 11):
            board.imu.tick(Clock(i * 0.01, 0.01))
        self.assertIsNotNone(board.imu.read())

    def test_hardware_base_does_not_know_the_world(self) -> None:
        import pathlib

        text = (pathlib.Path(__file__).parent.parent / "hardwares" / "hardware.py").read_text()
        self.assertNotIn("kros.worlds.world", text)

    def test_world_tells_each_object_where_it_is(self) -> None:
        world, robot = MazeWorld(puck_count=0), TurtleBot3()
        self.assertIsNone(robot.world)
        world.add(robot)
        self.assertIs(robot.world, world)

    def test_parts_can_be_swapped_when_making_the_board(self) -> None:
        dummy = DummyMpu9250()
        self.assertIs(TurtleBot3(opencr=TurtleBot3.make_opencr(imu=dummy)).imu, dummy)


if __name__ == "__main__":
    unittest.main()
