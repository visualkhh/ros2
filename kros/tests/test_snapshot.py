import dataclasses
import pickle
import unittest

from kros.msgs.geometry_msgs import Twist, Vector3
from kros.viewer.snapshot import Snapshot
from kros.viewer.snapshot_recorder import SnapshotRecorder
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.simulation_runner import SimulationRunner


def make() -> tuple[SimulationRunner, SnapshotRecorder, TurtleBot3]:
    world, robot = MazeWorld(seed=1), TurtleBot3()
    world.add(robot)
    runner = SimulationRunner(world)
    recorder = SnapshotRecorder(world, robot)
    runner.add_listener(recorder.record)  # 스텝마다 그 순간을 스냅샷으로
    return runner, recorder, robot


class TestSnapshotRecorder(unittest.TestCase):
    def test_has_an_initial_snapshot_before_any_step(self) -> None:
        _, recorder, _ = make()
        self.assertEqual(recorder.latest.time, 0.0)

    def test_snapshot_follows_the_world_and_is_immutable(self) -> None:
        runner, recorder, robot = make()
        initial = recorder.latest
        robot.command = Twist(Vector3(x=0.26), Vector3())
        for _ in range(60):
            runner.step_once()
        snap = recorder.latest
        self.assertIsNot(snap, initial)
        self.assertAlmostEqual(snap.time, 1.0)
        self.assertEqual(len(snap.poses), len(runner.world.objects))
        self.assertEqual(snap.poses[snap.robot_index], robot.pose)
        self.assertAlmostEqual(snap.poses[snap.robot_index].x - initial.poses[initial.robot_index].x, 0.26, places=2)
        self.assertEqual(initial.poses[initial.robot_index].x, 0.0)  # 이전 스냅샷은 그대로 (불변)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            snap.time = 5.0  # type: ignore[misc]

    def test_scan_in_snapshot_only_changes_when_lidar_measures(self) -> None:
        runner, recorder, _ = make()
        runner.step_once()
        first = recorder.latest.scan
        assert first is not None
        runner.step_once()  # 1/60초 뒤: 다음 측정(0.2초)은 아직
        self.assertIs(recorder.latest.scan, first)  # 같은 객체: 화면이 다시 그릴 필요 없다
        for _ in range(12):
            runner.step_once()
        self.assertIsNot(recorder.latest.scan, first)

    def test_wheel_angles_come_from_the_wheel_motors(self) -> None:
        runner, recorder, robot = make()
        robot.command = Twist(Vector3(x=0.2), Vector3())
        for _ in range(60):
            runner.step_once()
        left, right = recorder.latest.wheel_angles
        self.assertAlmostEqual(left, robot.drive.left.position)
        self.assertAlmostEqual(right, robot.drive.right.position)
        self.assertGreater(left, 1.0)

    def test_snapshot_is_plain_data(self) -> None:
        runner, recorder, _ = make()
        runner.step_once()
        snap: Snapshot = recorder.latest
        self.assertEqual(pickle.loads(pickle.dumps(snap)), snap)  # 다른 프로세스로 보낼 수 있다


if __name__ == "__main__":
    unittest.main()
