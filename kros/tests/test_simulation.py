import dataclasses
import math
import time
import unittest

from kros.simulation.runner import SimulationRunner
from kros.simulation.snapshot import Snapshot
from kros.msgs.geometry_msgs import Twist, Vector3
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3


def make(seed: int = 1) -> SimulationRunner:
    world, robot = MazeWorld(seed=seed), TurtleBot3()
    world.add(robot)
    return SimulationRunner(world, robot)


class TestSnapshot(unittest.TestCase):
    def test_snapshot_follows_the_world_and_is_immutable(self) -> None:
        runner = make()
        initial = runner.latest
        runner.robot.command = Twist(Vector3(x=0.26), Vector3())
        for _ in range(60):
            runner.step_once()
        snap = runner.latest
        self.assertIsNot(snap, initial)
        self.assertAlmostEqual(snap.time, 1.0)
        self.assertEqual(len(snap.poses), len(runner.world.objects))
        self.assertEqual(snap.poses[snap.robot_index], runner.robot.pose)
        self.assertAlmostEqual(snap.poses[snap.robot_index].x - initial.poses[initial.robot_index].x, 0.26, places=2)
        self.assertEqual(initial.poses[initial.robot_index].x, 0.0)  # 이전 스냅샷은 그대로 (불변)
        with self.assertRaises(dataclasses.FrozenInstanceError):
            snap.time = 5.0  # type: ignore[misc]

    def test_scan_in_snapshot_only_changes_when_lidar_measures(self) -> None:
        runner = make()
        runner.step_once()
        first = runner.latest.scan
        assert first is not None
        runner.step_once()  # 1/60초 뒤: 다음 측정(0.2초)은 아직
        self.assertIs(runner.latest.scan, first)  # 같은 객체: 화면이 다시 그릴 필요 없다
        for _ in range(12):
            runner.step_once()
        self.assertIsNot(runner.latest.scan, first)

    def test_snapshot_is_plain_data(self) -> None:
        import pickle

        snap: Snapshot = make().step_once()
        self.assertEqual(pickle.loads(pickle.dumps(snap)), snap)  # 다른 프로세스로 보낼 수 있다


class TestRunnerThread(unittest.TestCase):
    def test_runs_in_real_time_on_its_own_thread_and_stops(self) -> None:
        runner = make()
        runner.robot.command = Twist(Vector3(x=0.26), Vector3())
        runner.start()
        started = time.perf_counter()
        time.sleep(0.5)
        runner.stop()
        elapsed = time.perf_counter() - started
        self.assertGreater(runner.latest.time, 0.3)
        self.assertLess(runner.latest.time, elapsed + 0.1)  # 실시간보다 빠르게 달리지 않는다
        self.assertGreater(runner.latest.poses[runner.latest.robot_index].x, 0.05)  # 스레드가 월드를 진행시켰다
        t = runner.latest.time
        time.sleep(0.1)
        self.assertEqual(runner.latest.time, t)  # stop 뒤에는 멈춘다

    def test_command_set_from_another_thread_takes_effect(self) -> None:
        runner = make()
        runner.start()
        self.addCleanup(runner.stop)
        runner.robot.command = Twist(Vector3(), Vector3(z=1.0))  # 메인 스레드에서 명령을 넣는다
        time.sleep(0.3)
        self.assertGreater(abs(runner.latest.poses[runner.latest.robot_index].theta), 0.1)

    def test_fixed_step_makes_physics_independent_of_wall_clock(self) -> None:
        a, b = make(), make()
        for r in (a, b):
            r.robot.command = Twist(Vector3(x=0.26), Vector3(z=0.4))
        for _ in range(120):
            a.step_once()
        for _ in range(120):
            b.step_once()
            time.sleep(0.0001)  # 실행 속도가 달라도
        self.assertEqual(a.latest.poses, b.latest.poses)  # 결과는 똑같다


class TestBatchRays(unittest.TestCase):
    def test_ray_distances_equals_individual_rays(self) -> None:
        world = MazeWorld(seed=2)
        angles = [i * math.pi / 36 for i in range(72)]
        batch = world.ray_distances(0.3, 0.2, angles, 0.05)
        single = [world.ray_distance(0.3, 0.2, a, 0.05) for a in angles]
        self.assertEqual(len(batch), len(single))
        for b, s in zip(batch, single):
            self.assertAlmostEqual(b, s) if math.isfinite(s) else self.assertEqual(b, s)

    def test_batch_matches_single_for_many_poses_and_heights(self) -> None:
        world = MazeWorld(seed=3)
        angles = [i * 2 * math.pi / 90 for i in range(90)]
        for x, y in ((0.0, 0.0), (-1.5, -2.7), (1.6, 0.5), (0.3, 2.2), (-2.0, 3.0)):
            for height in (0.0, 0.132, 0.2, 0.3):
                batch = world.ray_distances(x, y, angles, height)
                for a, b in zip(angles, batch):
                    s = world.ray_distance(x, y, a, height)
                    self.assertAlmostEqual(b, s, msg=f"{(x, y, a, height)}") if math.isfinite(s) else self.assertEqual(b, s, f"{(x, y, a, height)}")


if __name__ == "__main__":
    unittest.main()
