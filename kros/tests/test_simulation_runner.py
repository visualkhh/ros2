import threading
import time
import unittest

from kros.msgs.geometry_msgs import Twist, Vector3
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.simulation_runner import SimulationRunner
from kros.worlds.world import World


def make(seed: int = 1) -> tuple[SimulationRunner, TurtleBot3]:
    world, robot = MazeWorld(seed=seed), TurtleBot3()
    world.add(robot)
    return SimulationRunner(world), robot


class TestStepping(unittest.TestCase):
    def test_step_once_advances_the_world_by_the_fixed_step(self) -> None:
        runner, _ = make()
        for _ in range(60):
            runner.step_once()
        self.assertAlmostEqual(runner.world.time, 1.0)

    def test_step_hz_sets_the_fixed_step(self) -> None:
        world = MazeWorld(puck_count=0)
        runner = SimulationRunner(world, step_hz=100.0)
        runner.step_once()
        self.assertAlmostEqual(world.time, 0.01)

    def test_fixed_step_makes_physics_independent_of_wall_clock(self) -> None:
        (a, ra), (b, rb) = make(), make()
        for robot in (ra, rb):
            robot.command = Twist(Vector3(x=0.26), Vector3(z=0.4))
        for _ in range(120):
            a.step_once()
        for _ in range(120):
            b.step_once()
            time.sleep(0.0001)  # 실행 속도가 달라도
        self.assertEqual(ra.pose, rb.pose)  # 결과는 똑같다


class TestListeners(unittest.TestCase):
    def test_listeners_get_the_world_after_every_step_in_order(self) -> None:
        runner, _ = make()
        calls: list[str] = []
        times: list[float] = []

        def first(world: World) -> None:
            calls.append("first")

        def second(world: World) -> None:
            calls.append("second")
            times.append(world.time)

        runner.add_listener(first)
        runner.add_listener(second)
        for _ in range(3):
            runner.step_once()
        self.assertEqual(calls, ["first", "second"] * 3)
        self.assertAlmostEqual(times[-1], 3 / 60)

    def test_listener_sees_the_world_after_the_step_not_before(self) -> None:
        runner, robot = make()
        robot.command = Twist(Vector3(x=0.26), Vector3())
        seen: list[float] = []

        def listener(world: World) -> None:
            seen.append(robot.pose.x)

        runner.add_listener(listener)
        runner.step_once()
        self.assertGreater(seen[0], 0.0)  # 스텝이 끝난 뒤의 위치

    def test_listener_runs_on_the_simulation_thread_when_started(self) -> None:
        runner, _ = make()
        names: list[str] = []
        runner.add_listener(lambda world: names.append(threading.current_thread().name))
        runner.start()
        time.sleep(0.1)
        runner.stop()
        self.assertTrue(names and set(names) == {"simulation"})

    def test_runner_does_not_know_about_robots(self) -> None:
        # 러너는 월드만 안다: 로봇 없는 월드도 그대로 돈다
        world = MazeWorld(puck_count=0)
        runner = SimulationRunner(world)
        runner.step_once()
        self.assertAlmostEqual(world.time, runner.dt)


class TestThread(unittest.TestCase):
    def test_runs_in_real_time_on_its_own_thread_and_stops(self) -> None:
        runner, robot = make()
        robot.command = Twist(Vector3(x=0.26), Vector3())
        runner.start()
        started = time.perf_counter()
        time.sleep(0.5)
        runner.stop()
        elapsed = time.perf_counter() - started
        self.assertGreater(runner.world.time, 0.3)
        self.assertLess(runner.world.time, elapsed + 0.1)  # 실시간보다 빠르게 달리지 않는다
        self.assertGreater(robot.pose.x, 0.05)  # 스레드가 월드를 진행시켰다
        t = runner.world.time
        time.sleep(0.1)
        self.assertEqual(runner.world.time, t)  # stop 뒤에는 멈춘다

    def test_command_set_from_another_thread_takes_effect(self) -> None:
        runner, robot = make()
        runner.start()
        self.addCleanup(runner.stop)
        robot.command = Twist(Vector3(), Vector3(z=1.0))  # 메인 스레드에서 명령을 넣는다
        time.sleep(0.3)
        self.assertGreater(abs(robot.pose.theta), 0.1)


if __name__ == "__main__":
    unittest.main()
