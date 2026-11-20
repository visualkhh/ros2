import unittest

try:
    import panda3d  # noqa: F401

    HAS_PANDA = True
except ImportError:
    HAS_PANDA = False

if HAS_PANDA:
    from panda3d.core import loadPrcFileData

    loadPrcFileData("", "window-type offscreen")  # 창 없이 (GPU/화면이 없어도 돌린다)
    from kros.interactions import Keyboard
    from kros.viewer.panda_viewer import Viewer
    from kros.viewer.snapshot_recorder import SnapshotRecorder
    from kros.worlds.maze.maze_world import MazeWorld
    from kros.worlds.robots.turtlebot3 import TurtleBot3
    from kros.worlds.simulation_runner import SimulationRunner


@unittest.skipUnless(HAS_PANDA, "panda3d 없음")
class TestViewerSmoke(unittest.TestCase):
    """화면은 자동 테스트가 어려워서 오프스크린으로 몇 프레임만 돌려 본다. (ShowBase가 이미 쓰는 이름을 덮어써서 프레임마다 죽던 버그를 잡는 용도)"""

    def test_viewer_runs_frames_and_draws_the_latest_snapshot(self) -> None:
        world, robot, keyboard = MazeWorld(seed=1), TurtleBot3(), Keyboard()
        world.add(robot)
        robot.receive(keyboard)
        runner = SimulationRunner(world)
        recorder = SnapshotRecorder(world, robot)
        runner.add_listener(recorder.record)
        viewer = Viewer(recorder, keyboard.set_keys)
        self.addCleanup(viewer.destroy)
        viewer.keys.add("w")
        for _ in range(30):
            runner.step_once()  # 월드가 먼저 돌고
            viewer.taskMgr.step()  # 화면은 가장 최근 스냅샷을 그린다
        self.assertEqual(keyboard.state.keys if keyboard.state else None, frozenset({"w"}))  # 창의 키가 입력 장치로 갔다
        pose = recorder.latest.poses[recorder.latest.robot_index]
        self.assertGreater(pose.x, 0.0)
        self.assertAlmostEqual(viewer.robot.getX(), pose.x, places=5)  # 화면의 로봇이 스냅샷 위치로 갔다 (panda는 32비트 실수)
        self.assertIs(viewer.snap, recorder.latest)

    def test_viewer_does_not_shadow_showbase_attributes(self) -> None:
        world, robot = MazeWorld(puck_count=0), TurtleBot3()
        world.add(robot)
        recorder = SnapshotRecorder(world, robot)
        viewer = Viewer(recorder, lambda keys: None)
        self.addCleanup(viewer.destroy)
        self.assertIsNot(viewer.recorder, recorder)  # ShowBase 고유의 recorder 속성을 우리 객체로 덮어쓰지 않았다
        self.assertIs(viewer.snapshots, recorder)

    def test_physical_keys_read_without_a_window_or_return_none(self) -> None:
        from kros.viewer.physical_keys import PhysicalKeys

        keys = PhysicalKeys()  # X11이 없으면 None, 있으면 지금 눌린 조종 키 집합 (아무 키도 안 누르고 돌린다)
        self.addCleanup(keys.close)
        pressed = keys.pressed()
        self.assertTrue(pressed is None or pressed <= {"q", "w", "a", "s", "d", "z", "up", "down", "left", "right"})


if __name__ == "__main__":
    unittest.main()
