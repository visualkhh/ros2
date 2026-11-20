import unittest

try:
    import panda3d  # noqa: F401

    HAS_PANDA = True
except ImportError:
    HAS_PANDA = False

if HAS_PANDA:
    from panda3d.core import loadPrcFileData

    loadPrcFileData("", "window-type offscreen")
    from kros import Kros
    from kros.examples.turtlebot3.robot_bridge import RobotBridge
    from kros.msgs.geometry_msgs import Twist, Vector3
    from kros.nodes import DiffDriveOdometry, RobotStatePublisher
    from kros.timer import SimulatedTimeSource
    from kros.viz import Viz
    from kros.worlds.maze.maze_world import MazeWorld
    from kros.worlds.robots.turtlebot3 import TurtleBot3
    from kros.worlds.robots.turtlebot3.turtlebot3 import WHEEL_RADIUS, WHEEL_SEPARATION


@unittest.skipUnless(HAS_PANDA, "panda3d 없음")
class TestVizSmoke(unittest.TestCase):
    def test_viz_draws_only_what_it_receives_through_topics(self) -> None:
        world, robot = MazeWorld(puck_count=0), TurtleBot3()
        world.add(robot)
        source = SimulatedTimeSource()
        kros = Kros(
            lambda ctx: [
                RobotBridge(ctx, robot),
                RobotStatePublisher(ctx, parameters={"robot_description": robot.urdf_path.read_text()}),
                DiffDriveOdometry(ctx, parameters={"wheel_radius": WHEEL_RADIUS, "wheel_separation": WHEEL_SEPARATION}),
            ],
            time_source=source,
        )
        self.addCleanup(kros.shutdown)
        viz = Viz(kros, robot.description_dir)
        self.addCleanup(viz.close)
        viz.base.taskMgr.step()
        self.assertIsNone(viz.node.latest("odom"))  # 오도메트리는 아직 안 받았다
        robot.command = Twist(Vector3(x=0.1), Vector3())
        for _ in range(120):
            world.step(1 / 60)
            source.set(world.time)
        kros.context.timer_manager.join()
        kros.context.topic_manager.join()
        viz.base.taskMgr.step()
        self.assertIsNotNone(viz.model)  # robot_description을 받아 모델을 만들었다
        self.assertEqual(viz.node.buffer.lookup_transform("odom", "base_footprint").header.frame_id, "odom")
        assert viz.model is not None
        self.assertFalse(viz.model.links["base_link"].isHidden())  # TF가 있는 링크는 보인다
        self.assertIn("odom x=", viz.hud.getText())
        text = " ".join(row.getText() for row in viz.rows.values())
        for topic in ("tf", "tf_static", "robot_description", "scan", "imu", "odom"):
            self.assertIn(topic, text)  # 구독 중인 토픽이 패널에 나온다
        self.assertIn("* scan", text)
        viz.enabled["scan"] = viz.enabled["odom"] = viz.enabled["robot_description"] = False  # 체크 해제
        viz.base.taskMgr.step()
        self.assertTrue(viz.model.links["base_link"].isHidden())
        self.assertTrue(viz._scan_np is None or viz._scan_np.isHidden())
        self.assertNotIn("odom x=", viz.hud.getText())
        self.assertIn("[ ] * scan", viz.rows["scan"].getText())
        self.assertIn("[x] * imu", viz.rows["imu"].getText())
        for index, name in enumerate(viz.rows):  # 각 줄의 가운데를 누르면 그 줄이 바뀐다
            before = dict(viz.enabled)
            viz.toggle_row_at(0.84 - index * 0.13 - 0.03)
            changed = [k for k in before if before[k] != viz.enabled[k]]
            self.assertEqual(changed, [name] if name in before else [], name)
        viz.toggle_row_at(-0.99)  # 빈 곳
        viz.close()
        viz.close()  # 두 번 닫아도 된다


if __name__ == "__main__":
    unittest.main()
