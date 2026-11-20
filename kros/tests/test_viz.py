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
    from kros.examples.turtlebot3.main import IMU_CONFIG, ODOM_CONFIG
    from kros.nodes import DiffDriveOdometry, Ekf, RobotStatePublisher
    from kros.timer import SimulatedTimeSource
    from kros.context import Context
    from kros.msgs.geometry_msgs import Point, Pose, Transform, TransformStamped
    from kros.tf import TransformBroadcaster
    from kros.msgs.nav_msgs import FREE, OCCUPIED, UNKNOWN, MapMetaData, OccupancyGrid
    from kros.msgs.std_msgs import Header
    from kros.nodes import Node
    from kros.topic import QOS_LATCHED
    from kros.viz import Viz
    from kros.viz.viz import MAP_COLORS
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
                DiffDriveOdometry(ctx, parameters={"wheel_radius": WHEEL_RADIUS, "wheel_separation": WHEEL_SEPARATION, "publish_tf": False}),
                Ekf(ctx, parameters={"odom0": "odom", "odom0_config": ODOM_CONFIG, "imu0": "imu", "imu0_config": IMU_CONFIG}),  # main.py처럼 TF는 EKF가 낸다
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
        self.assertFalse(viz._labels["odom"].isHidden())  # TF 프레임 이름이 보인다
        self.assertEqual(viz._labels["base_footprint"].node().getText(), "base_footprint")
        self.assertIn("ekf  x=", viz.hud.getText())
        self.assertFalse(viz.arrows["odom"].isHidden())  # 바퀴 오도메트리 화살표
        self.assertGreater(viz.arrows["odom"].getX(), 0.1)  # 앞으로 간 만큼
        self.assertFalse(viz.arrows["odometry/filtered"].isHidden())
        text = " ".join(row.getText() for row in viz.rows.values()) + " ".join(viz.rows)
        for topic in ("tf", "tf_static", "robot_description", "scan", "imu", "odom", "odometry/filtered"):
            self.assertIn(topic, text)  # 구독 중인 토픽이 패널에 나온다
        self.assertIn("* LaserScan", text)
        viz.enabled["scan"] = viz.enabled["odom"] = viz.enabled["robot_description"] = False  # 체크 해제
        viz.base.taskMgr.step()
        self.assertTrue(viz.model.links["base_link"].isHidden())
        self.assertTrue(viz._scan_np is None or viz._scan_np.isHidden())
        self.assertNotIn("odom x=", viz.hud.getText())
        self.assertTrue(viz.arrows["odom"].isHidden())
        viz.enabled["tf"] = False
        viz.base.taskMgr.step()
        self.assertTrue(viz._labels["odom"].isHidden())  # tf를 끄면 이름도 숨는다
        viz.enabled["tf"] = True
        self.assertFalse(viz.enabled["scan"])
        self.assertIn("LaserScan", viz.rows["scan"].getText())
        self.assertTrue(viz.enabled["imu"])
        for name, check in viz.checks.items():  # 체크박스를 누르면 (눌린 것처럼 commandFunc) 그 표시가 바뀐다
            check["indicatorValue"] = viz.enabled[name]  # 앞에서 값만 바꿨으니 체크박스 모양을 맞춘다
            before = viz.enabled[name]
            check.commandFunc(None)
            self.assertEqual(viz.enabled[name], not before, name)
        self.assertNotIn("tf_static", viz.checks)  # tf_static은 체크박스가 없다
        self.assertTrue(viz.base.isAccepting("window-event"))  # ShowBase 자신의 창 이벤트 처리를 덮어쓰지 않았다
        viz._lens3d.setAspectRatio(3.0)  # 창 비율과 다른 렌즈 (오프스크린 버퍼는 크기를 못 바꿔서 이렇게 시험한다)
        viz.base.taskMgr.step()
        ratio = viz.win.getXSize() / viz.win.getYSize()
        self.assertAlmostEqual(viz._lens3d.getAspectRatio(), ratio, places=3)  # 창 크기가 바뀌어도 3D가 늘어나지 않는다
        viz.close()
        viz.close()  # 두 번 닫아도 된다


@unittest.skipUnless(HAS_PANDA, "panda3d 없음")
class TestVizMap(unittest.TestCase):
    def test_map_is_drawn_where_its_frame_is_and_fixed_frame_can_be_chosen(self) -> None:
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            made["mapper"] = Node(ctx, "Mapper")
            return [made["mapper"]]

        kros = Kros(create, time_source=SimulatedTimeSource())
        self.addCleanup(kros.shutdown)
        # 4x3칸, 0.5m 칸, 칸 (0, 0)의 모서리가 odom의 (1, 2): 맨 아래 줄 첫 칸이 벽, 둘째 칸은 모름
        cells = (OCCUPIED, UNKNOWN, FREE, FREE) + (FREE,) * 8
        grid = OccupancyGrid(Header(0.0, "odom"), MapMetaData(0.0, 0.5, 4, 3, Pose(Point(1.0, 2.0, 0.0))), cells)
        made["mapper"].publish("map", grid, QOS_LATCHED)
        kros.context.topic_manager.join()
        viz = Viz(kros, TurtleBot3.description_dir)  # 지도를 낸 뒤에 켜도 latch라 받는다
        self.addCleanup(viz.close)
        TransformBroadcaster(made["mapper"]).send_transform(TransformStamped(Header(0.0, "odom"), "base_footprint", Transform()))  # odom 프레임이 있게 (tf는 latch가 아니라 viz가 켜진 뒤에)
        kros.context.topic_manager.join()
        viz.base.taskMgr.step()
        self.assertFalse(viz._map_np.isHidden())
        low, high = viz._map_np.getTightBounds()
        for got, want in zip((low.x, low.y, high.x, high.y), (1.0, 2.0, 3.0, 3.5)):  # 원점에서 4*0.5 x 3*0.5 m
            self.assertAlmostEqual(got, want, places=3)
        texture = viz._map_np.findAllTextures()[0]
        self.assertEqual((texture.getXSize(), texture.getYSize()), (4, 3))
        image = texture.getRamImageAs("RGBA").getData()
        self.assertEqual(tuple(image[0:4]), tuple(MAP_COLORS[OCCUPIED]))  # 칸 (0, 0) = 벽 = 어두움
        self.assertEqual(tuple(image[4:8]), tuple(MAP_COLORS[UNKNOWN]))
        self.assertGreater(image[8], image[0])  # 빈 곳은 벽보다 밝다
        viz.fixed_menu.set(viz.fixed_menu["items"].index("map"))  # 패널에서 Fixed Frame을 map으로 고른다
        viz.base.taskMgr.step()
        self.assertEqual(viz.fixed_frame, "map")
        self.assertTrue(viz._map_np.isHidden())  # map -> odom TF가 없으니 odom에 있는 지도를 놓을 수 없다
        self.assertIn("no tf yet", viz.hud.getText())
        viz.enabled["map"] = False
        viz._set_fixed_frame("odom")
        viz.base.taskMgr.step()
        self.assertTrue(viz._map_np.isHidden())  # 체크를 끄면 숨는다


if __name__ == "__main__":
    unittest.main()
