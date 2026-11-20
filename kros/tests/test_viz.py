import math
import tempfile
import time
import unittest
from pathlib import Path

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
    from panda3d.core import Vec3
    from kros.msgs.geometry_msgs import PoseWithCovarianceStamped
    from kros.nodes import Cartographer, MapSaver, MapServer
    from panda3d.core import MouseButton, VirtualMouse
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


@unittest.skipUnless(HAS_PANDA, "panda3d 없음")
class TestVizSlamPanel(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.maps = Path(directory.name)
        made: dict[str, Node] = {}

        def create(ctx: Context) -> list[Node]:
            folder = {"maps_directory": str(self.maps)}
            made["watcher"], made["slam"] = Node(ctx, "Watcher"), Cartographer(ctx, parameters={"mode": "idle"})
            return [made["watcher"], made["slam"], MapServer(ctx, parameters=folder), MapSaver(ctx, parameters=folder)]

        self.kros = Kros(create, time_source=SimulatedTimeSource())
        self.addCleanup(self.kros.shutdown)
        slam = made["slam"]
        assert isinstance(slam, Cartographer)
        self.cartographer, self.watcher = slam, made["watcher"]
        self.viz = Viz(self.kros, TurtleBot3.description_dir)
        self.addCleanup(self.viz.close)
        self.viz.slam.maps_directory = self.maps
        self.panel = self.viz.slam

    def wait_status(self, prefix: str) -> None:
        for _ in range(200):
            self.viz.base.taskMgr.step()
            if self.panel.status.getText().startswith(prefix):
                return
            time.sleep(0.01)
        self.fail(f"status: {self.panel.status.getText()!r}")

    def test_mode_save_and_load_go_through_parameter_and_services(self) -> None:
        self.panel.slam_button["command"]()  # start SLAM = ros2 param set /cartographer mode mapping
        self.assertEqual(self.cartographer.mode.value, "mapping")
        self.viz.base.taskMgr.step()
        self.assertEqual(self.panel.slam_button["text"], "stop SLAM")
        grid = OccupancyGrid(Header(0.0, "map"), MapMetaData(0.0, 0.05, 2, 1), (OCCUPIED, FREE))
        self.watcher.publish("map", grid, QOS_LATCHED)  # 저장할 지도 (SLAM 자리)
        self.panel.save_button["command"]()
        self.wait_status("saved")
        names = self.panel.map_names()
        self.assertEqual(len(names), 1)
        self.assertTrue((self.maps / f"{names[0]}.pgm").exists())
        self.assertEqual(self.panel.map_menu.get(), names[0])  # 방금 저장한 지도가 골라져 있다
        self.panel.load_button["command"]()
        self.wait_status("loaded")
        self.assertEqual(self.cartographer.mode.value, "localization")  # 저장한 지도로 위치만 잡는다

    def test_pose_tool_publishes_initialpose_in_map_frame(self) -> None:
        got: list[PoseWithCovarianceStamped] = []
        self.watcher.subscribe("initialpose", PoseWithCovarianceStamped, got.append)
        self.viz.fixed_frame = "map"
        self.panel.toggle_pose_tool()
        self.panel.update((1.0, 2.0), True)  # 바닥을 누르고
        self.panel.update((1.0, 3.0), False)  # +y 쪽으로 끌어 놓는다
        self.kros.context.topic_manager.join()
        self.assertEqual(len(got), 1)
        pose = got[0].pose.pose
        self.assertEqual((got[0].header.frame_id, pose.position.x, pose.position.y), ("map", 1.0, 2.0))
        self.assertAlmostEqual(2 * math.atan2(pose.orientation.z, pose.orientation.w), math.pi / 2)
        self.assertFalse(self.panel.pose_tool)  # 한 번 찍으면 꺼진다
        self.viz.fixed_frame = "odom"
        self.panel.publish_initial_pose((0.0, 0.0, 0.0))  # map TF가 없으면 map 좌표로 못 바꾼다
        self.assertIn("map TF", self.panel.status.getText())

    def test_ground_point_under_the_screen_center_is_the_camera_target(self) -> None:
        self.viz.target = Vec3(0.7, -0.4, 0.0)
        self.viz.base.taskMgr.step()
        point = self.viz.ground_point(0.0, 0.0)
        assert point is not None
        self.assertAlmostEqual(point[0], 0.7, places=3)
        self.assertAlmostEqual(point[1], -0.4, places=3)
        self.viz.pitch = 0.05  # 거의 수평으로 보면 화면 위쪽은 하늘
        self.viz.base.taskMgr.step()
        self.assertIsNone(self.viz.ground_point(0.0, 0.9))

    def click(self, widget: object, mouse: "VirtualMouse") -> None:
        """위젯의 클릭 영역 한가운데를 진짜 마우스처럼 누른다. 그 점이 창 안이어야 사람이 누를 수 있다."""
        watcher = self.viz.mouse.node()
        frames = {group.getRegion(i).getName(): group.getRegion(i).getFrame() for g in range(watcher.getNumGroups()) for group in [watcher.getGroup(g)] for i in range(group.getNumRegions())}
        left, right, bottom, top = frames[widget.guiId]  # type: ignore[attr-defined]
        x, y = ((left + right) / 2 + 1) / 2 * self.size[0], (1 - (bottom + top) / 2) / 2 * self.size[1]
        self.assertTrue(0 <= x < self.size[0] and 0 <= y < self.size[1], f"{widget}: 클릭 영역이 창 밖 ({x:.0f}, {y:.0f})")
        mouse.setMousePos(int(x), int(y))
        self.viz.base.taskMgr.step()
        mouse.pressButton(MouseButton.one())
        self.viz.base.taskMgr.step()
        mouse.releaseButton(MouseButton.one())
        self.viz.base.taskMgr.step()

    def test_checkboxes_and_buttons_respond_to_real_mouse_clicks(self) -> None:
        # DirectGUI는 클릭 영역을 -1~1 화면 좌표로 마우스와 비교한다: 2D 장면 좌표를 창 비율로 늘리면 왼쪽 패널의 영역이 창 밖으로 나가 눌리지 않았다
        self.size = (self.viz.win.getXSize(), self.viz.win.getYSize())
        mouse = VirtualMouse("test-mouse")
        mouse.setWindowSize(*self.size)
        mouse.setMouseOn(True)
        self.viz.mouse.reparentTo(self.viz.base.dataRoot.attachNewNode(mouse))
        self.viz.base.taskMgr.step()
        for name, check in self.viz.checks.items():
            before = self.viz.enabled[name]
            self.click(check, mouse)
            self.assertEqual(self.viz.enabled[name], not before, name)
        self.click(self.panel.slam_button, mouse)
        self.assertEqual(self.cartographer.mode.value, "mapping")
        self.click(self.panel.slam_button, mouse)
        self.assertEqual(self.cartographer.mode.value, "idle")
        self.click(self.panel.pose_button, mouse)
        self.assertTrue(self.panel.pose_tool)

    def test_missing_nodes_are_reported_not_crashed(self) -> None:
        self.panel.map_saver = "nobody"
        self.panel.save_map()
        self.assertIn("no service", self.panel.status.getText())
        self.panel.cartographer = "nobody"
        self.panel.set_mode("mapping")
        self.assertIn("no node 'nobody'", self.panel.status.getText())


if __name__ == "__main__":
    unittest.main()
