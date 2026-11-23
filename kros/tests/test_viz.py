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
    from kros.msgs.geometry_msgs import Point, Pose, Transform, TransformStamped, Vector3
    from kros.msgs.nav_msgs import Path as NavPath  # pathlib.Path와 겹치지 않게
    from kros.tf import TransformBroadcaster
    from kros.msgs.nav_msgs import FREE, OCCUPIED, UNKNOWN, MapMetaData, OccupancyGrid
    from kros.msgs.std_msgs import Header
    from kros.nodes import Node
    from kros.topic import QOS_LATCHED
    from kros.viz import Viz
    from kros.viz.nav_display import MAP_COLORS
    from kros.viz.viz import TOGGLE_OFF, TOGGLE_ON
    from panda3d.core import Vec3
    from kros.msgs.geometry_msgs import PoseStamped, PoseWithCovarianceStamped
    from kros.nodes import Cartographer, MapCatalog, MapSaver, MapServer
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
        viz = Viz(context=kros.context, package_root=robot.description_dir)
        self.addCleanup(viz.close)
        viz.base.taskMgr.step()
        self.assertIsNone(viz.node.latest("odom"))  # 오도메트리는 아직 안 받았다
        robot.command = Twist(Vector3(x=0.1), Vector3())
        for _ in range(120):
            world.step(1 / 60)
            source.set(world.time)
        kros.context.timer_manager.join()
        kros.context.topic_executor.join()
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
        for topic in ("tf", "tf_static", "robot_description", "map", "global_costmap/costmap", "local_costmap/costmap", "plan", "local_plan", "scan", "imu", "odom", "odometry/filtered"):
            self.assertIn(topic, viz.rows)  # 구독 중인 토픽이 패널에 줄로 나온다
        self.assertIn("Hz", viz.rows["scan"].getText())
        color = viz.rows["scan"].textNode.getTextColor()
        for got, want in zip(color, (0.4, 1.0, 0.5)):  # 방금 받았다: 초록
            self.assertAlmostEqual(got, want, places=2)
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
        self.assertTrue(viz.enabled["imu"])
        for name, check in viz.checks.items():  # 토글 버튼을 누르면 그 표시가 바뀌고 버튼 색도 바뀐다
            before = viz.enabled[name]
            check["command"](*check["extraArgs"])
            self.assertEqual(viz.enabled[name], not before, name)
            self.assertEqual(tuple(check["frameColor"]), TOGGLE_OFF if before else TOGGLE_ON, name)
        self.assertNotIn("tf_static", viz.checks)  # tf_static은 토글이 없다
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
        kros.context.topic_executor.join()
        viz = Viz(context=kros.context, package_root=TurtleBot3.description_dir)  # 지도를 낸 뒤에 켜도 latch라 받는다
        self.addCleanup(viz.close)
        TransformBroadcaster(made["mapper"]).send_transform(TransformStamped(Header(0.0, "odom"), "base_footprint", Transform()))  # odom 프레임이 있게 (tf는 latch가 아니라 viz가 켜진 뒤에)
        kros.context.topic_executor.join()
        viz.base.taskMgr.step()
        self.assertFalse(viz.nav.grids["map"].isHidden())
        low, high = viz.nav.grids["map"].getTightBounds()
        for got, want in zip((low.x, low.y, high.x, high.y), (1.0, 2.0, 3.0, 3.5)):  # 원점에서 4*0.5 x 3*0.5 m
            self.assertAlmostEqual(got, want, places=3)
        texture = viz.nav.grids["map"].findAllTextures()[0]
        self.assertEqual((texture.getXSize(), texture.getYSize()), (4, 3))
        image = texture.getRamImageAs("RGBA").getData()
        self.assertEqual(tuple(image[0:4]), tuple(MAP_COLORS[OCCUPIED]))  # 칸 (0, 0) = 벽 = 어두움
        self.assertEqual(tuple(image[4:8]), tuple(MAP_COLORS[UNKNOWN]))
        self.assertGreater(image[8], image[0])  # 빈 곳은 벽보다 밝다
        viz.fixed_menu.set(viz.fixed_menu["items"].index("map"))  # 패널에서 Fixed Frame을 map으로 고른다
        viz.base.taskMgr.step()
        self.assertEqual(viz.fixed_frame, "map")
        self.assertTrue(viz.nav.grids["map"].isHidden())  # map -> odom TF가 없으니 odom에 있는 지도를 놓을 수 없다
        self.assertIn("no tf yet", viz.hud.getText())
        viz.enabled["map"] = False
        viz._set_fixed_frame("odom")
        viz.base.taskMgr.step()
        self.assertTrue(viz.nav.grids["map"].isHidden())  # 체크를 끄면 숨는다


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
            return [made["watcher"], made["slam"], MapServer(ctx, parameters=folder), MapSaver(ctx, parameters=folder), MapCatalog(ctx, parameters={**folder, "list_period": 0.1})]

        self.source = SimulatedTimeSource()
        self.kros = Kros(create, time_source=self.source)
        self.addCleanup(self.kros.shutdown)
        slam = made["slam"]
        assert isinstance(slam, Cartographer)
        self.cartographer, self.watcher = slam, made["watcher"]
        self.viz = Viz(context=self.kros.context, package_root=TurtleBot3.description_dir)
        self.addCleanup(self.viz.close)
        self.panel = self.viz.slam

    def pump(self) -> None:
        self.source.set(self.source.now() + 0.02)
        self.kros.context.timer_manager.join()
        self.kros.context.topic_executor.join()
        self.viz.base.taskMgr.step()

    def wait_status(self, prefix: str) -> None:
        for _ in range(200):
            self.pump()
            if self.panel.status.getText().startswith(prefix):
                return
            time.sleep(0.005)
        self.fail(f"status: {self.panel.status.getText()!r}")

    def wait_map_names(self, count: int) -> None:
        for _ in range(300):
            self.pump()
            if len(self.panel.map_names()) == count:
                return
            time.sleep(0.005)
        self.fail(f"map names: {self.panel.map_names()!r}")

    def test_mode_save_and_load_go_through_parameter_and_services(self) -> None:
        self.panel.slam_button["command"]()  # start SLAM = ros2 param set /cartographer mode mapping
        self.assertEqual(self.cartographer.mode.value, "mapping")
        self.viz.base.taskMgr.step()
        self.assertEqual(self.panel.slam_button["text"], "stop SLAM")
        grid = OccupancyGrid(Header(0.0, "map"), MapMetaData(0.0, 0.05, 2, 1), (OCCUPIED, FREE))
        self.watcher.publish("map", grid, QOS_LATCHED)  # 저장할 지도 (SLAM 자리)
        self.panel.save_button["command"]()
        self.wait_status("saved")
        self.wait_map_names(1)
        names = self.panel.map_names()
        self.assertEqual(len(names), 1)
        self.assertTrue((self.maps / names[0] / "map.pgm").exists())  # maps/<이름>/map.yaml + map.pgm
        self.assertTrue((self.maps / names[0] / "map.yaml").exists())
        self.assertEqual(self.panel.map_menu.get(), "(select map)")  # 저절로 고르지 않는다: 사람이 고른다
        self.assertEqual(self.panel.map_menu["items"], ["(select map)", names[0]])
        self.panel.load_button["command"]()
        self.assertEqual(self.panel.status.getText(), "select a map first")
        self.panel.map_menu.set(1, fCommand=False)  # 고른다
        self.panel.load_button["command"]()
        self.wait_status("loaded")
        self.assertEqual(self.cartographer.mode.value, "localization")  # 저장한 지도로 위치만 잡는다
        maps: list[OccupancyGrid] = []
        self.watcher.subscribe("map", OccupancyGrid, maps.append, QOS_LATCHED)
        self.kros.context.topic_executor.join()
        self.assertGreater(maps[-1].info.width, 0)  # 띄운 지도
        self.panel.unload_button["command"]()  # 지도 내리기
        self.wait_status("map unloaded")
        self.kros.context.topic_executor.join()
        self.assertEqual((maps[-1].info.width, maps[-1].info.height), (0, 0))  # 빈 지도 = 지도 없음
        self.assertEqual(self.cartographer.mode.value, "idle")  # 지도 없이 위치 추정은 끈다
        self.assertEqual(self.panel.map_menu.get(), "(select map)")

    def test_pose_tool_publishes_initialpose_in_map_frame(self) -> None:
        got: list[PoseWithCovarianceStamped] = []
        self.watcher.subscribe("initialpose", PoseWithCovarianceStamped, got.append)
        self.viz.fixed_frame = "map"
        self.panel.toggle_pose_tool()
        self.panel.update((1.0, 2.0), True)  # 바닥을 누르고
        self.panel.update((1.0, 3.0), False)  # +y 쪽으로 끌어 놓는다
        self.kros.context.topic_executor.join()
        self.assertEqual(len(got), 1)
        pose = got[0].pose.pose
        self.assertEqual((got[0].header.frame_id, pose.position.x, pose.position.y), ("map", 1.0, 2.0))
        self.assertAlmostEqual(2 * math.atan2(pose.orientation.z, pose.orientation.w), math.pi / 2)
        self.assertFalse(self.panel.pose_tool)  # 한 번 찍으면 꺼진다
        self.viz.fixed_frame = "odom"
        self.panel.publish_initial_pose((0.0, 0.0, 0.0))  # map TF가 없으면 map 좌표로 못 바꾼다
        self.assertIn("map TF", self.panel.status.getText())

    def test_goal_tool_needs_localization_previews_an_arrow_and_publishes_goal_pose(self) -> None:
        got: list[PoseStamped] = []
        self.watcher.subscribe("goal_pose", PoseStamped, got.append)
        self.panel.toggle_tool("goal")
        self.assertIsNone(self.panel.tool)  # idle: 지도에서 위치를 잡기 전에는 목표를 못 찍는다
        self.assertIn("localization", self.panel.status.getText())
        self.cartographer.set_parameter("mode", "localization")
        self.viz.base.taskMgr.step()
        self.assertIn("LOCALIZATION", self.viz.mode_banner.getText())  # 지금 모드를 크게 보인다
        self.viz.fixed_frame = "map"
        self.panel.toggle_tool("goal")
        self.assertEqual(self.panel.tool, "goal")
        self.panel.update((1.0, 2.0), True)  # 누른 곳 = 위치
        self.panel.update((0.0, 2.0), True)  # -x 쪽으로 끄는 중
        arrow = self.panel._preview["goal"]
        self.assertFalse(arrow.isHidden())  # 끄는 동안 화살표로 미리 보인다
        self.assertAlmostEqual(arrow.getH() % 360, 180.0, places=3)
        self.panel.update((0.0, 2.0), False)  # 놓으면 낸다
        self.kros.context.topic_executor.join()
        self.assertTrue(arrow.isHidden())
        self.assertIsNone(self.panel.tool)
        self.assertEqual((got[-1].header.frame_id, got[-1].pose.position.x, got[-1].pose.position.y), ("map", 1.0, 2.0))
        q = got[-1].pose.orientation
        self.assertAlmostEqual(abs(2 * math.atan2(q.z, q.w)), math.pi)  # 끈 방향 = yaw

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

    def test_save_map_uses_the_name_from_the_entry(self) -> None:
        grid = OccupancyGrid(Header(0.0, "map"), MapMetaData(0.0, 0.05, 2, 1), (OCCUPIED, FREE))
        self.watcher.publish("map", grid, QOS_LATCHED)
        self.kros.context.topic_executor.join()
        self.panel.map_name.set("custom_map")
        self.panel.save_button["command"]()
        self.wait_status("saved")
        self.assertTrue((self.maps / "custom_map" / "map.yaml").exists())
        self.wait_map_names(1)
        self.assertIn("custom_map", self.panel.map_menu["items"])

    def test_teleop_speed_slider_scales_the_command(self) -> None:
        self.panel.teleop_slider["value"] = 1.5
        self.panel._teleop_scale_changed()
        self.assertAlmostEqual(self.viz._teleop_speed, 0.3)
        self.assertAlmostEqual(self.viz._teleop_turn, 1.5)
        self.assertIn("0.30", self.panel.teleop_status.getText())
        self.panel.teleop_slider["value"] = 0.25
        self.panel._teleop_scale_changed()
        self.assertAlmostEqual(self.viz._teleop_speed, 0.05)
        self.assertAlmostEqual(self.viz._teleop_turn, 0.25)

    def test_missing_nodes_are_reported_not_crashed(self) -> None:
        self.panel.map_saver = "nobody"
        self.panel.save_map()
        self.assertIn("no service", self.panel.status.getText())
        self.panel.cartographer = "nobody"
        self.panel.set_mode("mapping")
        self.assertIn("no node 'nobody'", self.panel.status.getText())


@unittest.skipUnless(HAS_PANDA, "panda3d 없음")
class TestVizNavigationDisplays(unittest.TestCase):
    def test_costmaps_plans_and_trail_are_drawn_where_their_frames_are(self) -> None:
        made: dict[str, Node] = {}
        kros = Kros(lambda ctx: [made.setdefault("n", Node(ctx, "Nav"))], time_source=SimulatedTimeSource())
        self.addCleanup(kros.shutdown)
        node = made["n"]
        viz = Viz(context=kros.context, package_root=TurtleBot3.description_dir)
        self.addCleanup(viz.close)
        broadcaster = TransformBroadcaster(node)
        broadcaster.send_transform(TransformStamped(Header(0.0, "odom"), "base_footprint", Transform()))
        grid = OccupancyGrid(Header(0.0, "odom"), MapMetaData(0.0, 0.5, 2, 1, Pose(Point(1.0, 0.0, 0.0))), (100, 50))
        for topic in ("global_costmap/costmap", "local_costmap/costmap"):
            node.publish(topic, grid, QOS_LATCHED)
        path = NavPath(Header(0.0, "odom"), tuple(PoseStamped(Header(0.0, "odom"), Pose(Point(x / 10, 0.0, 0.0))) for x in range(10)))
        node.publish("plan", path, QOS_LATCHED)
        node.publish("local_plan", path)
        kros.context.topic_executor.join()
        viz.base.taskMgr.step()
        for name in ("global_costmap/costmap", "local_costmap/costmap"):
            root = viz.nav.grids[name]
            self.assertFalse(root.isHidden(), name)
            low, high = root.getTightBounds()
            self.assertAlmostEqual(low.x, 1.0, places=3)  # costmap origin 자리부터
            self.assertAlmostEqual(high.x, 2.0, places=3)
        for name in ("plan", "local_plan"):
            self.assertFalse(viz.nav.paths[name].isHidden(), name)
        self.assertAlmostEqual(viz.nav.paths["plan"].getTightBounds()[1].x, 1.02, places=2)
        viz.toggle("global_costmap/costmap")
        viz.toggle("plan")
        viz.base.taskMgr.step()
        self.assertTrue(viz.nav.grids["global_costmap/costmap"].isHidden())  # 따로 끈다
        self.assertFalse(viz.nav.grids["local_costmap/costmap"].isHidden())
        self.assertTrue(viz.nav.paths["plan"].isHidden())
        for x in (0.1, 0.2, 0.3):  # 로봇이 움직이면 지나온 궤적이 쌓인다
            broadcaster.send_transform(TransformStamped(Header(x, "odom"), "base_footprint", Transform(Vector3(x, 0.0, 0.0))))
            kros.context.topic_executor.join()
            viz.base.taskMgr.step()
        self.assertEqual(len(viz.nav.trail), 4)
        self.assertAlmostEqual(viz.nav.trail[-1][0], 0.3)


if __name__ == "__main__":
    unittest.main()
