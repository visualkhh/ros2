import math
import threading
import time
from pathlib import Path

from direct.gui import DirectGuiGlobals as DGG
from direct.gui.DirectCheckButton import DirectCheckButton
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.DirectObject import DirectObject
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight, ButtonThrower, Camera, CardMaker, DirectionalLight, Geom, GeomNode, GeomPoints, GeomVertexData, GeomVertexFormat, GeomVertexWriter,
    LineSegs, MouseAndKeyboard, OrthographicLens, PGTop, MouseButton, MouseWatcher, NodePath, PerspectiveLens, TextNode, Vec3, WindowProperties, LQuaternionf,
    LVecBase4f, GraphicsWindow,
)

from kros import Kros
from kros.msgs.geometry_msgs import TransformStamped
from kros.msgs.nav_msgs import Odometry
from kros.msgs.sensor_msgs import Imu, LaserScan
from kros.msgs.std_msgs import String
from kros.nodes import Node
from kros.msgs.tf2_msgs import TFMessage
from kros.tf import QOS_TF, QOS_TF_STATIC, ROBOT_DESCRIPTION_TOPIC_NAME, TF_STATIC_TOPIC_NAME, TF_TOPIC_NAME, Buffer, TransformException, TransformListener
from kros.topic import QOS_LATCHED, QOS_SENSOR_DATA
from kros.viewer.urdf_model import UrdfModel

NAME = "Viz"
HELP = "left drag: orbit    right drag: pan    wheel: zoom    checkbox: show/hide    esc: close"
PANEL_WIDTH, ROW_TOP, ROW_STEP = 0.7, 0.84, 0.13  # 2D 장면 좌표: 세로 +-1, 가로는 창 비율에 따라 (왼쪽 끝에서 잰다)
ODOMETRY_ARROWS = {"odom": (1.0, 0.35, 0.85, 1.0), "odometry/filtered": (0.3, 0.9, 1.0, 1.0)}  # 바퀴 오도메트리(분홍, TF 부모 선이 노랑이라)와 EKF 결과(하늘색)를 화살표로 겹쳐 본다
TOGGLES = ("grid", "tf", ROBOT_DESCRIPTION_TOPIC_NAME, "scan", "imu", *ODOMETRY_ARROWS)  # 체크박스가 있는 줄 (tf_static은 tf 화면에 같이 쓰여서 없다)


class VizNode(Node):
    """Viz가 쓰는 구독 노드. 받은 값은 락 안에서 최신 것만 들고 있고, 화면은 매 프레임 가져가 그린다 (구독 콜백은 다른 스레드)."""

    def __init__(self, context, name: str = NAME) -> None:  # type: ignore[no-untyped-def]
        super().__init__(context, name)
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, self)
        self._lock = threading.Lock()
        self._latest: dict[str, object] = {}
        self._stats: dict[str, tuple[str, int, float, float]] = {}  # 토픽 -> (타입 이름, 받은 개수, 첫 수신 시각, 마지막 수신 시각)
        for topic, msg_type, qos in (
            (TF_TOPIC_NAME, TFMessage, QOS_TF),  # 이 둘은 TransformListener도 따로 구독한다: 여기서는 패널의 통계용
            (TF_STATIC_TOPIC_NAME, TFMessage, QOS_TF_STATIC),
            (ROBOT_DESCRIPTION_TOPIC_NAME, String, QOS_LATCHED),
            ("scan", LaserScan, QOS_SENSOR_DATA),
            ("imu", Imu, QOS_SENSOR_DATA),
            ("odom", Odometry, QOS_SENSOR_DATA),
            ("odometry/filtered", Odometry, QOS_SENSOR_DATA),
        ):
            self._stats[topic] = (msg_type.__name__, 0, 0.0, 0.0)  # 구독만 하고 아직 못 받은 토픽도 패널에 보인다
            self.subscribe(topic, msg_type, lambda message, key=topic: self._store(key, message), qos)  # type: ignore[misc]

    def _store(self, key: str, message: object) -> None:
        now = time.monotonic()
        with self._lock:
            self._latest[key] = message
            kind, count, first, _ = self._stats[key]
            self._stats[key] = (kind, count + 1, first or now, now)

    def topic_rows(self) -> dict[str, str]:
        """구독 중인 토픽 한 줄씩: 상태(* 최근에 받음, - 2초 넘게 조용함/아직 없음), 이름, 타입, 개수, 평균 Hz."""
        now = time.monotonic()
        with self._lock:
            stats = dict(self._stats)
        rows = {}
        for topic, (kind, count, first, last) in stats.items():
            live = count > 0 and now - last < 2.0
            hz = (count - 1) / (last - first) if count > 1 and last > first else 0.0
            rows[topic] = f"{'*' if live else '-'} {topic}\n      {kind}  n={count}  {hz:.1f}Hz"
        return rows

    def latest(self, topic: str) -> object | None:
        with self._lock:
            return self._latest.get(topic)


def _mat_of(stamped: TransformStamped, node: NodePath) -> None:
    t, q = stamped.transform.translation, stamped.transform.rotation
    node.setPos(t.x, t.y, t.z)
    node.setQuat(LQuaternionf(q.w, q.x, q.y, q.z))


class Viz:
    """Viz(kros, package_root).run(): kros 안의 토픽을 구독해 그린다. (network 없이 같은 프로세스의 Kros 하나만 받는다)

    base를 주면 그 ShowBase에 두 번째 창을 열고, 안 주면 자기 ShowBase를 만든다 (창은 프로세스에 ShowBase 하나만 가능).
    package_root: URDF의 package://<pkg>/ 뒤 경로가 놓인 폴더 (메시 파일 위치). fixed_frame: 화면의 기준 프레임.
    그리는 것: 바닥 격자, TF 축과 부모 선, robot_description 모델(TF로 배치), 라이다 점(scan_frame에서), IMU/odom 글자,
    odom과 odometry/filtered의 위치 화살표 (둘 다 fixed_frame 기준일 때만: TF는 둘 중 하나만 내므로 나머지는 화살표로 비교한다).
    """

    def __init__(self, kros: Kros, package_root: Path, base: ShowBase | None = None, fixed_frame: str = "odom", scan_frame: str = "base_scan", imu_frame: str = "imu_link") -> None:
        self.kros, self.package_root = kros, package_root
        self.fixed_frame, self.scan_frame = fixed_frame, scan_frame
        self.node = VizNode(kros.context)
        self.own_base = base is None
        self.base = base or ShowBase()
        if base is None:
            props, self.win = None, self.base.win
        else:
            props = WindowProperties()
            props.setTitle("kros viz")
            props.setSize(900, 700)
            self.win = base.openWindow(props=props, makeCamera=False)
        self.scene = NodePath("viz")
        self._build_scene()
        self.yaw, self.pitch, self.dist, self.target = 0.8, 0.8, 2.5, Vec3(0, 0, 0.1)
        self.model: UrdfModel | None = None
        self.model_text: object = None
        self._scan_drawn: object = None
        self._scan_np: NodePath | None = None
        self._tf_np: NodePath | None = None
        self._last_mouse: tuple[float, float] | None = None
        self._closed = False
        self._build_input()
        self.enabled = {name: True for name in TOGGLES}
        self.checks: dict[str, DirectCheckButton] = {}  # 체크박스 (grid, tf, 모델, scan, imu, odom)
        self.rows: dict[str, OnscreenText] = {}  # 줄마다 토픽 상태 글자 (grid는 토픽이 아니라 맨 위 줄)
        card = CardMaker("panel")
        card.setFrame(0, PANEL_WIDTH, -1, 1)
        panel = self.left.attachNewNode(card.generate())
        panel.setColor(0.08, 0.09, 0.13, 0.85)
        panel.setTransparency(True)
        OnscreenText(text="Subscriptions", parent=self.left, pos=(0.04, 0.92), align=TextNode.ALeft, fg=(0.6, 0.8, 1, 1), scale=0.05)
        for index, name in enumerate(("grid", *self.node.topic_rows())):
            y = ROW_TOP - index * ROW_STEP
            if name in self.enabled:
                self.checks[name] = DirectCheckButton(
                    parent=self.left, text=name, text_align=TextNode.ALeft, text_fg=(1, 1, 1, 1), scale=0.045, pos=(0.07, 0, y), relief=None,
                    indicatorValue=1, boxPlacement="left", boxBorder=0.08, boxRelief=DGG.FLAT, indicator_frameColor=(0.2, 0.24, 0.34, 1), indicator_text_fg=(0.4, 1, 0.5, 1), indicator_text_pos=(0, -0.01), indicator_text_scale=1.2, command=self._set_enabled, extraArgs=[name],
                )  # 사각형 체크박스: 누르면 command(체크 여부, 이름)
            else:
                OnscreenText(text=name, parent=self.left, pos=(0.11, y), align=TextNode.ALeft, fg=(1, 1, 1, 1), scale=0.045)
            self.rows[name] = OnscreenText(parent=self.left, pos=(0.11, y - 0.05), align=TextNode.ALeft, fg=(0.75, 0.8, 0.9, 1), scale=0.035, mayChange=True)
        self.hud = OnscreenText(parent=self.hud_anchor, pos=(0.04, 0.92), align=TextNode.ALeft, fg=(1, 1, 1, 1), scale=0.05, mayChange=True)
        OnscreenText(text=HELP, parent=self.hud_anchor, pos=(0.04, -0.95), align=TextNode.ALeft, fg=(0.7, 0.7, 0.7, 1), scale=0.04)
        self.base.taskMgr.add(self._update, "viz-update")

    # ---- 만들기 ------------------------------------------------------------------------------

    def _build_scene(self) -> None:
        self.camera = self.scene.attachNewNode(Camera("viz-cam"))
        self._lens3d = PerspectiveLens()
        self._lens3d.setFov(65)
        self._lens3d.setNear(0.02)
        self.camera.node().setLens(self._lens3d)
        region = self.win.makeDisplayRegion()
        region.setCamera(self.camera)
        region.setClearColor(LVecBase4f(0.05, 0.05, 0.08, 1))
        region.setClearColorActive(True)
        region.setClearDepthActive(True)
        self._make_2d()
        ambient = AmbientLight("ambient")
        ambient.setColor((0.6, 0.6, 0.65, 1))
        self.scene.setLight(self.scene.attachNewNode(ambient))
        sun = DirectionalLight("sun")
        sun.setColor((0.7, 0.7, 0.65, 1))
        sun_np = self.scene.attachNewNode(sun)
        sun_np.setHpr(-30, -50, 0)
        self.scene.setLight(sun_np)
        grid = LineSegs()
        grid.setColor(0.3, 0.33, 0.4, 1)
        for i in range(-10, 11):
            grid.moveTo(i * 0.5, -5, 0)
            grid.drawTo(i * 0.5, 5, 0)
            grid.moveTo(-5, i * 0.5, 0)
            grid.drawTo(5, i * 0.5, 0)
        self.grid_np = self.scene.attachNewNode(grid.create())
        self.grid_np.setLightOff()
        self.arrows: dict[str, NodePath] = {}
        for topic, color in ODOMETRY_ARROWS.items():
            arrow = LineSegs()
            arrow.setColor(*color)
            arrow.setThickness(3)
            for start, end in (((0, 0), (0.3, 0)), ((0.3, 0), (0.22, 0.05)), ((0.3, 0), (0.22, -0.05))):
                arrow.moveTo(start[0], start[1], 0.02)
                arrow.drawTo(end[0], end[1], 0.02)
            self.arrows[topic] = self.scene.attachNewNode(arrow.create())
            self.arrows[topic].setLightOff()
            self.arrows[topic].setBin("fixed", 0)  # 모델에 가려지지 않게
            self.arrows[topic].setDepthTest(False)
            self.arrows[topic].hide()

    def _make_2d(self) -> None:
        """글자와 체크박스를 올리는 2D 장면 (세로 +-1, 가로는 창 비율). PGTop이 DirectGUI 위젯의 마우스 영역을 이 창의 마우스 감시기에 연결한다."""
        root = NodePath("viz-render2d")
        root.setDepthTest(False)
        root.setDepthWrite(False)
        cam = root.attachNewNode(Camera("viz-cam2d"))
        self._lens2d = OrthographicLens()
        self._lens2d.setNearFar(-1000, 1000)
        cam.node().setLens(self._lens2d)
        region = self.win.makeDisplayRegion()
        region.setSort(10)
        region.setCamera(cam)
        self.aspect = root.attachNewNode(PGTop("viz-aspect2d"))
        self.left = self.aspect.attachNewNode("left")  # 창의 왼쪽 끝에 붙는 패널의 기준
        self.hud_anchor = self.aspect.attachNewNode("hud")  # 패널 오른쪽
        self._resize()

    def _resize(self) -> None:
        ratio = self.win.getXSize() / max(1, self.win.getYSize())
        self._lens2d.setFilmSize(2 * ratio, 2)
        self._lens3d.setAspectRatio(ratio)  # 3D 렌즈도 창 비율을 따라야 늘어나 보이지 않는다
        self.left.setX(-ratio)
        self.hud_anchor.setX(-ratio + PANEL_WIDTH)

    def _set_enabled(self, checked: int, name: str) -> None:
        self.enabled[name] = bool(checked)

    def _build_input(self) -> None:
        """이 창만의 마우스/키 입력 (ShowBase의 전역 입력과 섞이지 않게 'viz-' 접두사 이벤트로)."""
        self.mouse = NodePath(MouseWatcher("viz"))  # 오프스크린(GraphicsBuffer)은 입력 장치가 없어서 연결하지 않는다
        if isinstance(self.win, GraphicsWindow):
            mk = self.base.dataRoot.attachNewNode(MouseAndKeyboard(self.win, 0, "viz"))
            self.mouse.reparentTo(mk)
            self.mouse.attachNewNode(ButtonThrower("viz")).node().setPrefix("viz-")
        self.aspect.node().setMouseWatcher(self.mouse.node())
        self._events = DirectObject()  # 이벤트는 이 객체 이름으로 받는다 (base.accept로 받으면 ShowBase 자신의 window-event 처리를 덮어써서 Viewer가 창 이벤트를 못 받는다)
        self._events.accept("viz-wheel_up", self._zoom, [0.88])
        self._events.accept("viz-wheel_down", self._zoom, [1 / 0.88])
        self._events.accept("viz-escape", self.close)
        self._events.accept("window-event", self._window_event)

    # ---- 매 프레임 ---------------------------------------------------------------------------

    def _zoom(self, factor: float) -> None:
        self.dist = min(20.0, max(0.3, self.dist * factor))

    def _window_event(self, win: GraphicsWindow) -> None:
        if win is self.win and not win.isValid() and not self.own_base:
            self.close()

    def _lookup(self, frame: str) -> TransformStamped | None:
        try:
            return self.node.buffer.lookup_transform(self.fixed_frame, frame)
        except TransformException:
            return None

    def _update(self, task):  # type: ignore[no-untyped-def]
        if self._closed:
            return task.done
        self._camera()
        self._update_model()
        self._update_tf()
        self._update_scan()
        self._update_odometry()
        self.hud.setText(self._hud_text())
        self._resize()
        texts = {"grid": "grid", **self.node.topic_rows()}
        for name, row in self.rows.items():
            first, _, second = texts[name].partition("\n")
            row.setText(f"{first[0]} {second.strip()}" if second else "")  # "* LaserScan  n=..  ..Hz": 앞 글자는 최근에 받았는지
        self.grid_np.show() if self.enabled["grid"] else self.grid_np.hide()
        return task.cont

    def _over_gui(self) -> bool:
        return self._pointer_in_panel()

    def _pointer_in_panel(self) -> bool:
        mouse = self.mouse.node()
        ratio = self.win.getXSize() / max(1, self.win.getYSize())
        return bool(mouse.hasMouse() and (mouse.getMouseX() + 1) * ratio < PANEL_WIDTH)

    def _camera(self) -> None:
        mouse = self.mouse.node()
        pos = (mouse.getMouseX(), mouse.getMouseY()) if mouse.hasMouse() else None
        if pos and self._last_mouse and not self._over_gui():  # 패널 위의 드래그는 카메라를 움직이지 않는다
            dx, dy = pos[0] - self._last_mouse[0], pos[1] - self._last_mouse[1]
            if mouse.isButtonDown(MouseButton.one()):
                self.yaw -= dx * 3.0
                self.pitch = min(1.5, max(0.05, self.pitch - dy * 2.0))
            if mouse.isButtonDown(MouseButton.three()):
                k = self.dist * 0.8
                self.target -= Vec3(-math.sin(self.yaw), math.cos(self.yaw), 0) * dx * k + Vec3(-math.cos(self.yaw), -math.sin(self.yaw), 0) * dy * k
        self._last_mouse = pos
        offset = Vec3(math.cos(self.pitch) * math.cos(self.yaw), math.cos(self.pitch) * math.sin(self.yaw), math.sin(self.pitch)) * self.dist
        self.camera.setPos(self.target + offset)
        self.camera.lookAt(self.target)

    def _update_model(self) -> None:
        text = self.node.latest(ROBOT_DESCRIPTION_TOPIC_NAME)
        if text is not self.model_text and isinstance(text, String):
            self.model_text = text
            if self.model:
                self.model.root.removeNode()
            self.model = UrdfModel(text.data, self.package_root)
            for link in self.model.links.values():  # 링크를 트리에서 떼어 내 TF로 직접 놓는다
                link.reparentTo(self.scene)
                link.hide()
        if self.model:
            for name, link in self.model.links.items():
                stamped = self._lookup(name) if self.enabled[ROBOT_DESCRIPTION_TOPIC_NAME] else None
                if stamped is None:
                    link.hide()
                else:
                    _mat_of(stamped, link)
                    link.show()

    def _update_tf(self) -> None:
        """모든 프레임에 축(x 빨강, y 초록, z 파랑)과 부모로 가는 노란 선."""
        lines = LineSegs()
        for frame in self.node.buffer.all_frames() if self.enabled["tf"] else ():
            stamped = self._lookup(frame)
            if stamped is None:
                continue
            t, q = stamped.transform.translation, stamped.transform.rotation
            origin = Vec3(t.x, t.y, t.z)
            rotation = LQuaternionf(q.w, q.x, q.y, q.z)
            for axis, color in ((Vec3(1, 0, 0), (1, 0, 0, 1)), (Vec3(0, 1, 0), (0, 1, 0, 1)), (Vec3(0, 0, 1), (0.2, 0.4, 1, 1))):
                lines.setColor(*color)
                lines.moveTo(origin)
                lines.drawTo(origin + rotation.xform(axis) * 0.05)
            parent = self.node.buffer.parent_of(frame)
            parent_pose = self._lookup(parent) if parent else None
            if parent_pose is not None:
                p = parent_pose.transform.translation
                lines.setColor(1, 0.9, 0.2, 1)
                lines.moveTo(origin)
                lines.drawTo(Vec3(p.x, p.y, p.z))
        if self._tf_np is not None:
            self._tf_np.removeNode()
        self._tf_np = self.scene.attachNewNode(lines.create())
        self._tf_np.setLightOff()
        self._tf_np.setBin("fixed", 0)  # 모델에 가려지지 않게 위에 그린다
        self._tf_np.setDepthTest(False)

    def _update_scan(self) -> None:
        """scan의 점을 센서 프레임 기준으로 한 번 만들고, 센서의 TF에 맞춰 놓는다 (새 스캔일 때만 만든다)."""
        scan = self.node.latest("scan")
        if isinstance(scan, LaserScan) and scan is not self._scan_drawn:
            self._scan_drawn = scan
            vdata = GeomVertexData("scan", GeomVertexFormat.getV3(), Geom.UHStatic)
            vertex = GeomVertexWriter(vdata, "vertex")
            for i, r in enumerate(scan.ranges):
                if math.isfinite(r):
                    a = scan.angle_min + i * scan.angle_increment
                    vertex.addData3(r * math.cos(a), r * math.sin(a), 0)
            prim = GeomPoints(Geom.UHStatic)
            prim.addConsecutiveVertices(0, vdata.getNumRows())
            geom = Geom(vdata)
            geom.addPrimitive(prim)
            node = GeomNode("scan")
            node.addGeom(geom)
            if self._scan_np is not None:
                self._scan_np.removeNode()
            self._scan_np = self.scene.attachNewNode(node)
            self._scan_np.setLightOff()
            self._scan_np.setColor(1, 0.15, 0.15, 1)
            self._scan_np.setRenderModeThickness(4)
            self._scan_np.hide()
        if self._scan_np is not None:
            stamped = self._lookup(self.scan_frame) if self.enabled["scan"] else None
            if stamped is None:
                self._scan_np.hide()
            else:
                _mat_of(stamped, self._scan_np)
                self._scan_np.show()

    def _update_odometry(self) -> None:
        for topic, arrow in self.arrows.items():
            odom = self.node.latest(topic)
            if isinstance(odom, Odometry) and self.enabled[topic] and odom.header.frame_id == self.fixed_frame:
                p, q = odom.pose.pose.position, odom.pose.pose.orientation
                arrow.setPos(p.x, p.y, p.z)
                arrow.setQuat(LQuaternionf(q.w, q.x, q.y, q.z))
                arrow.show()
            else:
                arrow.hide()

    def _hud_text(self) -> str:
        imu = self.node.latest("imu")
        rows = [f"fixed frame: {self.fixed_frame}   frames: {len(self.node.buffer.all_frames())}"]
        for topic, label in (("odom", "odom"), ("odometry/filtered", "ekf ")):
            odom = self.node.latest(topic)
            if isinstance(odom, Odometry) and self.enabled[topic]:
                p = odom.pose.pose.position
                rows.append(f"{label} x={p.x:.2f} y={p.y:.2f}   v={odom.twist.twist.linear.x:.2f} w={odom.twist.twist.angular.z:.2f}")
        if isinstance(imu, Imu) and self.enabled["imu"]:
            a, w = imu.linear_acceleration, imu.angular_velocity
            rows.append(f"imu a=({a.x:.1f},{a.y:.1f},{a.z:.1f}) gyro_z={w.z:.2f}")
        return "\n".join(rows)

    # ---- 끝내기 ------------------------------------------------------------------------------

    def run(self) -> None:
        """자기 ShowBase를 쓸 때: 창을 닫을 때까지 돈다."""
        try:
            self.base.run()
        finally:
            self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._events.ignoreAll()
        for check in self.checks.values():
            check.destroy()
        if self.own_base:
            self.base.destroy()
        else:
            self.base.closeWindow(self.win)
