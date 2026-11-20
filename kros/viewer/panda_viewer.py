import math
from collections.abc import Callable

from direct.gui.OnscreenText import OnscreenText
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight,
    BitMask32,
    ClockObject,
    DirectionalLight,
    Geom,
    GeomNode,
    GeomPoints,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    LineSegs,
    NodePath,
    TextNode,
    Vec3,
    loadPrcFileData,
)

from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.objects import Puck, Wall
from kros.worlds.maze.track import CENTER, SIZE

from kros.viewer.simulation_runner import SimulationRunner
from kros.viewer.snapshot import Snapshot
from .shapes import box, cylinder
from .urdf_model import UrdfModel

SHADOW_CAMERA = BitMask32.bit(5)
HELP = "w/a/s/d, arrows: drive    q/z: speed up/down    left drag: orbit    right drag: pan    wheel: zoom    l: lidar points    f: follow on/off    c: overview    esc: quit"
VIEWS = {  # (yaw, pitch, dist): follow는 로봇 뒤쪽 기준 상대 yaw, overview는 월드 yaw
    True: (0.0, math.atan2(0.6, 1.0), 1.17),
    False: (-math.pi / 2, 1.2, 8.5),
}
ARROWS = {"arrow_up": "up", "arrow_down": "down", "arrow_left": "left", "arrow_right": "right"}


class Viewer(ShowBase):  # type: ignore[misc]  # panda3d는 타입 정보가 없다
    """Panda3D 3D 창(GPU 렌더링): 키 입력 -> on_keys, 매 프레임 runner.latest(Snapshot)를 그린다 (월드는 따로 돈다)."""

    def __init__(self, runner: SimulationRunner, on_keys: Callable[[frozenset[str]], None]) -> None:
        loadPrcFileData("", "window-title kros TurtleBot3 simulator\nframebuffer-multisample 1\nmultisamples 4\nsync-video 1")
        super().__init__()
        self.runner = runner  # 화면은 runner.latest(Snapshot)만 읽는다. 월드는 만지지 않는다
        self.world, self.robot_hw = runner.world, runner.robot  # 장면을 처음 만들 때만 쓴다 (모양, 크기, URDF)
        self.snap: Snapshot = runner.latest
        self.on_keys = on_keys  # 창이 받은 키를 바깥으로 넘기는 통로
        self._scan_drawn: object = None
        self._scan_np: NodePath | None = None
        self.keys: set[str] = set()
        self.follow = True
        self.yaw, self.pitch, self.dist = VIEWS[True]
        self.show_scan = False
        self.panning = False
        self.az = 0.0
        self.pan = Vec3(0, 0, 0)  # 카메라 목표점에 더하는 이동량
        self.dragging = False
        self.camera_target = Vec3(0, 0, 0.08)
        self.last_mouse: tuple[float, float] | None = None
        self.disableMouse()
        self.setBackgroundColor(0.04, 0.05, 0.08)
        self.camLens.setFov(65)
        self.camLens.setNear(0.02)
        self._build_lights()
        self._pucks: list[tuple[int, Puck, NodePath]] = []
        self._build_world()
        self._build_robot()
        self.hud = OnscreenText(
            parent=self.a2dTopLeft, pos=(0.04, -0.08), align=TextNode.ALeft, fg=(1, 1, 1, 1), scale=0.05, mayChange=True
        )
        OnscreenText(text=HELP, parent=self.a2dBottomLeft, pos=(0.04, 0.04), align=TextNode.ALeft, fg=(0.7, 0.7, 0.7, 1), scale=0.04)
        for key in "wasdqz":
            self.accept(key, self.keys.add, [key])
            self.accept(f"{key}-up", self.keys.discard, [key])
        for panda_key, name in ARROWS.items():
            self.accept(panda_key, self.keys.add, [name])
            self.accept(f"{panda_key}-up", self.keys.discard, [name])
        self.accept("f", self._toggle_follow)
        self.accept("c", self._overview)
        self.accept("l", self._toggle_scan)
        self.accept("mouse1", self._drag, [True])
        self.accept("mouse1-up", self._drag, [False])
        self.accept("mouse3", self._pan, [True])
        self.accept("mouse3-up", self._pan, [False])
        self.accept("wheel_up", self._zoom, [0.88])
        self.accept("wheel_down", self._zoom, [1 / 0.88])
        self.accept("escape", self.userExit)
        self.taskMgr.add(self._update, "update")
        print(f"GPU renderer: {self.win.getGsg().getDriverRenderer()}")

    def windowEvent(self, win) -> None:  # type: ignore[no-untyped-def]
        super().windowEvent(win)
        if not win.getProperties().getForeground():
            self.keys.clear()  # 포커스를 잃으면 key-up이 안 오므로 정지
            self.dragging = self.panning = False

    # ---- 장면 만들기 -------------------------------------------------------------------------

    def _part(self, parent: NodePath, shape: NodePath, color: tuple[float, float, float], pos: tuple[float, float, float], hpr: tuple[float, float, float] = (0, 0, 0)) -> NodePath:
        shape.reparentTo(parent)
        shape.setColor(*color, 1)
        shape.setPos(*pos)
        shape.setHpr(*hpr)
        return shape

    def _build_lights(self) -> None:
        ambient = AmbientLight("ambient")
        ambient.setColor((0.38, 0.4, 0.45, 1))
        self.render.setLight(self.render.attachNewNode(ambient))
        sun = DirectionalLight("sun")
        sun.setColor((0.95, 0.92, 0.85, 1))
        sun.setShadowCaster(True, 2048, 2048)
        sun.setCameraMask(SHADOW_CAMERA)  # 이 마스크로만 그림자 카메라에 보인다 (라이다 점을 그림자에서 빼려고)
        sun.getLens().setFilmSize(9, 9)
        sun.getLens().setNearFar(1, 20)
        sun_np = self.render.attachNewNode(sun)
        sun_np.setPos(CENTER[0] - 3, CENTER[1] - 2, 8)
        sun_np.lookAt(CENTER[0], CENTER[1], 0)
        self.render.setLight(sun_np)
        self.render.setShaderAuto()  # 픽셀 단위 조명 + 그림자 (GPU 셰이더)

    def _build_world(self) -> None:
        world = self.render.attachNewNode("world")
        cx, cy = CENTER
        self._part(world, box(40, 40, 0.02), (0.07, 0.08, 0.1), (cx, cy, -0.03))  # 방 밖 바닥
        self._part(world, box(SIZE[0] + 0.1, SIZE[1] + 0.1, 0.02), (0.22, 0.24, 0.27), (cx, cy, -0.01))
        grid = LineSegs()
        grid.setColor(0.4, 0.44, 0.5, 1)
        for i in range(int(SIZE[0] / 2 / 0.5) + 1):
            for m in {i * 0.5, -i * 0.5}:
                grid.moveTo(cx + m, cy - SIZE[1] / 2, 0.001)
                grid.drawTo(cx + m, cy + SIZE[1] / 2, 0.001)
        for i in range(int(SIZE[1] / 2 / 0.5) + 1):
            for m in {i * 0.5, -i * 0.5}:
                grid.moveTo(cx - SIZE[0] / 2, cy + m, 0.001)
                grid.drawTo(cx + SIZE[0] / 2, cy + m, 0.001)
        world.attachNewNode(grid.create())
        for obj in self.world.objects:  # 벽도 월드 객체
            if isinstance(obj, Wall):
                self._part(world, box(obj.size_x, obj.size_y, obj.height), (0.8, 0.82, 0.86), (obj.pose.x, obj.pose.y, obj.height / 2))
        for index, obj in enumerate(self.world.objects):  # 월드에 놓인 퍽들
            if isinstance(obj, Puck):
                p = obj.pose
                shape = cylinder(obj.radius, obj.height, obj.shape.value)
                self._pucks.append((index, obj, self._part(world, shape, obj.color, (p.x, p.y, obj.height / 2), (math.degrees(p.theta), 0, 0))))

    def _build_robot(self) -> None:
        """Robot이 알려 주는 URDF를 읽어 만든다. 앞쪽이 +x."""
        self.robot = self.render.attachNewNode("robot")
        self.model = UrdfModel(self.robot_hw.urdf_path, self.robot_hw.description_dir)
        self.model.root.reparentTo(self.robot)

    # ---- 매 프레임 ---------------------------------------------------------------------------

    def _toggle_follow(self) -> None:
        """f: 로봇 추적 켜기/끄기. 끄면 지금 보던 자리 그대로 고정(마우스로 옮긴 위치 유지), 켜면 지금 각도로 로봇을 다시 따라간다."""
        p = self.snap.poses[self.snap.robot_index]
        if self.follow:
            self.camera_target = self.camera_target + self.pan  # 지금 바라보던 점을 고정
            self.yaw = self.az  # 월드 기준 각도로 변환
        else:
            self.yaw = self.az - p.theta - math.pi  # 로봇 기준 각도로 변환
        self.pan = Vec3(0, 0, 0)
        self.follow = not self.follow

    def _overview(self) -> None:
        """c: 트랙 전체가 보이는 시점으로 (추적은 꺼짐)."""
        self.follow = False
        self.yaw, self.pitch, self.dist = VIEWS[False]
        self.camera_target = Vec3(CENTER[0], CENTER[1], 0)
        self.pan = Vec3(0, 0, 0)

    def _toggle_scan(self) -> None:
        self.show_scan = not self.show_scan
        if self._scan_np is not None:
            self._scan_np.show() if self.show_scan else self._scan_np.hide()

    def _drag(self, on: bool) -> None:
        self.dragging = on

    def _pan(self, on: bool) -> None:
        self.panning = on

    def _zoom(self, factor: float) -> None:
        self.dist = min(12.0, max(0.3, self.dist * factor))

    def _update(self, task):  # type: ignore[no-untyped-def]
        dt = min(ClockObject.getGlobalClock().getDt(), 0.1)
        self.on_keys(frozenset(self.keys))
        self.snap = snap = self.runner.latest  # 월드는 따로 돌고 있다. 화면은 가장 최근 스냅샷을 그리기만 한다
        p, cmd = snap.poses[snap.robot_index], snap.command
        self.robot.setPos(p.x, p.y, 0)
        self.robot.setH(math.degrees(p.theta))
        v, w = cmd.linear.x, cmd.angular.z
        for joint, angle in zip(("wheel_left_joint", "wheel_right_joint"), snap.wheel_angles):  # 바퀴 모터가 실제로 돈 각도
            self.model.set_joint_angle(joint, angle)
        for index, puck, node in self._pucks:  # 퍽은 로봇이 밀면 움직인다
            q = snap.poses[index]
            node.setPos(q.x, q.y, puck.height / 2)
        self._update_scan()
        self._update_camera(dt)
        nearest = min(snap.scan.ranges) if snap.scan else math.inf
        self.hud.setText(f"x={p.x:.2f} y={p.y:.2f} th={math.degrees(p.theta):.0f}deg   v={v:.2f} w={w:.2f}   nearest={nearest:.2f}m")
        return task.cont

    def _update_scan(self) -> None:
        """LidarNode의 최신 스캔(5Hz)을 월드 좌표의 빨간 점으로 그린다."""
        scan, sensor = self.snap.scan, self.snap.scan_pose
        if scan is None or sensor is None or scan is self._scan_drawn:
            return
        self._scan_drawn = scan
        vdata = GeomVertexData("scan", GeomVertexFormat.getV3(), Geom.UHStatic)
        vertex = GeomVertexWriter(vdata, "vertex")
        for i, r in enumerate(scan.ranges):
            if math.isinf(r):
                continue
            a = sensor.theta + scan.angle_min + i * scan.angle_increment
            r -= 0.01  # 벽면에 파묻혀 가려지지 않게 센서 쪽으로 1cm
            vertex.addData3(sensor.x + r * math.cos(a), sensor.y + r * math.sin(a), self.robot_hw.lidar.mount_height)
        prim = GeomPoints(Geom.UHStatic)
        prim.addConsecutiveVertices(0, vdata.getNumRows())
        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode("scan")
        node.addGeom(geom)
        if self._scan_np is not None:
            self._scan_np.removeNode()
        self._scan_np = self.render.attachNewNode(node)
        self._scan_np.setLightOff()  # 조명 영향 없이 밝은 빨강
        self._scan_np.setColor(1, 0.15, 0.15, 1)
        self._scan_np.setRenderModeThickness(3)  # 점 크기(픽셀)
        self._scan_np.hide(SHADOW_CAMERA)  # 점은 그림자를 만들지 않는다
        if not self.show_scan:
            self._scan_np.hide()

    def _update_camera(self, dt: float) -> None:
        mouse = self.mouseWatcherNode
        pos = (mouse.getMouseX(), mouse.getMouseY()) if mouse and mouse.hasMouse() else None
        if self.dragging and pos and self.last_mouse:
            self.yaw -= (pos[0] - self.last_mouse[0]) * 3.0
            self.pitch = min(1.5, max(0.05, self.pitch - (pos[1] - self.last_mouse[1]) * 2.0))
        if self.panning and pos and self.last_mouse:  # 바닥 평면에서 화면 오른쪽/위쪽 방향으로 이동 (멀수록 크게)
            k = self.dist * 0.8
            right = Vec3(-math.sin(self.az), math.cos(self.az), 0)
            forward = Vec3(-math.cos(self.az), -math.sin(self.az), 0)  # 카메라가 보는 방향(바닥 투영)
            self.pan -= right * (pos[0] - self.last_mouse[0]) * k + forward * (pos[1] - self.last_mouse[1]) * k  # 잡아끄는 느낌
        self.last_mouse = pos
        p = self.snap.poses[self.snap.robot_index]
        if self.follow:
            want = Vec3(p.x, p.y, 0.08)
            here = Vec3(self.camera_target)
            self.camera_target = here + (want - here) * min(1.0, dt * 6)
            azimuth = self.yaw + p.theta + math.pi  # yaw 0 = 로봇 뒤쪽
        else:
            azimuth = self.yaw  # 추적을 끄면 camera_target은 그대로 둔다
        self.az = azimuth
        focus = self.camera_target + self.pan
        offset = Vec3(math.cos(self.pitch) * math.cos(azimuth), math.cos(self.pitch) * math.sin(azimuth), math.sin(self.pitch)) * self.dist
        self.camera.setPos(focus + offset)
        self.camera.lookAt(focus)
