"""Viz 왼쪽 패널의 SLAM 칸: Cartographer 모드, 지도 저장/불러오기, 2D Pose Estimate.

viz가 직접 SLAM을 하거나 파일을 만지지 않는다. ROS에서 사람이 하는 일을 대신 눌러 줄 뿐이다:
  모드 바꾸기 = ros2 param set /cartographer mode ...      지도 저장 = map_saver/save_map 서비스
  지도 띄우기 = map_server/load_map 서비스 + localization 모드   2D Pose Estimate = /initialpose 토픽 (rviz와 같다)
"""
import math
import time
from collections.abc import Callable
from concurrent.futures import Future
from pathlib import Path
from typing import TYPE_CHECKING, Any

from direct.gui import DirectGuiGlobals as DGG
from direct.gui.DirectButton import DirectButton
from direct.gui.DirectOptionMenu import DirectOptionMenu
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import LineSegs, NodePath, TextNode

from kros.msgs.geometry_msgs import Point, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from kros.msgs.nav2_msgs import LoadMap, LoadMapRequest, LoadMapResponse, SaveMap, SaveMapRequest, SaveMapResponse
from kros.msgs.std_msgs import Header
from kros.nodes.cartographer.cartographer import INITIAL_POSE_TOPIC_NAME, compose, yaw_of
from kros.nodes.nav2.bt_navigator import GOAL_POSE_TOPIC_NAME
from kros.parameter import ParameterNotDeclaredError, ParameterRejectedError
from kros.service.service_type import Req, Res, ServiceType
from kros.tf import TransformException

if TYPE_CHECKING:
    from .viz import Viz

TOP = -0.17  # 패널에서 SLAM 칸이 시작하는 높이
NO_SELECTION = "(select map)"  # 지도 목록의 처음 상태: 아무것도 고르지 않았다 (사람이 고른다)
MAP_FILE = "map"  # 지도 폴더 안의 파일 이름: maps/<지도 이름>/map.yaml + map.pgm
TOOL_COLORS = {"initialpose": (0.3, 1.0, 0.4, 1.0), "goal": (1.0, 0.35, 0.8, 1.0)}  # 드래그 중 미리 보는 화살표 색 (rviz처럼 도구마다)
TOOL_ACTIVE = (0.6, 0.35, 0.1, 1)  # 켜진 도구 버튼 색
INITIAL_POSE_COVARIANCE = tuple(0.25 if i in (0, 7) else 0.0685 if i == 35 else 0.0 for i in range(36))  # rviz 2D Pose Estimate의 기본 공분산
BUTTON_COLOR = (0.26, 0.31, 0.44, 1)
# 입체 테두리(RAISED: 누르는 동안 SUNKEN으로 들어간다), 글자 둘레 여백
GUI = {"scale": 0.04, "text_fg": (1, 1, 1, 1), "frameColor": BUTTON_COLOR, "relief": DGG.RAISED, "borderWidth": (0.12, 0.12), "pad": (0.3, 0.25)}


def _button(parent: object, text: str, left: float, right: float, y: float, command: Callable[[], None], scale: float = 0.04) -> DirectButton:
    """패널 좌표 left~right 폭을 꽉 채운 버튼 (글자는 가운데). 같은 줄/열의 버튼 끝이 맞는다."""
    half = (right - left) / 2 / scale
    return DirectButton(parent=parent, text=text, pos=((left + right) / 2, 0, y), frameSize=(-half, half, -0.5, 1.0), command=command, **{**GUI, "pad": (0, 0), "scale": scale})


def _arrow(color: tuple[float, float, float, float]) -> NodePath:
    """길이 1의 바닥 화살표 (+x 방향). 드래그하는 동안 위치/방향을 미리 보여 준다."""
    segs = LineSegs()
    segs.setColor(*color)
    segs.setThickness(4)
    for start, end in (((0, 0), (1, 0)), ((1, 0), (0.8, 0.12)), ((1, 0), (0.8, -0.12))):
        segs.moveTo(start[0], start[1], 0.05)
        segs.drawTo(end[0], end[1], 0.05)
    node = NodePath(segs.create())
    node.setLightOff()
    node.setBin("fixed", 2)
    node.setDepthTest(False)
    node.hide()
    return node


class SlamPanel:
    def __init__(self, viz: "Viz", cartographer: str = "cartographer", map_server: str = "map_server", map_saver: str = "map_saver") -> None:
        self.viz, self.node = viz, viz.node
        self.cartographer, self.map_server, self.map_saver = cartographer, map_server, map_saver
        self.tool: str | None = None  # "initialpose"(2D Pose Estimate) 또는 "goal"(2D Goal Pose): 켜지면 3D 화면의 왼쪽 드래그가 카메라 대신 위치/방향을 찍는다
        self.mode = ""  # Cartographer의 지금 모드 (매 프레임 갱신)
        self._drag_start: tuple[float, float] | None = None
        self._preview = {tool: _arrow(color) for tool, color in TOOL_COLORS.items()}
        for arrow in self._preview.values():
            arrow.reparentTo(viz.scene)
        self._pending: list[tuple[Future[Any], Callable[[object], None]]] = []  # 응답을 기다리는 서비스 호출 (UI는 매 프레임 확인한다)
        left = viz.left
        OnscreenText(text="SLAM", parent=left, pos=(0.04, TOP), align=TextNode.ALeft, fg=(0.6, 0.8, 1, 1), scale=0.045)
        self.mode_text = OnscreenText(parent=left, pos=(0.2, TOP), align=TextNode.ALeft, fg=(1, 0.75, 0.3, 1), scale=0.04, mayChange=True)
        self.slam_button = _button(left, "start SLAM", 0.06, 0.32, TOP - 0.09, self.toggle_slam)
        self.save_button = _button(left, "save map", 0.36, 0.62, TOP - 0.09, self.save_map)
        self.map_menu = DirectOptionMenu(
            parent=left, items=[NO_SELECTION, *self.map_names()], initialitem=0, pos=(0.075, 0, TOP - 0.19), highlightColor=(0.3, 0.4, 0.6, 1), item_text_fg=(1, 1, 1, 1), item_frameColor=(0.2, 0.24, 0.34, 1),
            **{**GUI, "frameColor": (0.12, 0.14, 0.2, 1), "relief": DGG.SUNKEN},  # 고르는 칸은 들어간 모양 (버튼과 구별)
        )
        self.map_menu["frameSize"] = (-0.4, (0.44 - 0.075) / GUI["scale"] - 0.4, -0.5, 1.0)  # 지도 이름이 길다
        self.load_button = _button(left, "load", 0.47, 0.62, TOP - 0.19, self.load_map)
        self.pose_button = _button(left, "2D Pose Estimate", 0.06, 0.33, TOP - 0.29, lambda: self.toggle_tool("initialpose"), scale=0.032)
        self.goal_button = _button(left, "2D Goal Pose", 0.36, 0.62, TOP - 0.29, lambda: self.toggle_tool("goal"), scale=0.032)
        self.status = OnscreenText(parent=left, pos=(0.07, TOP - 0.38), align=TextNode.ALeft, fg=(0.75, 0.8, 0.9, 1), scale=0.033, mayChange=True)

    # ---- 버튼 -----------------------------------------------------------------------------------

    def _get_mode(self) -> str:
        try:
            return str(self.node.context.parameter_manager.get(self.cartographer, "mode"))
        except ParameterNotDeclaredError:
            return ""

    def set_mode(self, mode: str) -> None:
        """ros2 param set /cartographer mode <mode>."""
        try:
            self.node.context.parameter_manager.set(self.cartographer, "mode", mode)
            self.status.setText("")
        except (ParameterNotDeclaredError, ParameterRejectedError):
            self.status.setText(f"no node '{self.cartographer}'")

    def toggle_slam(self) -> None:
        """start SLAM = mapping (새 지도), stop SLAM = idle (만든 지도는 map 토픽에 남아 save map으로 저장할 수 있다)."""
        self.set_mode("idle" if self._get_mode() == "mapping" else "mapping")

    @property
    def maps_directory(self) -> Path:
        """지도 폴더는 map_saver 노드의 maps_directory 파라미터를 따른다 (경로를 한 곳에서만 정한다). map_saver가 없으면 작업 폴더."""
        try:
            return Path(str(self.node.context.parameter_manager.get(self.map_saver, "maps_directory"))).expanduser()
        except ParameterNotDeclaredError:
            return Path()

    def map_names(self) -> list[str]:
        """지도 폴더들 (maps/<이름>/map.yaml이 있는 것)."""
        return sorted(path.parent.name for path in self.maps_directory.glob(f"*/{MAP_FILE}.yaml"))

    def save_map(self) -> None:
        name = time.strftime("map_%Y%m%d_%H%M%S")
        self.status.setText(f"saving {name} ...")
        url = str(self.maps_directory / name / MAP_FILE)  # map_saver가 폴더를 만들고 map.yaml + map.pgm을 쓴다
        self._call(f"{self.map_saver}/save_map", SaveMap, SaveMapRequest(map_url=url), lambda response: self._saved(name, response))

    def _saved(self, name: str, response: object) -> None:
        ok = isinstance(response, SaveMapResponse) and response.result
        self.status.setText(f"saved {name}" if ok else "save failed: no map yet (start SLAM first)")
        self.refresh_maps()

    def refresh_maps(self) -> None:
        """지도 목록을 다시 읽는다. 고른 것은 그대로 두고, 없어졌으면 '고르지 않음'으로."""
        chosen = self.map_menu.get()
        items = [NO_SELECTION, *self.map_names()]
        self.map_menu["items"] = items
        self.map_menu.set(items.index(chosen) if chosen in items else 0, fCommand=False)

    def load_map(self) -> None:
        name = self.map_menu.get()
        if name == NO_SELECTION or name not in self.map_names():
            self.status.setText("select a map first")
            return
        self.status.setText(f"loading {name} ...")
        request = LoadMapRequest(str(self.maps_directory / name / f"{MAP_FILE}.yaml"))
        self._call(f"{self.map_server}/load_map", LoadMap, request, lambda response: self._loaded(name, response))

    def _loaded(self, name: str, response: object) -> None:
        if not (isinstance(response, LoadMapResponse) and response.result == LoadMapResponse.RESULT_SUCCESS):
            self.status.setText(f"load failed: {name}")
            return
        self.set_mode("localization")  # 저장한 지도로 위치만 잡는다
        self.status.setText(f"loaded {name}: set the robot with 2D Pose Estimate")

    def _call(self, service: str, srv_type: ServiceType[Req, Res], request: Req, done: Callable[[object], None]) -> None:
        try:
            future: Future[Any] = self.node.call_service_async(service, srv_type, request)
        except LookupError:  # 그 노드를 안 띄웠다
            self.status.setText(f"no service {service}")
            return
        self._pending.append((future, done))

    def toggle_tool(self, tool: str) -> None:
        """2D Pose Estimate / 2D Goal Pose 버튼 (rviz처럼: 누른 곳이 위치, 끄는 방향이 yaw). 같은 버튼을 다시 누르면 끈다."""
        if self.tool != tool and tool == "goal" and self._get_mode() != "localization":
            self.status.setText("2D Goal Pose: load a map first (localization)")  # 목표는 저장한 지도에서 위치를 잡은 뒤에 찍는다
            return
        self.tool = None if self.tool == tool else tool
        self._drag_start = None
        self.pose_button["frameColor"] = TOOL_ACTIVE if self.tool == "initialpose" else BUTTON_COLOR
        self.goal_button["frameColor"] = TOOL_ACTIVE if self.tool == "goal" else BUTTON_COLOR
        self.status.setText("press = position, drag = heading" if self.tool else "")

    def toggle_pose_tool(self) -> None:
        self.toggle_tool("initialpose")

    @property
    def pose_tool(self) -> bool:
        """어떤 도구든 켜져 있다 (그동안 왼쪽 드래그는 카메라를 돌리지 않는다)."""
        return self.tool is not None

    # ---- 매 프레임 --------------------------------------------------------------------------------

    def update(self, ground: tuple[float, float] | None, pressed: bool) -> None:
        """Viz가 매 프레임 부른다: 모드 표시, 끝난 서비스 응답, 2D Pose Estimate 드래그 (ground: 마우스 아래 바닥점, fixed frame)."""
        self.mode = mode = self._get_mode()  # 다른 곳(ros2 param set 같은)에서 바꿔도 따라간다
        self.mode_text.setText(mode or "(no cartographer)")
        mapping = mode == "mapping"
        self.slam_button["text"] = "stop SLAM" if mapping else "start SLAM"
        self.slam_button["frameColor"] = (0.6, 0.2, 0.2, 1) if mapping else (0.15, 0.45, 0.25, 1)
        for future, done in [item for item in self._pending if item[0].done()]:
            self._pending.remove((future, done))
            error = future.exception()
            if error is None:
                done(future.result())
            else:  # 핸들러가 던진 예외: 화면에 알리고 계속 돈다
                self.status.setText(f"{type(error).__name__}: {error}")
        tool = self.tool
        for name, arrow in self._preview.items():
            if name != tool or self._drag_start is None:
                arrow.hide()
        if tool is None:
            return
        if pressed and ground is not None and self._drag_start is None:
            self._drag_start = ground
        start = self._drag_start
        if start is None:
            return
        end = ground or start
        yaw = math.atan2(end[1] - start[1], end[0] - start[0]) if math.dist(start, end) > 1e-3 else 0.0
        if pressed:  # 끄는 동안: 위치와 방향을 화살표로 미리 보인다
            arrow = self._preview[tool]
            arrow.setPos(start[0], start[1], 0)
            arrow.setH(math.degrees(yaw))
            arrow.setScale(max(0.3, math.dist(start, end)))
            arrow.show()
            return
        self._drag_start = None
        self._preview[tool].hide()
        if tool == "initialpose":
            self.publish_initial_pose((start[0], start[1], yaw))
        else:
            self.publish_goal((start[0], start[1], yaw))
        self.toggle_tool(tool)  # 한 번 찍으면 끈다 (rviz와 같다)

    def _in_map(self, pose: tuple[float, float, float]) -> tuple[float, float, float] | None:
        """fixed frame에서 찍은 위치를 map 좌표로 (Cartographer/nav2는 map 좌표로 받는다). map TF가 없으면 None."""
        fixed = self.viz.fixed_frame
        if fixed == "map":
            return pose
        try:
            t = self.node.buffer.lookup_transform("map", fixed).transform
        except TransformException:
            self.status.setText("no map TF: start SLAM or load a map")
            return None
        return compose((t.translation.x, t.translation.y, yaw_of(t.rotation)), pose)

    def publish_goal(self, pose: tuple[float, float, float]) -> None:
        """2D Goal Pose: /goal_pose (PoseStamped, map). bt_navigator가 받아 계획하고 따라간다."""
        in_map = self._in_map(pose)
        if in_map is None:
            return
        x, y, yaw = in_map
        self.node.publish(GOAL_POSE_TOPIC_NAME, PoseStamped(Header(self.node.now(), "map"), Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))))
        self.status.setText(f"goal ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg)")

    def publish_initial_pose(self, pose: tuple[float, float, float]) -> None:
        """2D Pose Estimate: /initialpose (PoseWithCovarianceStamped, map)."""
        in_map = self._in_map(pose)
        if in_map is None:
            return
        pose = in_map
        x, y, yaw = pose
        orientation = Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        self.node.publish(INITIAL_POSE_TOPIC_NAME, PoseWithCovarianceStamped(Header(self.node.now(), "map"), PoseWithCovariance(Pose(Point(x, y, 0.0), orientation), INITIAL_POSE_COVARIANCE)))
        self.status.setText(f"initialpose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg)")

    def destroy(self) -> None:
        for arrow in self._preview.values():
            arrow.removeNode()
        for widget in (self.goal_button, self.slam_button, self.save_button, self.map_menu, self.load_button, self.pose_button):
            widget.destroy()
