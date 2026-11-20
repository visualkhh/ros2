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

from direct.gui.DirectButton import DirectButton
from direct.gui.DirectOptionMenu import DirectOptionMenu
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode

from kros.msgs.geometry_msgs import Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from kros.msgs.nav2_msgs import LoadMap, LoadMapRequest, LoadMapResponse, SaveMap, SaveMapRequest, SaveMapResponse
from kros.msgs.std_msgs import Header
from kros.nodes.cartographer.cartographer import INITIAL_POSE_TOPIC_NAME, compose, yaw_of
from kros.nodes.map_server import MAPS_DIRECTORY
from kros.parameter import ParameterNotDeclaredError, ParameterRejectedError
from kros.service.service_type import Req, Res, ServiceType
from kros.tf import TransformException

if TYPE_CHECKING:
    from .viz import Viz

TOP = -0.33  # 패널에서 SLAM 칸이 시작하는 높이
INITIAL_POSE_COVARIANCE = tuple(0.25 if i in (0, 7) else 0.0685 if i == 35 else 0.0 for i in range(36))  # rviz 2D Pose Estimate의 기본 공분산
GUI = {"scale": 0.04, "text_fg": (1, 1, 1, 1), "frameColor": (0.2, 0.24, 0.34, 1), "relief": 1}


class SlamPanel:
    def __init__(self, viz: "Viz", cartographer: str = "cartographer", map_server: str = "map_server", map_saver: str = "map_saver", maps_directory: Path = MAPS_DIRECTORY) -> None:
        self.viz, self.node = viz, viz.node
        self.cartographer, self.map_server, self.map_saver, self.maps_directory = cartographer, map_server, map_saver, maps_directory
        self.pose_tool = False  # 2D Pose Estimate: 켜지면 3D 화면의 왼쪽 드래그가 카메라 대신 위치/방향을 찍는다
        self._drag_start: tuple[float, float] | None = None
        self._pending: list[tuple[Future[Any], Callable[[object], None]]] = []  # 응답을 기다리는 서비스 호출 (UI는 매 프레임 확인한다)
        left = viz.left
        OnscreenText(text="SLAM", parent=left, pos=(0.04, TOP), align=TextNode.ALeft, fg=(0.6, 0.8, 1, 1), scale=0.045)
        self.mode_text = OnscreenText(parent=left, pos=(0.2, TOP), align=TextNode.ALeft, fg=(1, 0.75, 0.3, 1), scale=0.04, mayChange=True)
        self.slam_button = DirectButton(parent=left, text="start SLAM", pos=(0.07, 0, TOP - 0.08), command=self.toggle_slam, text_align=TextNode.ALeft, **GUI)
        self.save_button = DirectButton(parent=left, text="save map", pos=(0.4, 0, TOP - 0.08), command=self.save_map, text_align=TextNode.ALeft, **GUI)
        maps = self.map_names()
        self.map_menu = DirectOptionMenu(
            parent=left, items=maps or ["(no maps)"], pos=(0.07, 0, TOP - 0.17), highlightColor=(0.3, 0.4, 0.6, 1), item_text_fg=(1, 1, 1, 1), item_frameColor=(0.2, 0.24, 0.34, 1), **GUI,
        )
        self.load_button = DirectButton(parent=left, text="load", pos=(0.52, 0, TOP - 0.17), command=self.load_map, text_align=TextNode.ALeft, **GUI)
        self.pose_button = DirectButton(parent=left, text="2D Pose Estimate", pos=(0.07, 0, TOP - 0.26), command=self.toggle_pose_tool, text_align=TextNode.ALeft, **GUI)
        self.status = OnscreenText(parent=left, pos=(0.07, TOP - 0.34), align=TextNode.ALeft, fg=(0.75, 0.8, 0.9, 1), scale=0.035, mayChange=True)

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

    def map_names(self) -> list[str]:
        return sorted(path.stem for path in self.maps_directory.glob("*.yaml"))

    def save_map(self) -> None:
        name = time.strftime("map_%Y%m%d_%H%M%S")
        self.status.setText(f"saving {name} ...")
        self._call(f"{self.map_saver}/save_map", SaveMap, SaveMapRequest(map_url=str(self.maps_directory / name)), lambda response: self._saved(name, response))

    def _saved(self, name: str, response: object) -> None:
        ok = isinstance(response, SaveMapResponse) and response.result
        self.status.setText(f"saved {name}" if ok else "save failed: no map yet (start SLAM first)")
        maps = self.map_names()
        if maps:
            self.map_menu["items"] = maps
            self.map_menu.set(maps.index(name) if name in maps else 0, fCommand=False)

    def load_map(self) -> None:
        name = self.map_menu.get()
        if name not in self.map_names():
            self.status.setText("no map to load")
            return
        self.status.setText(f"loading {name} ...")
        self._call(f"{self.map_server}/load_map", LoadMap, LoadMapRequest(str(self.maps_directory / f"{name}.yaml")), lambda response: self._loaded(name, response))

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

    def toggle_pose_tool(self) -> None:
        self.pose_tool = not self.pose_tool
        self._drag_start = None
        self.pose_button["frameColor"] = (0.6, 0.35, 0.1, 1) if self.pose_tool else GUI["frameColor"]
        self.status.setText("drag on the floor: position -> heading" if self.pose_tool else "")

    # ---- 매 프레임 --------------------------------------------------------------------------------

    def update(self, ground: tuple[float, float] | None, pressed: bool) -> None:
        """Viz가 매 프레임 부른다: 모드 표시, 끝난 서비스 응답, 2D Pose Estimate 드래그 (ground: 마우스 아래 바닥점, fixed frame)."""
        mode = self._get_mode()  # 다른 곳(ros2 param set 같은)에서 바꿔도 따라간다
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
        if not self.pose_tool:
            return
        if pressed and ground is not None and self._drag_start is None:
            self._drag_start = ground
        elif not pressed and self._drag_start is not None:
            start, end = self._drag_start, ground or self._drag_start
            self._drag_start = None
            yaw = math.atan2(end[1] - start[1], end[0] - start[0]) if end != start else 0.0
            self.publish_initial_pose((start[0], start[1], yaw))
            self.toggle_pose_tool()

    def publish_initial_pose(self, pose: tuple[float, float, float]) -> None:
        """fixed frame에서 찍은 위치를 map 좌표로 바꿔 /initialpose로 낸다 (Cartographer는 map 좌표만 받는다)."""
        fixed = self.viz.fixed_frame
        if fixed != "map":
            try:
                t = self.node.buffer.lookup_transform("map", fixed).transform
            except TransformException:
                self.status.setText("no map TF: start SLAM or load a map")
                return
            pose = compose((t.translation.x, t.translation.y, yaw_of(t.rotation)), pose)
        x, y, yaw = pose
        orientation = Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        self.node.publish(INITIAL_POSE_TOPIC_NAME, PoseWithCovarianceStamped(Header(self.node.now(), "map"), PoseWithCovariance(Pose(Point(x, y, 0.0), orientation), INITIAL_POSE_COVARIANCE)))
        self.status.setText(f"initialpose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg)")

    def destroy(self) -> None:
        for widget in (self.slam_button, self.save_button, self.map_menu, self.load_button, self.pose_button):
            widget.destroy()
