"""지도 목록/변경/삭제 (kros에만 있는 노드). 원격 Viz가 로봇의 지도 폴더를 직접 못 보므로 토픽으로 주고받는다.

- `map_list` (std_msgs/String, latch): 지도 이름들을 줄바꿈으로 이어 붙여 낸다. 폴더가 바뀌면 다시 낸다 (1초마다 확인).
- `map_change` (std_msgs/String): 이름이 있으면 그 지도로 바꾼다 (map_server/load_map + cartographer localization).
  빈 문자열이면 **지금 띄운 지도**를 지운다 (map_server/unload_map 후 폴더 삭제).
- `map_status` (std_msgs/String, latch): 마지막 결과 ("loaded map_x" / "deleted map_x" / "no map loaded" ...).

지도 하나 = maps_directory/<이름>/map.yaml + map.pgm (map_saver와 같은 규칙).
"""
import shutil
import threading
from collections.abc import Mapping
from pathlib import Path

from kros.context import Context
from kros.msgs.nav2_msgs import LoadMap, LoadMapRequest, LoadMapResponse
from kros.msgs.nav_msgs import OccupancyGrid
from kros.msgs.std_msgs import String
from kros.msgs.std_srvs import Trigger, TriggerRequest, TriggerResponse
from kros.nodes.node import Node
from kros.parameter import ParameterNotDeclaredError, ParameterRejectedError, ParameterValue
from kros.topic import QOS_DEFAULT, QOS_LATCHED

MAP_LIST_TOPIC = "map_list"
MAP_CHANGE_TOPIC = "map_change"
MAP_STATUS_TOPIC = "map_status"
MAP_FILE = "map"


class MapCatalog(Node):
    """지도 폴더 목록을 내고, 원격에서 온 map_change로 지도를 바꾸거나 지운다.

    파라미터: maps_directory (지도 폴더, ""이면 지금 작업 폴더), list_period (목록 확인 주기 초, 기본 1.0),
    map_server ("map_server"), cartographer ("cartographer").
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "map_catalog", parameters)
        positive = lambda value: value > 0  # noqa: E731
        self.maps_directory = self.declare_parameter("maps_directory", "")
        self.list_period = self.declare_parameter("list_period", 1.0, validator=positive)
        self.map_server = self.declare_parameter("map_server", "map_server")
        self.cartographer = self.declare_parameter("cartographer", "cartographer")
        self._lock = threading.Lock()
        self._loaded: str | None = None  # map_change로 띄운 지도 이름 (빈 map_change가 지울 대상)
        self._last_list: tuple[str, ...] | None = None
        self.subscribe(MAP_CHANGE_TOPIC, String, self._on_change)
        self.subscribe("map", OccupancyGrid, self._on_map, QOS_LATCHED)  # unload(0x0 지도)를 보면 loaded를 지운다
        self.create_timer(self.list_period.value, self._refresh)
        self._refresh()

    # ---- 목록 ---------------------------------------------------------------------------------

    def directory(self) -> Path:
        value = str(self.maps_directory.value).strip()
        return (Path(value).expanduser() if value else Path.cwd()).resolve()

    def names(self) -> list[str]:
        """maps_directory 아래 map.yaml이 있는 폴더 이름들 (정렬)."""
        directory = self.directory()
        if not directory.is_dir():
            return []
        return sorted(path.parent.name for path in directory.glob(f"*/{MAP_FILE}.yaml"))

    def _refresh(self) -> None:
        names = tuple(self.names())
        if names == self._last_list:
            return
        self._last_list = names
        self.publish(MAP_LIST_TOPIC, String("\n".join(names)), QOS_LATCHED)

    # ---- 변경/삭제 ----------------------------------------------------------------------------

    def _on_change(self, message: String) -> None:
        name = message.data.strip()
        if name:
            self._load(name)
        else:
            self._delete_loaded()

    def _load(self, name: str) -> None:
        if Path(name).name != name or name in (".", ".."):
            self._status(f"invalid map name: {name}")
            return
        if name not in self.names():
            self._status(f"no map named {name}")
            return
        path = self.directory() / name / f"{MAP_FILE}.yaml"
        try:
            response = self.call_service(f"{self.map_server.value}/load_map", LoadMap, LoadMapRequest(str(path)), timeout=5.0)
        except LookupError:
            self._status(f"no service {self.map_server.value}/load_map")
            return
        if not isinstance(response, LoadMapResponse) or response.result != LoadMapResponse.RESULT_SUCCESS:
            self._status(f"load failed: {name}")
            return
        with self._lock:
            self._loaded = name
        self._set_mode("localization")
        self.get_logger().info("지도를 바꿨다: %s", name)
        self._status(f"loaded {name}")
        self._refresh()

    def _delete_loaded(self) -> None:
        with self._lock:
            name = self._loaded
            self._loaded = None
        if name is None:
            self._status("no map loaded (map_change with a name first)")
            return
        directory = self.directory() / name
        try:  # 지도를 내린 뒤 폴더를 지운다 (지도가 없으면 무시)
            self.call_service(f"{self.map_server.value}/unload_map", Trigger, TriggerRequest(), timeout=5.0)
        except LookupError:
            pass
        self._set_mode("idle")
        try:
            if directory.is_dir() and directory.resolve().parent == self.directory():
                shutil.rmtree(directory)
            else:
                self._status(f"not a map folder: {name}")
                return
        except OSError as error:
            self.get_logger().error("지도 폴더를 지우지 못했다: %s", error)
            self._status(f"delete failed: {name}")
            return
        self.get_logger().info("지도를 지웠다: %s", name)
        self._status(f"deleted {name}")
        self._refresh()

    def _on_map(self, grid: OccupancyGrid) -> None:
        if grid.info.width == 0 or grid.info.height == 0:  # map_server/unload_map
            with self._lock:
                self._loaded = None

    def _set_mode(self, mode: str) -> None:
        try:
            self.context.parameter_manager.set(str(self.cartographer.value), "mode", mode)
        except (ParameterNotDeclaredError, ParameterRejectedError):
            pass  # cartographer가 없는 구성(지도만 보는 등)이면 그냥 둔다

    def _status(self, text: str) -> None:
        self.get_logger().info("%s", text)
        self.publish(MAP_STATUS_TOPIC, String(text), QOS_LATCHED)
