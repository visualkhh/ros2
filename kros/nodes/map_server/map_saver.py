import threading
from collections.abc import Mapping

from kros.context import Context
from kros.msgs.nav2_msgs import SaveMap, SaveMapRequest, SaveMapResponse
from kros.msgs.nav_msgs import OccupancyGrid
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.topic import QOS_LATCHED

from .map_io import FREE_THRESH, OCCUPIED_THRESH, MapIoError, save_map
from .map_server import resolve


class MapSaver(Node):
    """nav2 map_saver_server 흉내: 서비스를 받으면 map 토픽의 지도 하나를 받아 yaml + pgm으로 저장한다.

    파라미터: save_map_timeout (지도를 기다릴 초), free_thresh_default, occupied_thresh_default (요청에 0을 주면 쓰는 값), maps_directory (상대 경로의 기준, 기본 "": 작업 폴더).
    서비스 map_saver/save_map (nav2_msgs/SaveMap): map_url이 비면 "map", 상대 경로면 maps_directory 아래에 저장한다.
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "map_saver", parameters)
        positive = lambda value: value > 0  # noqa: E731
        ratio = lambda value: 0 < value < 1  # noqa: E731
        self.save_map_timeout = self.declare_parameter("save_map_timeout", 2.0, validator=positive)
        self.free_thresh_default = self.declare_parameter("free_thresh_default", FREE_THRESH, validator=ratio)
        self.occupied_thresh_default = self.declare_parameter("occupied_thresh_default", OCCUPIED_THRESH, validator=ratio)
        self.maps_directory = self.declare_parameter("maps_directory", "")
        self.create_service(f"{self.name}/save_map", SaveMap, self._on_save_map)

    def _on_save_map(self, request: SaveMapRequest) -> SaveMapResponse:
        if request.image_format != "pgm" or request.map_mode != "trinary":
            self.get_logger().error("pgm/trinary만 저장한다: %s/%s", request.image_format, request.map_mode)  # shortcut: png와 scale/raw 모드는 없다, 필요해지면 추가
            return SaveMapResponse(False)
        received: list[OccupancyGrid] = []
        arrived = threading.Event()

        def on_map(grid: OccupancyGrid) -> None:
            received.append(grid)
            arrived.set()

        subscription = self.subscribe(request.map_topic or "map", OccupancyGrid, on_map, QOS_LATCHED)  # latch라 이미 나온 지도도 바로 온다
        try:
            if not arrived.wait(self.save_map_timeout.value):
                self.get_logger().error("%s 토픽에서 지도를 받지 못했다 (%.1f초)", request.map_topic, self.save_map_timeout.value)
                return SaveMapResponse(False)
        finally:
            self.unsubscribe(subscription)
        path = resolve(request.map_url or "map", self.maps_directory.value)
        free = request.free_thresh or self.free_thresh_default.value
        occupied = request.occupied_thresh or self.occupied_thresh_default.value
        try:
            yaml = save_map(received[-1], path.with_suffix(""), free, occupied)
        except (MapIoError, OSError) as error:
            self.get_logger().error("지도를 저장하지 못했다: %s", error)
            return SaveMapResponse(False)
        self.get_logger().info("지도를 저장했다: %s", yaml)
        return SaveMapResponse(True)
