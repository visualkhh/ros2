from collections.abc import Mapping
from pathlib import Path

from kros.context import Context
from kros.msgs.nav2_msgs import LoadMap, LoadMapRequest, LoadMapResponse
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.topic import QOS_LATCHED

from .map_io import MapIoError, load_map

def resolve(url: str, directory: str) -> Path:
    """상대 경로면 지도 폴더 기준, 절대 경로면 그대로. 지도 폴더도 상대 경로면 ROS처럼 지금 작업 폴더(cwd) 기준이다."""
    path = Path(url).expanduser()
    return path if path.is_absolute() else Path(directory).expanduser() / path


class MapServer(Node):
    """nav2 map_server 흉내: 지도 파일(yaml + pgm)을 읽어 map 토픽(latch)으로 낸다.

    파라미터: yaml_filename (시작할 때 띄울 지도, ""이면 없음), topic_name ("map"), frame_id ("map"), maps_directory (상대 경로의 기준, 기본 "": 작업 폴더. 예제는 examples/turtlebot3/artifacts/maps).
    서비스 map_server/load_map (nav2_msgs/LoadMap): 실행 중에 다른 지도를 띄운다. 실패하면 지금 지도는 그대로다.
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "map_server", parameters)
        self.yaml_filename = self.declare_parameter("yaml_filename", "")
        self.topic_name = self.declare_parameter("topic_name", "map")
        self.frame_id = self.declare_parameter("frame_id", "map")
        self.maps_directory = self.declare_parameter("maps_directory", "")
        self.create_service(f"{self.name}/load_map", LoadMap, self._on_load_map)
        if self.yaml_filename.value:
            response = self.load(self.yaml_filename.value)
            if response.result != LoadMapResponse.RESULT_SUCCESS:
                self.get_logger().error("시작 지도를 띄우지 못했다: %s", self.yaml_filename.value)

    def load(self, url: str) -> LoadMapResponse:
        path = resolve(url, self.maps_directory.value)
        try:
            grid = load_map(path, self.frame_id.value, self.now())
        except FileNotFoundError:
            self.get_logger().error("지도 파일이 없다: %s", path)
            return LoadMapResponse(LoadMapResponse.RESULT_MAP_DOES_NOT_EXIST)
        except (MapIoError, OSError) as error:
            self.get_logger().error("지도를 읽지 못했다: %s", error)
            return LoadMapResponse(LoadMapResponse.RESULT_INVALID_MAP_DATA)
        self.publish(self.topic_name.value, grid, QOS_LATCHED)
        self.get_logger().info("지도를 띄웠다: %s (%dx%d, %.3fm)", path, grid.info.width, grid.info.height, grid.info.resolution)
        return LoadMapResponse(LoadMapResponse.RESULT_SUCCESS, grid)

    def _on_load_map(self, request: LoadMapRequest) -> LoadMapResponse:
        return self.load(request.map_url)
