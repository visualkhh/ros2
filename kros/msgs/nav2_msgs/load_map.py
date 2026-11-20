from dataclasses import dataclass, field

from kros.msgs.nav_msgs import OccupancyGrid
from kros.msgs.std_msgs import Header
from kros.service.service_type import ServiceType


@dataclass(frozen=True)
class LoadMapRequest:
    map_url: str  # 불러올 지도의 yaml 경로


@dataclass(frozen=True)
class LoadMapResponse:
    RESULT_SUCCESS = 0
    RESULT_MAP_DOES_NOT_EXIST = 1
    RESULT_INVALID_MAP_DATA = 2
    RESULT_INVALID_MAP_METADATA = 3
    RESULT_UNDEFINED_FAILURE = 255

    result: int
    map: OccupancyGrid = field(default_factory=lambda: OccupancyGrid(Header(0.0)))


LoadMap = ServiceType("nav2_msgs/srv/LoadMap", LoadMapRequest, LoadMapResponse)
