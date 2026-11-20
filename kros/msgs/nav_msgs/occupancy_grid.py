from dataclasses import dataclass, field

from kros.msgs.std_msgs import Header

from .map_meta_data import MapMetaData

UNKNOWN = -1  # 모르는 칸
FREE = 0
OCCUPIED = 100


@dataclass(frozen=True)
class OccupancyGrid:
    """ROS nav_msgs/OccupancyGrid 흉내: 2D 격자 지도. data는 칸마다 점유 확률(0~100, 모르면 -1)을 행 우선으로 편 것 (인덱스 = y * width + x)."""

    header: Header
    info: MapMetaData = field(default_factory=MapMetaData)
    data: tuple[int, ...] = ()
