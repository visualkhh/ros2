from dataclasses import dataclass, field

from kros.msgs.geometry_msgs import Pose


@dataclass(frozen=True)
class MapMetaData:
    """ROS nav_msgs/MapMetaData 흉내: 격자 지도의 크기와 위치.
    resolution은 칸 하나의 크기(m), origin은 칸 (0, 0)의 모서리가 map 좌표계에서 놓인 곳 (x축이 width 방향, y축이 height 방향)."""

    map_load_time: float = 0.0
    resolution: float = 0.05
    width: int = 0
    height: int = 0
    origin: Pose = field(default_factory=Pose)
