from dataclasses import dataclass

from kros.service.service_type import ServiceType


@dataclass(frozen=True)
class SaveMapRequest:
    map_topic: str = "map"
    map_url: str = ""  # 저장할 경로 (확장자 없이). 상대 경로면 MapSaver의 maps_directory 아래, 비면 "map"
    image_format: str = "pgm"
    map_mode: str = "trinary"
    free_thresh: float = 0.0  # 0이면 MapSaver의 기본값
    occupied_thresh: float = 0.0


@dataclass(frozen=True)
class SaveMapResponse:
    result: bool


SaveMap = ServiceType("nav2_msgs/srv/SaveMap", SaveMapRequest, SaveMapResponse)
