from dataclasses import dataclass
from enum import Enum

from .parameter_manager import ParameterValue

PARAMETER_EVENTS_TOPIC_NAME = "parameter_events"  # 모든 노드의 파라미터 변화가 이 토픽 하나로 나온다 (ROS2의 /parameter_events)


class ParameterEventKind(Enum):
    DECLARED = "declared"  # 노드가 파라미터를 선언했다 (시작 값이 정해졌다)
    CHANGED = "changed"  # 값이 바뀌었다


@dataclass(frozen=True)
class ParameterEvent:
    """파라미터 변화 알림. ROS2의 ParameterEvent와 달리 이전 값(old)도 담는다. 선언일 때 old는 None."""

    node: str
    name: str
    kind: ParameterEventKind
    old: ParameterValue | None
    new: ParameterValue
