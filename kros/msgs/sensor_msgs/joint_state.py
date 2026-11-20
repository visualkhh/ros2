from dataclasses import dataclass

from kros.msgs.std_msgs import Header


@dataclass(frozen=True)
class JointState:
    """ROS sensor_msgs/JointState 흉내: 관절 이름별 위치(rad 또는 m)와 속도(없으면 빈 튜플). name과 position은 같은 순서다."""

    header: Header
    name: tuple[str, ...]
    position: tuple[float, ...]
    velocity: tuple[float, ...] = ()
