from dataclasses import dataclass


@dataclass(frozen=True)
class Bool:
    """ROS std_msgs/Bool 흉내 (twist_mux의 lock 같은 켜기/끄기 신호)."""

    data: bool = False
