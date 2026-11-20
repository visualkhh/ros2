from dataclasses import dataclass


@dataclass(frozen=True)
class String:
    """ROS std_msgs/String 흉내."""

    data: str
