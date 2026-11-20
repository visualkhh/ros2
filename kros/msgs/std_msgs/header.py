from dataclasses import dataclass


@dataclass(frozen=True)
class Header:
    """ROS std_msgs/Header 흉내: 메시지가 만들어진 시각(stamp, kros 시간 원천의 초)과 어느 좌표계의 값인지(frame_id)."""

    stamp: float
    frame_id: str = ""
