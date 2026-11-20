from dataclasses import dataclass

from kros.msgs.geometry_msgs import TransformStamped


@dataclass(frozen=True)
class TFMessage:
    """tf / tf_static 토픽에 나가는 메시지: 변환 여러 개 (ROS tf2_msgs/TFMessage 흉내)."""

    transforms: tuple[TransformStamped, ...]
