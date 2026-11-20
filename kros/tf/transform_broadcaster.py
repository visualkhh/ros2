from collections.abc import Sequence

from kros.msgs.geometry_msgs import TransformStamped
from kros.msgs.tf2_msgs import TFMessage
from kros.node import Node

from .topics import QOS_TF_STATIC, TF_STATIC_TOPIC_NAME, TF_TOPIC_NAME


class TransformBroadcaster:
    """움직이는 변환을 tf 토픽으로 내보낸다 (ROS의 tf2_ros.TransformBroadcaster). TransformStamped의 header.frame_id가 부모, child_frame_id가 자식이다."""

    def __init__(self, node: Node) -> None:
        self._node = node

    def send_transform(self, transform: TransformStamped | Sequence[TransformStamped]) -> None:
        transforms = (transform,) if isinstance(transform, TransformStamped) else tuple(transform)
        if transforms:
            self._node.publish(TF_TOPIC_NAME, TFMessage(transforms))

    sendTransform = send_transform  # ROS(rclpy)와 같은 이름


class StaticTransformBroadcaster:
    """안 변하는 변환(센서가 달린 위치 등)을 tf_static 토픽으로 한 번 내보낸다. latch(TRANSIENT_LOCAL)로 내므로 나중에 뜬 리스너도 받는다."""

    def __init__(self, node: Node) -> None:
        self._node = node

    def send_transform(self, transform: TransformStamped | Sequence[TransformStamped]) -> None:
        transforms = (transform,) if isinstance(transform, TransformStamped) else tuple(transform)
        if transforms:
            self._node.publish(TF_STATIC_TOPIC_NAME, TFMessage(transforms), QOS_TF_STATIC)

    sendTransform = send_transform  # ROS(rclpy)와 같은 이름
