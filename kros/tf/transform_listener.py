from __future__ import annotations

from typing import TYPE_CHECKING

from kros.msgs.tf2_msgs import TFMessage

from .errors import TransformException
from .topics import QOS_TF, QOS_TF_STATIC, TF_STATIC_TOPIC_NAME, TF_TOPIC_NAME
from .buffer import Buffer

if TYPE_CHECKING:  # Node가 있는 nodes 패키지가 이 패키지를 import하므로 실행 때는 import하지 않는다 (순환 방지)
    from kros.nodes import Node


class TransformListener:
    """tf와 tf_static 토픽을 구독해 버퍼를 채운다 (ROS의 tf2_ros.TransformListener). 버퍼는 사용자가 만들어 넘기고, 조회는 그 버퍼에 한다 (ROS와 같다):
        buffer = Buffer(); TransformListener(buffer, node); buffer.lookup_transform(...)

    tf_static은 latch로 구독하므로 이 리스너가 나중에 떠도 이미 나간 정적 변환을 받는다.
    """

    def __init__(self, buffer: Buffer, node: Node) -> None:
        self.buffer = buffer
        self._node = node
        node.subscribe(TF_TOPIC_NAME, TFMessage, lambda message: self._on_message(message, False), QOS_TF)
        node.subscribe(TF_STATIC_TOPIC_NAME, TFMessage, lambda message: self._on_message(message, True), QOS_TF_STATIC)

    def _on_message(self, message: TFMessage, is_static: bool) -> None:
        for stamped in message.transforms:
            try:
                self.buffer.set_transform(stamped.header.frame_id, stamped.child_frame_id, stamped.header.stamp, stamped.transform, is_static)
            except (TransformException, ValueError) as error:  # 잘못된 변환 하나가 나머지를 막으면 안 된다
                self._node.get_logger().warn("TF를 버퍼에 넣지 못했다 (%s -> %s): %s", stamped.header.frame_id, stamped.child_frame_id, error)
