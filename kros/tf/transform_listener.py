from kros.msgs.tf2_msgs import TFMessage
from kros.node import Node

from .errors import TransformException
from .topics import QOS_TF, QOS_TF_STATIC, TF_STATIC_TOPIC_NAME, TF_TOPIC_NAME
from .transform_buffer import TransformBuffer


class TransformListener:
    """tf와 tf_static 토픽을 구독해 버퍼를 채운다 (ROS의 tf2_ros.TransformListener). 조회는 listener.buffer.lookup_transform(...).

    tf_static은 latch로 구독하므로 이 리스너가 나중에 떠도 이미 나간 정적 변환을 받는다.
    """

    def __init__(self, node: Node, buffer: TransformBuffer | None = None) -> None:
        self.buffer = buffer if buffer is not None else TransformBuffer()
        self._node = node
        node.subscribe(TF_TOPIC_NAME, TFMessage, lambda message: self._on_message(message, False), QOS_TF)
        node.subscribe(TF_STATIC_TOPIC_NAME, TFMessage, lambda message: self._on_message(message, True), QOS_TF_STATIC)

    def _on_message(self, message: TFMessage, is_static: bool) -> None:
        for stamped in message.transforms:
            try:
                self.buffer.set_transform(stamped.header.frame_id, stamped.child_frame_id, stamped.header.stamp, stamped.transform, is_static)
            except (TransformException, ValueError) as error:  # 잘못된 변환 하나가 나머지를 막으면 안 된다
                self._node.get_logger().warn("TF를 버퍼에 넣지 못했다 (%s -> %s): %s", stamped.header.frame_id, stamped.child_frame_id, error)
