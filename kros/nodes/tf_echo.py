from kros.context import Context
from kros.nodes import Node
from kros.tf import Buffer, TransformException, TransformListener
from kros.tf.transform_math import to_rpy


class TfEcho(Node):
    """target_frame에서 본 source_frame의 변환을 rate(Hz)마다 로그로 찍는 노드 (ROS의 tf2_echo).

    파라미터 target_frame, source_frame은 실행 중에도 바꿀 수 있다. rate는 만들 때 한 번 읽는다.
    TF가 없는 동안은 같은 이유를 한 번만 알리고, 다시 되면 알린 뒤 계속 찍는다.
    """

    def __init__(self, context: Context, name: str | None = None, parameters: dict[str, bool | int | float | str] | None = None) -> None:
        super().__init__(context, name, parameters)
        self.target_frame = self.declare_parameter("target_frame", "")
        self.source_frame = self.declare_parameter("source_frame", "")
        rate = self.declare_parameter("rate", 1.0, validator=lambda hz: hz > 0)
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, self)
        self._last_error = ""
        self.create_timer(1.0 / rate.value, self._tick)

    def _tick(self) -> None:
        target, source = self.target_frame.value, self.source_frame.value
        if not target or not source:
            return
        try:
            stamped = self.buffer.lookup_transform(target, source)
        except TransformException as error:
            if str(error) != self._last_error:  # 같은 실패를 계속 찍지 않는다
                self._last_error = str(error)
                self.get_logger().warn("%s", error)
            return
        if self._last_error:
            self._last_error = ""
            self.get_logger().info("%s -> %s 조회가 다시 된다", target, source)
        t = stamped.transform
        roll, pitch, yaw = to_rpy(t.rotation)
        self.get_logger().info(
            "At time %.3f\n- Translation: [%.3f, %.3f, %.3f]\n- Rotation: in Quaternion [%.3f, %.3f, %.3f, %.3f]\n- Rotation: in RPY (radian) [%.3f, %.3f, %.3f]",
            stamped.header.stamp, t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w, roll, pitch, yaw,
        )
