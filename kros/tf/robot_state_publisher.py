import logging

from kros.context import Context
from kros.msgs.geometry_msgs import Transform, TransformStamped, Vector3
from kros.msgs.sensor_msgs import JointState
from kros.msgs.std_msgs import Header, String
from kros.node import Node
from kros.topic import QOS_LATCHED
from kros.urdf import UrdfError, UrdfJoint, UrdfRobot, parse_urdf

from .topics import JOINT_STATES_TOPIC_NAME, ROBOT_DESCRIPTION_TOPIC_NAME
from .transform_broadcaster import StaticTransformBroadcaster, TransformBroadcaster
from .transform_math import compose, from_axis_angle, from_rpy

logger = logging.getLogger(__name__)


def _is_valid_urdf(text: str) -> bool:
    if not text:
        return True  # 아직 로봇 설명이 없다
    try:
        parse_urdf(text)
    except UrdfError as error:
        logger.warning("robot_description을 받아들이지 않는다: %s", error)
        return False
    return True


def _origin(joint: UrdfJoint) -> Transform:
    return Transform(Vector3(*joint.xyz), from_rpy(*joint.rpy))


class RobotStatePublisher(Node):
    """URDF로 로봇 내부의 TF를 만들어 내보내는 노드 (ROS의 robot_state_publisher). 사용자가 node_create에서 직접 만들어 돌려주면 일한다.

    파라미터 robot_description(URDF 본문 문자열, 기본 빈 값, 실행 중 바꿀 수 있고 바꾸면 다시 읽는다)을 읽어서:
    - robot_description 토픽(latch)으로 URDF를 한 번 내보낸다 (늦게 뜬 노드도 받는다).
    - fixed 관절은 tf_static으로 한 번 내보낸다 (센서가 달린 위치 등).
    - 움직이는 관절(continuous/revolute/prismatic)은 joint_states를 받을 때마다 tf로 내보낸다. 그 관절의 값이 아직 안 왔으면 내보내지 않는다.
    프레임 이름은 URDF의 링크 이름이다. 움직이는 관절의 변환은 (기준 위치) * (축을 따라 움직인 만큼)이다.
    floating/planar 관절은 지원하지 않는다 (경고만 하고 건너뛴다).
    """

    def __init__(self, context: Context, name: str | None = None, parameters: dict[str, bool | int | float | str] | None = None) -> None:
        super().__init__(context, name, parameters)
        self._tf = TransformBroadcaster(self)
        self._static = StaticTransformBroadcaster(self)
        self._robot: UrdfRobot | None = None
        self._movable: dict[str, UrdfJoint] = {}  # 관절 이름 -> 관절 (움직이는 것만)
        self._warned_unknown: set[str] = set()
        self.robot_description = self.declare_parameter("robot_description", "", validator=_is_valid_urdf, on_change=self._rebuild)
        self.subscribe(JOINT_STATES_TOPIC_NAME, JointState, self._on_joint_states)
        self._rebuild(self.robot_description.value)  # 시작 값(덮어쓴 값 포함)으로 처음 만든다 (선언할 때는 on_change가 안 불린다)

    @property
    def robot(self) -> UrdfRobot | None:
        return self._robot

    def _rebuild(self, text: str) -> None:
        self._warned_unknown.clear()
        if not text:
            self._robot, self._movable = None, {}
            return
        robot = parse_urdf(text)  # 검증을 통과한 값이라 실패하지 않는다
        self._robot = robot
        self._movable = {joint.name: joint for joint in robot.joints if joint.is_movable}
        self.publish(ROBOT_DESCRIPTION_TOPIC_NAME, String(text), QOS_LATCHED)
        stamp = self.now()
        fixed = [
            TransformStamped(Header(stamp, joint.parent), joint.child, _origin(joint)) for joint in robot.joints if joint.type == "fixed"
        ]
        self._static.send_transform(fixed)
        for joint in robot.joints:
            if joint.type in ("floating", "planar"):
                self.get_logger().warn("관절 %s의 type %s는 지원하지 않아 건너뛴다", joint.name, joint.type)
        self.get_logger().info("로봇 %s: 고정 관절 %d개(tf_static), 움직이는 관절 %d개", robot.name, len(fixed), len(self._movable))

    def _on_joint_states(self, state: JointState) -> None:
        if len(state.name) != len(state.position):
            self.get_logger().error("joint_states의 name(%d개)과 position(%d개) 길이가 다르다", len(state.name), len(state.position))
            return
        transforms: list[TransformStamped] = []
        for joint_name, position in zip(state.name, state.position):
            joint = self._movable.get(joint_name)
            if joint is None:
                if joint_name not in self._warned_unknown:  # 같은 이름은 한 번만 알린다
                    self._warned_unknown.add(joint_name)
                    self.get_logger().debug("URDF에 움직이는 관절로 없는 이름을 무시한다: %s", joint_name)
                continue
            motion = (
                Transform(Vector3(*(a * position for a in joint.axis)))
                if joint.type == "prismatic"
                else Transform(rotation=from_axis_angle(joint.axis, position))
            )
            transforms.append(TransformStamped(Header(state.header.stamp, joint.parent), joint.child, compose(_origin(joint), motion)))
        self._tf.send_transform(transforms)
