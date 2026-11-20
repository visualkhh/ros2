import math

from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose, PoseWithCovariance, Quaternion, Transform, TransformStamped, Twist, TwistWithCovariance, Vector3
from kros.msgs.nav_msgs import Odometry
from kros.msgs.sensor_msgs import JointState
from kros.msgs.std_msgs import Header
from kros.nodes.node import Node
from kros.tf import JOINT_STATES_TOPIC_NAME, TransformBroadcaster

ODOM_TOPIC_NAME = "odom"


def _matrix(diagonal: tuple[float, ...]) -> tuple[float, ...]:
    """대각 6개 -> 행 우선 6x6 (36개)."""
    return tuple(diagonal[i // 6] if i % 7 == 0 else 0.0 for i in range(36))


class DiffDriveOdometry(Node):
    """차동 구동 로봇의 오도메트리: joint_states의 바퀴 회전각을 적분해 odom 좌표계에서 본 로봇 위치를 추정한다 (바퀴 미끄러짐이 있으면 오차가 쌓인다).
    odom 토픽(Odometry)과 TF(odom -> base_footprint)로 내보낸다. 쓸지는 사용자가 node_create에서 정한다.

    파라미터(실행 중에도 바꿀 수 있다): wheel_radius, wheel_separation (m), left_joint, right_joint (joint_states의 관절 이름),
    odom_frame, base_frame, publish_tf, jump_threshold (한 번에 이만큼(rad) 넘게 바퀴 각이 뛰면 재부팅 등으로 보고 그 변화는 버린다),
    pose_covariance_diagonal, twist_covariance_diagonal (ROS diff_drive_controller처럼 odom에 실을 공분산의 대각 6개: x, y, z, roll, pitch, yaw.
    평면 로봇이라 z/roll/pitch는 크게(1e6) 둔다. 바퀴는 미끄러지면 틀리니 회전(yaw)을 덜 믿는다).
    첫 joint_states를 받으면 원점(0, 0, 0)의 odom을 낸다.
    """

    def __init__(self, context: Context, name: str | None = None, parameters: dict[str, bool | int | float | str] | None = None) -> None:
        super().__init__(context, name, parameters)
        positive = lambda value: value > 0  # noqa: E731
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.033, validator=positive)
        self.wheel_separation = self.declare_parameter("wheel_separation", 0.288, validator=positive)
        self.left_joint = self.declare_parameter("left_joint", "wheel_left_joint")
        self.right_joint = self.declare_parameter("right_joint", "wheel_right_joint")
        self.odom_frame = self.declare_parameter("odom_frame", "odom")
        self.base_frame = self.declare_parameter("base_frame", "base_footprint")
        self.publish_tf = self.declare_parameter("publish_tf", True)
        self.jump_threshold = self.declare_parameter("jump_threshold", 1.0, validator=positive)
        diagonal = lambda value: len(value) == 6 and all(item >= 0 for item in value)  # noqa: E731
        self.pose_covariance_diagonal = self.declare_parameter("pose_covariance_diagonal", (0.001, 0.001, 1e6, 1e6, 1e6, 0.01), validator=diagonal)
        self.twist_covariance_diagonal = self.declare_parameter("twist_covariance_diagonal", (0.001, 0.001, 1e6, 1e6, 1e6, 0.01), validator=diagonal)
        self.x = self.y = self.yaw = 0.0
        self.jumps = 0  # 버린 바퀴 각 점프 횟수
        self._previous: tuple[float, float, float] | None = None  # 지난 (시각, 왼쪽 각, 오른쪽 각)
        self._broadcaster = TransformBroadcaster(self)
        self.subscribe(JOINT_STATES_TOPIC_NAME, JointState, self._on_joint_states)

    def _on_joint_states(self, message: JointState) -> None:
        names = dict(zip(message.name, message.position))
        left, right = names.get(self.left_joint.value), names.get(self.right_joint.value)
        if left is None or right is None:
            return  # 이 로봇의 바퀴가 아닌 joint_states
        stamp, previous = message.header.stamp, self._previous
        self._previous = (stamp, left, right)
        v = w = 0.0
        if previous is not None:
            dl, dr, dt = left - previous[1], right - previous[2], stamp - previous[0]
            if abs(dl) > self.jump_threshold.value or abs(dr) > self.jump_threshold.value:
                self.jumps += 1
                self.get_logger().warn("바퀴 각이 한 번에 (%+.2f, %+.2f) rad 뛰었다: 그 변화는 버린다 (보드 재부팅?)", dl, dr)
            elif dt > 0:
                r, separation = self.wheel_radius.value, self.wheel_separation.value
                distance, turn = r * (dl + dr) / 2, r * (dr - dl) / separation
                self.x += distance * math.cos(self.yaw + turn / 2)  # 중간 각도로 적분하면 호를 따라 움직인 것과 같다
                self.y += distance * math.sin(self.yaw + turn / 2)
                self.yaw += turn
                v, w = distance / dt, turn / dt
        self._publish(stamp, v, w)

    def _publish(self, stamp: float, v: float, w: float) -> None:
        orientation = Quaternion(0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))
        odom_frame, base_frame = self.odom_frame.value, self.base_frame.value
        pose = PoseWithCovariance(Pose(Point(self.x, self.y, 0.0), orientation), _matrix(self.pose_covariance_diagonal.value))
        twist = TwistWithCovariance(Twist(Vector3(x=v), Vector3(z=w)), _matrix(self.twist_covariance_diagonal.value))
        self.publish(ODOM_TOPIC_NAME, Odometry(Header(stamp, odom_frame), base_frame, pose, twist))
        if self.publish_tf.value:
            transform = Transform(Vector3(self.x, self.y, 0.0), orientation)
            self._broadcaster.send_transform(TransformStamped(Header(stamp, odom_frame), base_frame, transform))
