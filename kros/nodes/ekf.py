"""robot_localization의 ekf_node 흉내: 바퀴 오도메트리와 IMU를 확장 칼만 필터로 합쳐 odom 좌표계의 로봇 위치를 추정한다 (2D만, two_d_mode)."""
import math
import threading
from collections.abc import Mapping

from kros.context import Context
from kros.msgs.geometry_msgs import Point, Pose, PoseWithCovariance, Quaternion, Transform, TransformStamped, Twist, TwistWithCovariance, Vector3
from kros.msgs.nav_msgs import Odometry
from kros.msgs.sensor_msgs import Imu
from kros.msgs.std_msgs import Header
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.tf import TransformBroadcaster

FILTERED_TOPIC_NAME = "odometry/filtered"

X, Y, YAW, V, W = range(5)  # 상태: odom에서 본 위치/방향 + 로봇 축으로 본 앞 속도/회전 속도
CONFIG_INDEX = {X: 0, Y: 1, YAW: 5, V: 6, W: 11}  # robot_localization의 15개 순서 (x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az)에서의 자리
FLOOR = 1e-9  # 공분산이 0(ROS의 "모름")이면 robot_localization처럼 아주 작은 값으로 바꾼다

Matrix = list[list[float]]
Measurement = list[tuple[int, float, float]]  # (상태 자리, 값, 분산)


def _mul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def _transpose(a: Matrix) -> Matrix:
    return [list(row) for row in zip(*a)]


def _inverse(a: Matrix) -> Matrix:
    """가우스-조던 소거 (부분 피벗). 공분산이라 늘 역행렬이 있다."""
    n = len(a)
    m = [row[:] + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        p = m[col][col]
        m[col] = [v / p for v in m[col]]
        for r in range(n):
            if r != col and m[r][col] != 0.0:
                f = m[r][col]
                m[r] = [v - f * c for v, c in zip(m[r], m[col])]
    return [row[n:] for row in m]


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw(q: Quaternion) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _diagonal(values: tuple[float, ...]) -> Matrix:
    """robot_localization 순서의 대각 15개 중 이 필터가 쓰는 5개로 5x5 대각 행렬."""
    return [[values[CONFIG_INDEX[i]] if i == j else 0.0 for j in range(5)] for i in range(5)]


class Ekf(Node):
    """robot_localization의 ekf_node 흉내. 상태 [x, y, yaw, v, w], 일정 속도 모델로 예측하고 센서 값으로 고친다.

    파라미터 (robot_localization과 같은 이름. 토픽 이름과 frequency는 만들 때만 읽는다):
      frequency: 이만큼(Hz)마다 쌓인 센서 값을 시각 순서로 처리하고 결과를 낸다. 지난번에 처리한 시각보다 오래된 값은 버린다 (late 로 센다).
      odom_frame, base_link_frame, publish_tf: 결과 odometry/filtered 의 좌표계와 odom -> base_link_frame TF를 낼지.
        (TF를 내면 바퀴 오도메트리(DiffDriveOdometry)의 publish_tf는 꺼야 한다: 같은 TF를 둘이 내면 안 된다)
      odom0/imu0: 구독할 nav_msgs/Odometry, sensor_msgs/Imu 토픽 ("" 이면 안 쓴다).
      odom0_config/imu0_config: 15개 bool (x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az) 중 쓸 값. 2D라 x, y, yaw, vx, vyaw만 본다.
      odom0/imu0_pose_rejection_threshold, _twist_rejection_threshold: 마할라노비스 거리가 이보다 크면 그 값은 이상값으로 버린다 (rejected 로 센다).
      imu0_relative: 처음 받은 IMU 방향을 0으로 친다 (IMU 방향은 월드 기준이라 odom 시작 방향과 다를 때).
      process_noise_covariance, initial_estimate_covariance: 15개 대각 (robot_localization은 15x15 전체지만 여기선 대각만).
    shortcut: 센서 좌표계(imu_link)가 base_link_frame과 같은 방향이라고 본다 (TurtleBot3는 그렇다). 다르면 TF로 돌려야 한다.
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name, parameters)
        positive = lambda value: value > 0  # noqa: E731
        flags = lambda value: len(value) == 15  # noqa: E731
        diagonal = lambda value: len(value) == 15 and all(item >= 0 for item in value)  # noqa: E731
        off = (False,) * 15
        self.frequency = self.declare_parameter("frequency", 30.0, validator=positive)
        self.odom_frame = self.declare_parameter("odom_frame", "odom")
        self.base_link_frame = self.declare_parameter("base_link_frame", "base_footprint")
        self.publish_tf = self.declare_parameter("publish_tf", True)
        self.odom0 = self.declare_parameter("odom0", "")
        self.odom0_config = self.declare_parameter("odom0_config", off, validator=flags)
        self.odom0_pose_rejection_threshold = self.declare_parameter("odom0_pose_rejection_threshold", math.inf, validator=positive)
        self.odom0_twist_rejection_threshold = self.declare_parameter("odom0_twist_rejection_threshold", math.inf, validator=positive)
        self.imu0 = self.declare_parameter("imu0", "")
        self.imu0_config = self.declare_parameter("imu0_config", off, validator=flags)
        self.imu0_relative = self.declare_parameter("imu0_relative", False)
        self.imu0_pose_rejection_threshold = self.declare_parameter("imu0_pose_rejection_threshold", math.inf, validator=positive)
        self.imu0_twist_rejection_threshold = self.declare_parameter("imu0_twist_rejection_threshold", math.inf, validator=positive)
        self.process_noise_covariance = self.declare_parameter(
            "process_noise_covariance", (0.05, 0.05, 0.06, 0.03, 0.03, 0.06, 0.025, 0.025, 0.04, 0.01, 0.01, 0.02, 0.01, 0.01, 0.015), validator=diagonal
        )
        self.initial_estimate_covariance = self.declare_parameter("initial_estimate_covariance", (1e-9,) * 15, validator=diagonal)
        self.state = [0.0] * 5
        self.covariance = _diagonal(self.initial_estimate_covariance.value)
        self.time: float | None = None  # 상태가 가리키는 시각 (첫 센서 값을 받기 전엔 None: 아무것도 내지 않는다)
        self.rejected = self.late = 0
        self._imu_yaw_offset: float | None = None
        self._queue: list[tuple[float, int, Measurement, Measurement, float, float]] = []  # (시각, 순번, pose 값, twist 값, pose 문턱, twist 문턱)
        self._lock = threading.Lock()
        self._broadcaster = TransformBroadcaster(self)
        if self.odom0.value:
            self.subscribe(self.odom0.value, Odometry, self._on_odom)
        if self.imu0.value:
            self.subscribe(self.imu0.value, Imu, self._on_imu)
        self.create_timer(1.0 / self.frequency.value, self._on_timer)

    # ---- 센서 값 받기: 쓸 값만 골라 큐에 쌓는다 ------------------------------------------------

    def _pick(self, config: tuple[bool, ...], values: Measurement) -> Measurement:
        return [(i, value, max(variance, FLOOR)) for i, value, variance in values if config[CONFIG_INDEX[i]]]

    def _enqueue(self, stamp: float, pose: Measurement, twist: Measurement, pose_threshold: float, twist_threshold: float) -> None:
        with self._lock:
            self._queue.append((stamp, len(self._queue), pose, twist, pose_threshold, twist_threshold))

    def _on_odom(self, message: Odometry) -> None:
        config, p, t = self.odom0_config.value, message.pose, message.twist
        pose = self._pick(config, [(X, p.pose.position.x, p.covariance[0]), (Y, p.pose.position.y, p.covariance[7]), (YAW, _yaw(p.pose.orientation), p.covariance[35])])
        twist = self._pick(config, [(V, t.twist.linear.x, t.covariance[0]), (W, t.twist.angular.z, t.covariance[35])])
        self._enqueue(message.header.stamp, pose, twist, self.odom0_pose_rejection_threshold.value, self.odom0_twist_rejection_threshold.value)

    def _on_imu(self, message: Imu) -> None:
        config = self.imu0_config.value
        pose: Measurement = []
        if message.orientation_covariance[0] != -1.0:  # ROS: -1이면 방향 값이 없다
            yaw = _yaw(message.orientation)
            if self.imu0_relative.value:
                if self._imu_yaw_offset is None:
                    self._imu_yaw_offset = yaw
                yaw = _wrap(yaw - self._imu_yaw_offset)
            pose = self._pick(config, [(YAW, yaw, message.orientation_covariance[8])])
        twist = self._pick(config, [(W, message.angular_velocity.z, message.angular_velocity_covariance[8])])
        self._enqueue(message.header.stamp, pose, twist, self.imu0_pose_rejection_threshold.value, self.imu0_twist_rejection_threshold.value)

    # ---- 필터 -------------------------------------------------------------------------------

    def _on_timer(self) -> None:
        with self._lock:
            queue, self._queue = sorted(self._queue), []
        for stamp, _, pose, twist, pose_threshold, twist_threshold in queue:
            if self.time is None:
                self.time = stamp  # 첫 값의 시각에서 원점으로 시작한다
            if stamp < self.time:
                self.late += 1
                continue
            self.predict(stamp - self.time)
            self.time = stamp
            self.correct(pose, pose_threshold)
            self.correct(twist, twist_threshold)
        if self.time is not None:
            self._publish(self.time)

    def predict(self, dt: float) -> None:
        """일정 속도 모델로 dt초 뒤를 예측한다. 불확실성은 F P F^T + Q dt 만큼 커진다."""
        if dt <= 0:
            return
        x, y, yaw, v, w = self.state
        c, s = math.cos(yaw), math.sin(yaw)
        self.state = [x + v * c * dt, y + v * s * dt, _wrap(yaw + w * dt), v, w]
        f = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
        f[X][YAW], f[X][V] = -v * s * dt, c * dt
        f[Y][YAW], f[Y][V] = v * c * dt, s * dt
        f[YAW][W] = dt
        q = _diagonal(self.process_noise_covariance.value)
        self.covariance = [[a + q[i][j] * dt for j, a in enumerate(row)] for i, row in enumerate(_mul(_mul(f, self.covariance), _transpose(f)))]

    def correct(self, measurement: Measurement, threshold: float) -> bool:
        """측정한 값들로 상태를 고친다. 마할라노비스 거리가 threshold를 넘으면 버리고 False."""
        if not measurement:
            return False
        idx = [i for i, _, _ in measurement]
        innovation = [[_wrap(value - self.state[i]) if i == YAW else value - self.state[i]] for i, value, _ in measurement]
        p = self.covariance
        s = [[p[a][b] + (variance if r == c else 0.0) for c, b in enumerate(idx)] for r, (a, _, variance) in enumerate(measurement)]
        s_inv = _inverse(s)
        if _mul(_mul(_transpose(innovation), s_inv), innovation)[0][0] > threshold**2:
            self.rejected += 1
            return False
        gain = _mul([[p[r][c] for c in idx] for r in range(5)], s_inv)  # K = P H^T S^-1
        correction = _mul(gain, innovation)
        self.state = [value + correction[i][0] for i, value in enumerate(self.state)]
        self.state[YAW] = _wrap(self.state[YAW])
        reduce = _mul(gain, [p[i] for i in idx])  # K H P
        updated = [[a - b for a, b in zip(row, sub)] for row, sub in zip(p, reduce)]
        self.covariance = [[(updated[i][j] + updated[j][i]) / 2 for j in range(5)] for i in range(5)]  # 반올림 오차로 대칭이 깨지지 않게
        return True

    def _publish(self, stamp: float) -> None:
        x, y, yaw, v, w = self.state
        p = self.covariance
        orientation = Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        pose_cov = [0.0] * 36
        for (a, ra), (b, rb) in ((m, n) for m in ((X, 0), (Y, 1), (YAW, 5)) for n in ((X, 0), (Y, 1), (YAW, 5))):
            pose_cov[ra * 6 + rb] = p[a][b]
        twist_cov = [0.0] * 36
        twist_cov[0], twist_cov[5], twist_cov[30], twist_cov[35] = p[V][V], p[V][W], p[W][V], p[W][W]
        odom_frame, base_frame = self.odom_frame.value, self.base_link_frame.value
        pose = PoseWithCovariance(Pose(Point(x, y, 0.0), orientation), tuple(pose_cov))
        twist = TwistWithCovariance(Twist(Vector3(x=v), Vector3(z=w)), tuple(twist_cov))
        self.publish(FILTERED_TOPIC_NAME, Odometry(Header(stamp, odom_frame), base_frame, pose, twist))
        if self.publish_tf.value:
            self._broadcaster.send_transform(TransformStamped(Header(stamp, odom_frame), base_frame, Transform(Vector3(x, y, 0.0), orientation)))
