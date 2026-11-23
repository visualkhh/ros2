"""cartographer_ros 흉내 (2D): 라이다 스캔을 격자 지도에 맞춰(스캔 매칭) 로봇 위치를 바로잡고, map -> odom TF를 낸다.

mode 파라미터 (실행 중에 바꿀 수 있다: ros2 param set처럼):
  idle         아무것도 안 한다 (TF도 안 낸다).
  mapping      SLAM. 스캔으로 새 지도를 만들면서 그 지도에 맞춰 위치를 고친다. 지도는 map 토픽(latch)으로 낸다. 들어갈 때마다 새 지도로 시작한다.
  localization 저장한 지도(map_server가 낸 map 토픽)에 맞춰 위치만 고친다 (cartographer의 pure localization). 지도는 고치지 않는다.
odom -> base 까지는 EKF(또는 바퀴 오도메트리)가 내고, 이 노드는 그 위에 map -> odom 하나만 얹는다 (ROS의 REP-105와 같다).
/initialpose (geometry_msgs/PoseWithCovarianceStamped, rviz의 2D Pose Estimate): 로봇이 지도의 어디에 있는지 알려 준다. ROS에서는 nav2의 amcl이 받는다
(cartographer_ros는 start_trajectory 서비스로 받는다). 여기서는 localization 모드가 amcl 자리라 이 노드가 받는다. 받은 뒤 첫 매칭은 넓게 뒤진다.
"""
import math
import threading
from collections.abc import Mapping

from kros.context import Context
from kros.msgs.geometry_msgs import PoseWithCovarianceStamped, Quaternion, Transform, TransformStamped, Vector3
from kros.msgs.nav_msgs import OccupancyGrid
from kros.msgs.sensor_msgs import LaserScan
from kros.msgs.std_msgs import Header
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.tf import Buffer, TransformBroadcaster, TransformException, TransformListener
from kros.topic import QOS_LATCHED

from .probability_grid import ProbabilityGrid
from .scan_matcher import Pose2, match

MODES = ("idle", "mapping", "localization")
INITIAL_POSE_TOPIC_NAME = "initialpose"
RELOCALIZE_SCALE = 3.0  # /initialpose 뒤 첫 매칭은 탐색 범위를 이만큼 넓힌다 (사람이 찍은 위치는 대충이다)


def compose(a: Pose2, b: Pose2) -> Pose2:
    """a 좌표계 안의 b를 a의 부모 좌표계로."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2]))


def yaw_of(q: Quaternion) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def inverse(a: Pose2) -> Pose2:
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


class Cartographer(Node):
    """2D SLAM / 위치 추정. 파라미터 (cartographer 설정 이름을 따랐다):
      mode (위 설명), map_frame ("map"), odom_frame ("odom"), scan_topic ("scan"), resolution (지도 칸 m),
      map_publish_period_sec, tf_publish_period_sec,
      linear_search_window / angular_search_window (스캔 매칭이 예상 위치 둘레를 뒤지는 범위 m/rad),
      min_score (이보다 못 맞으면 매칭을 버리고 오도메트리를 믿는다), motion_filter_max_distance / motion_filter_max_angle
      (지도에 스캔을 넣는 간격: 이만큼 움직였을 때만. 서 있을 때 같은 스캔을 계속 쌓지 않게), num_matching_points (매칭에 쓰는 점 수).
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "cartographer", parameters)
        positive = lambda value: value > 0  # noqa: E731
        self.map_frame = self.declare_parameter("map_frame", "map")
        self.odom_frame = self.declare_parameter("odom_frame", "odom")
        self.tracking_frame = self.declare_parameter("tracking_frame", "base_footprint")  # /initialpose가 가리키는 로봇의 좌표계
        self.scan_topic = self.declare_parameter("scan_topic", "scan")
        self.resolution = self.declare_parameter("resolution", 0.05, validator=positive)
        self.map_publish_period_sec = self.declare_parameter("map_publish_period_sec", 1.0, validator=positive)
        self.tf_publish_period_sec = self.declare_parameter("tf_publish_period_sec", 0.05, validator=positive)
        self.linear_search_window = self.declare_parameter("linear_search_window", 0.1, validator=positive)
        self.angular_search_window = self.declare_parameter("angular_search_window", math.radians(6), validator=positive)
        self.min_score = self.declare_parameter("min_score", 0.55, validator=lambda value: 0 <= value <= 1)
        self.motion_filter_max_distance = self.declare_parameter("motion_filter_max_distance", 0.05, validator=positive)
        self.motion_filter_max_angle = self.declare_parameter("motion_filter_max_angle", math.radians(3), validator=positive)
        self.num_matching_points = self.declare_parameter("num_matching_points", 60, validator=positive)
        self.match_every = self.declare_parameter("match_every", 1, validator=lambda value: isinstance(value, int) and value >= 1)  # N번째 스캔마다 매칭 (실기 Pi는 2: 5Hz)
        self.map_to_odom: Pose2 = (0.0, 0.0, 0.0)
        self.grid = ProbabilityGrid(self.resolution.value)
        self.scans = self.rejected = 0  # 처리한 스캔 수, 점수가 낮아 매칭을 버린 수
        self._mode = "idle"
        self._scan_index = 0
        self._lock = threading.Lock()
        self._last_inserted: Pose2 | None = None
        self._localized = False  # map -> odom을 알게 됐다 (스캔을 처리했거나 /initialpose를 받았다): 그때부터 TF를 낸다
        self._relocalize = False  # 다음 매칭은 넓게 (/initialpose 직후)
        self._dirty = False
        self._published: OccupancyGrid | None = None  # 내가 낸 지도 (map 토픽으로 되돌아와도 저장한 지도로 착각하지 않게)
        self._external_map: OccupancyGrid | None = None  # map_server가 낸 지도
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, self)
        self._broadcaster = TransformBroadcaster(self)
        self.subscribe(self.scan_topic.value, LaserScan, self._on_scan)
        self.subscribe("map", OccupancyGrid, self._on_map, QOS_LATCHED)
        self.subscribe(INITIAL_POSE_TOPIC_NAME, PoseWithCovarianceStamped, self._on_initial_pose)
        self.create_timer(self.map_publish_period_sec.value, self._publish_map)
        self.create_timer(self.tf_publish_period_sec.value, self._publish_tf)
        self.mode = self.declare_parameter("mode", "mapping", validator=lambda value: value in MODES, on_change=self._on_mode)  # 상태를 다 만든 뒤에 (바꾸면 _on_mode가 불린다)
        self._on_mode(self.mode.value)

    # ---- 모드 ----------------------------------------------------------------------------------

    def _on_mode(self, mode: str) -> None:
        with self._lock:
            if mode == self._mode:
                return
            self._mode = mode
            self._last_inserted = None
            if mode == "mapping":
                self.grid = ProbabilityGrid(self.resolution.value)  # 새 지도로 시작한다 (위치는 지금까지 고친 것을 이어 간다)
            elif mode == "localization":
                self.grid = ProbabilityGrid.from_message(self._external_map) if self._external_map else ProbabilityGrid(self.resolution.value)
        self.get_logger().info("mode: %s", mode)

    def _on_map(self, grid: OccupancyGrid) -> None:
        if grid is self._published:
            return
        with self._lock:
            self._external_map = grid
            if self._mode == "localization":
                self.grid = ProbabilityGrid.from_message(grid)

    # ---- 스캔 -----------------------------------------------------------------------------------

    def _on_scan(self, scan: LaserScan) -> None:
        if self._mode == "idle":
            return
        self._scan_index += 1
        if self._scan_index % self.match_every.value != 0:
            return  # 스캔매칭은 무거우니 N번째만 (TF는 마지막 보정값으로 계속 낸다)
        try:  # 스캔을 잰 순간의 센서 위치 (odom 기준)
            t = self.buffer.lookup_transform(self.odom_frame.value, scan.header.frame_id, scan.header.stamp).transform
        except TransformException:
            try:  # 그 시각의 TF가 아직 없다 (스캔이 TF보다 먼저 왔다): 최신 TF로. 버리면 바퀴가 헛돌 때 스캔이 연달아 빠져 위치를 놓친다
                t = self.buffer.lookup_transform(self.odom_frame.value, scan.header.frame_id).transform
            except TransformException:
                return
        sensor_in_odom = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        points = []  # 센서 좌표의 스캔 끝점 (잰 것만)
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                a = scan.angle_min + i * scan.angle_increment
                points.append((r * math.cos(a), r * math.sin(a)))
        if len(points) < 10:
            return
        with self._lock:
            mode, grid = self._mode, self.grid
            guess = compose(self.map_to_odom, sensor_in_odom)
            pose = guess
            if grid.log_odds:
                step = max(1, len(points) // self.num_matching_points.value)
                scale = RELOCALIZE_SCALE if self._relocalize else 1.0
                self._relocalize = False
                found, value = match(grid, points[::step], guess, self.linear_search_window.value * scale, self.angular_search_window.value * scale)
                if value >= self.min_score.value:
                    pose = found
                else:
                    self.rejected += 1
            elif mode == "localization":
                return  # 지도가 아직 없다
            self.map_to_odom = compose(pose, inverse(sensor_in_odom))
            self.scans += 1
            self._localized = True
            if mode == "mapping" and self._moved(pose):
                c, s = math.cos(pose[2]), math.sin(pose[2])
                grid.insert((pose[0], pose[1]), [(pose[0] + c * x - s * y, pose[1] + s * x + c * y) for x, y in points])
                self._last_inserted, self._dirty = pose, True

    def _on_initial_pose(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id != self.map_frame.value:
            self.get_logger().warn("/initialpose는 %s 좌표계여야 한다: %s", self.map_frame.value, message.header.frame_id)
            return
        try:
            t = self.buffer.lookup_transform(self.odom_frame.value, self.tracking_frame.value).transform
        except TransformException:
            self.get_logger().warn("%s -> %s TF가 아직 없어 /initialpose를 쓸 수 없다", self.odom_frame.value, self.tracking_frame.value)
            return
        p = message.pose.pose.position
        base_in_odom = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        base_in_map = (p.x, p.y, yaw_of(message.pose.pose.orientation))
        with self._lock:
            self.map_to_odom = compose(base_in_map, inverse(base_in_odom))  # 로봇이 base_in_map에 있게 하는 map -> odom
            self._relocalize, self._localized, self._last_inserted = True, True, None
        self.get_logger().info("initialpose: (%.2f, %.2f, %.0fdeg)", base_in_map[0], base_in_map[1], math.degrees(base_in_map[2]))

    def _moved(self, pose: Pose2) -> bool:
        last = self._last_inserted
        if last is None:
            return True
        turn = abs(math.atan2(math.sin(pose[2] - last[2]), math.cos(pose[2] - last[2])))
        return math.hypot(pose[0] - last[0], pose[1] - last[1]) >= self.motion_filter_max_distance.value or turn >= self.motion_filter_max_angle.value

    # ---- 내보내기 -------------------------------------------------------------------------------

    def _publish_map(self) -> None:
        with self._lock:
            if not (self._dirty and self._mode == "mapping"):
                return
            self._dirty = False
            grid = self.grid.to_message(Header(self.now(), self.map_frame.value))
            self._published = grid
        self.publish("map", grid, QOS_LATCHED)

    def _publish_tf(self) -> None:
        if self._mode == "idle" or not self._localized:
            return
        x, y, yaw = self.map_to_odom
        transform = Transform(Vector3(x, y, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        self._broadcaster.send_transform(TransformStamped(Header(self.now(), self.map_frame.value), self.odom_frame.value, transform))
