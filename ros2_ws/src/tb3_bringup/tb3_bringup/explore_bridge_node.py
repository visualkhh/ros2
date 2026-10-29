"""explore_bridge — 자동 맵핑 연결: 제어 상태 ↔ explore_lite (m-explore-ros2, ros2_ws/deps.repos).

explore_lite는 켜지자마자 탐색을 시작하고, 시작·멈춤은 /explore/resume, 진행은 /explore/status로만 알린다.
여기서 우리 제어 상태에 맞춘다:
  /state_control = SLAM_AUTO_MAPPING  → /explore/resume true   (그 밖이면 false — 켜지자마자 시작한 것도 바로 멈춤)
  /explore/status exploration_complete → /cmd_manual_control MANUAL (끝)
    단 시작 후 grace 초 안의 complete는 무시하고 다시 resume — SLAM 막 켠 빈 지도엔 frontier가 없어 바로 끝나버림
  pub /explore_status (String)  사람이 읽는 진행 상황 (tool 상태바)
  /map → /map_explore (3값 지도): cartographer 지도는 빈 칸이 0이 아니라 1~49 확률이 많고, 아는 곳 둘레에 늘 40대 띠가 있음.
    explore_lite는 '정확히 0인 칸 옆의 모르는 칸'만 frontier로 봐서 띠에 막혀 frontier 0개 → 바로 완료돼 버림.
    그래서 <50 → 0 (빈 칸) · ≥50 → 100 · -1 그대로로 바꿔 explore_lite에만 준다 (Nav2는 원래 /map).

explore-nav.launch.py가 explore_lite와 같이 띄운다.
"""
import time
from array import array

import numpy as np
import rclpy
from explore_lite_msgs.msg import ExploreStatus
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
MODE = 'SLAM_AUTO_MAPPING'
RUNNING = (ExploreStatus.EXPLORATION_STARTED, ExploreStatus.EXPLORATION_IN_PROGRESS)
DONE = (ExploreStatus.EXPLORATION_COMPLETE, ExploreStatus.RETURNED_TO_ORIGIN)
STATUS_TXT = {
    ExploreStatus.EXPLORATION_STARTED: '탐색 중',
    ExploreStatus.EXPLORATION_IN_PROGRESS: '탐색 중',
    ExploreStatus.EXPLORATION_PAUSED: '멈춤',
    ExploreStatus.EXPLORATION_COMPLETE: '완료: 더 갈 frontier 없음',
    ExploreStatus.RETURNING_TO_ORIGIN: '출발점으로 돌아가는 중',
    ExploreStatus.RETURNED_TO_ORIGIN: '출발점 도착',
}


def ternary(data, free_below: int = 50) -> np.ndarray:
    """확률 지도(-1 · 0~100) → explore_lite용 3값 (-1 모름 · 0 빈 칸 · 100 막힘)."""
    a = np.asarray(data, dtype=np.int16)
    return np.where(a < 0, -1, np.where(a < free_below, 0, 100)).astype(np.int8)


def decide(mode: str, status: str, since_start: float, grace: float) -> str | None:
    """explore_lite 상태가 왔을 때 할 일: 'pause' · 'resume' · 'manual' · None."""
    if mode != MODE:
        return 'pause' if status in RUNNING else None
    if status in DONE:
        return 'resume' if since_start < grace else 'manual'
    return None


class ExploreBridge(Node):
    def __init__(self):
        super().__init__('explore_bridge')
        self.grace = self.declare_parameter('grace', 15.0).value   # [s] 시작 직후 complete 무시
        self.mode = ''
        self.started = 0.0
        self.retry = None   # 한 번만 도는 재시도 타이머
        self.pub_resume = self.create_publisher(Bool, 'explore/resume', 10)
        self.pub_status = self.create_publisher(String, 'explore_status', 10)
        self.pub_mode = self.create_publisher(String, 'cmd_manual_control', 10)
        self.create_subscription(String, 'state_control', self.on_mode, LATCHED)
        self.create_subscription(ExploreStatus, 'explore/status', self.on_status,
                                 QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_map = self.create_publisher(OccupancyGrid, 'map_explore', 1)
        # volatile 구독: 발행자가 latched든 아니든 붙음 (cartographer는 지도를 주기적으로 다시 냄)
        self.create_subscription(OccupancyGrid, 'map', self.on_map,
                                 QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE))

    def on_map(self, msg: OccupancyGrid) -> None:
        out = OccupancyGrid(header=msg.header, info=msg.info)
        out.data = array('b', ternary(msg.data).tobytes())
        self.pub_map.publish(out)

    def resume(self, on: bool) -> None:
        self.pub_resume.publish(Bool(data=on))

    def on_mode(self, msg: String) -> None:
        if msg.data == MODE and self.mode != MODE:
            self.started = time.monotonic()
            self.get_logger().info('자동 맵핑 시작 → explore_lite resume')
            self.resume(True)
        elif msg.data != MODE and self.mode == MODE:
            self.get_logger().info(f'제어 상태 {msg.data} → explore_lite 멈춤')
            self.resume(False)
        self.mode = msg.data

    def on_status(self, msg: ExploreStatus) -> None:
        if self.mode == MODE:
            self.pub_status.publish(String(data=STATUS_TXT.get(msg.status, msg.status)))
        action = decide(self.mode, msg.status, time.monotonic() - self.started, self.grace)
        if action == 'pause':
            self.resume(False)
        elif action == 'resume':
            self.pub_status.publish(String(data='확인 중: 지도가 아직 작음 — 다시 탐색'))
            if self.retry is None:
                self.retry = self.create_timer(2.0, self._retry_once)
        elif action == 'manual':
            self.get_logger().info('자동 맵핑 완료 — 지도를 저장하세요 (run-cli map save)')
            self.pub_mode.publish(String(data='MANUAL'))

    def _retry_once(self) -> None:
        # 2초 뒤 한 번만 — SLAM 지도가 조금 자란 다음 다시 시도
        self.destroy_timer(self.retry)
        self.retry = None
        if self.mode == MODE:
            self.resume(True)


def main(args=None):
    rclpy.init(args=args)
    node = ExploreBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
