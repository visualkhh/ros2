"""frontier_explorer — 자동 맵핑. /state_control = SLAM_AUTO_MAPPING일 때만 움직이고, 그 밖엔 대기.

  /map(SLAM)에서 frontier(빈 칸 ↔ 모르는 칸 경계)를 찾아 가장 가까운 덩어리로 Nav2 NavigateToPose를 보낸다 → 도착·실패하면 다음.
  더 갈 frontier가 없으면 끝: /cmd_manual_control = MANUAL 요청 (state_manager가 모드를 바꿈).

  sub  /map (OccupancyGrid, latched) · /state_control (latched) · TF map → base_footprint
  act  navigate_to_pose (Nav2 — SLAM + `run-cli explore start`로 주행부가 떠 있어야 함)
  pub  /explore_status (String)   사람이 읽는 진행 상황 · /cmd_manual_control (끝나면 MANUAL)

외부 라이브러리 없이 numpy만 (m-explore-ros2는 Lyrical apt에 없고 라이선스 미표기).
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
MODE = 'SLAM_AUTO_MAPPING'


def frontier_clusters(grid: np.ndarray, min_size: int) -> list[list[tuple[int, int]]]:
    """빈 칸(0) 중 상하좌우에 모르는 칸(-1)이 있는 칸 → 8방향 연결 덩어리 (min_size 이상만)."""
    unknown = grid == -1
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    frontier = (grid == 0) & near_unknown
    seen = np.zeros_like(frontier)
    clusters = []
    for r, c in zip(*np.nonzero(frontier)):
        if seen[r, c]:
            continue
        seen[r, c] = True
        cells, q = [], deque([(r, c)])
        while q:
            y, x = q.popleft()
            cells.append((y, x))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1] and frontier[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        q.append((ny, nx))
        if len(cells) >= min_size:
            clusters.append(cells)
    return clusters


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        p = self.declare_parameter
        self.min_size = p('min_frontier_cells', 8).value          # 덩어리 최소 칸 수 (0.05 m 칸 → 40 cm)
        self.goal_timeout = p('goal_timeout', 60.0).value         # 한 목표 최대 [s]
        self.blacklist_radius = p('blacklist_radius', 0.4).value  # 실패한 목표 주변 제외 [m]
        self.min_goal_dist = p('min_goal_dist', 0.3).value        # 너무 가까운 frontier 무시 [m]
        # 완료 판정: frontier 없음이 이만큼 계속돼야 (SLAM 막 켠 직후 첫 지도엔 빈 칸이 거의 없어 frontier도 없음)
        self.done_after = p('done_after', 10.0).value             # [s]
        self.no_frontier_since: float | None = None
        self.map: OccupancyGrid | None = None
        self.mode = ''
        self.blacklist: list[tuple[float, float]] = []
        self.goal_handle = None
        self.goal_xy: tuple[float, float] | None = None
        self.goal_sent = 0.0
        self.pending = False
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map', m), LATCHED)
        self.create_subscription(String, 'state_control', self.on_mode, LATCHED)
        self.pub_status = self.create_publisher(String, 'explore_status', 10)
        self.pub_mode = self.create_publisher(String, 'cmd_manual_control', 10)
        self.create_timer(1.0, self.tick)

    def status(self, text: str) -> None:
        self.pub_status.publish(String(data=text))

    def on_mode(self, msg: String) -> None:
        if msg.data != MODE and self.mode == MODE:
            self.cancel('모드 변경')
        if msg.data == MODE and self.mode != MODE:
            self.blacklist.clear()
            self.no_frontier_since = None
            self.get_logger().info('자동 맵핑 시작')
        self.mode = msg.data

    def cancel(self, why: str) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.get_logger().info(f'목표 취소: {why}')
        self.goal_handle, self.goal_xy = None, None

    def robot_xy(self) -> tuple[float, float] | None:
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
            return t.x, t.y
        except Exception:
            return None

    def pick_goal(self, robot: tuple[float, float]) -> tuple[float, float, int] | None:
        m = self.map
        info = m.info
        grid = np.asarray(m.data, dtype=np.int8).reshape(info.height, info.width)
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        best = None
        clusters = frontier_clusters(grid, self.min_size)
        for cells in clusters:
            arr = np.array(cells, dtype=float)
            cy, cx = arr.mean(axis=0)
            gy, gx = cells[int(np.argmin((arr[:, 0] - cy) ** 2 + (arr[:, 1] - cx) ** 2))]  # 중심에 가장 가까운 frontier 칸
            x, y = ox + (gx + 0.5) * res, oy + (gy + 0.5) * res
            d = math.hypot(x - robot[0], y - robot[1])
            if d < self.min_goal_dist or any(math.hypot(x - bx, y - by) < self.blacklist_radius for bx, by in self.blacklist):
                continue
            if best is None or d < best[3]:
                best = (x, y, len(cells), d)
        return None if best is None else (best[0], best[1], len(clusters))

    def tick(self) -> None:
        if self.mode != MODE:
            return
        if self.map is None:
            return self.status('대기: /map 없음 (SLAM 켜기)')
        if not self.nav.server_is_ready():
            return self.status('대기: Nav2 주행부 없음 (run-cli explore start)')
        if self.pending:
            return
        if self.goal_handle is not None:
            if time.monotonic() - self.goal_sent > self.goal_timeout:
                self.blacklist.append(self.goal_xy)
                self.cancel(f'{self.goal_timeout:.0f}초 초과')
            return
        robot = self.robot_xy()
        if robot is None:
            return self.status('대기: TF map→base_footprint 없음')
        goal = self.pick_goal(robot)
        if goal is None:
            now = time.monotonic()
            self.no_frontier_since = self.no_frontier_since or now
            if now - self.no_frontier_since < self.done_after:
                return self.status(f'확인 중: frontier 없음 {now - self.no_frontier_since:.0f}/{self.done_after:.0f}초')
            self.no_frontier_since = None
            self.status('완료: 더 갈 frontier 없음 → MANUAL')
            self.get_logger().info('자동 맵핑 완료 — 지도를 저장하세요 (run-cli map save)')
            self.pub_mode.publish(String(data='MANUAL'))
            return
        self.no_frontier_since = None
        x, y, n = goal
        self.send(robot, x, y, n)

    def send(self, robot, x: float, y: float, n_clusters: int) -> None:
        yaw = math.atan2(y - robot[1], x - robot[0])
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pending, self.goal_xy, self.goal_sent = True, (x, y), time.monotonic()
        self.status(f'이동: ({x:.2f}, {y:.2f}) · frontier {n_clusters}곳 남음')
        self.nav.send_goal_async(g).add_done_callback(self.on_accepted)

    def on_accepted(self, fut) -> None:
        self.pending = False
        handle = fut.result()
        if not handle.accepted or self.mode != MODE:
            if handle.accepted:
                handle.cancel_goal_async()
            else:
                self.blacklist.append(self.goal_xy)
            self.goal_xy = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f, h=handle: self.on_result(h, f))

    def on_result(self, handle, fut) -> None:
        if handle is not self.goal_handle:   # 이미 취소·교체된 목표
            return
        if fut.result().status != 4:         # 4 = SUCCEEDED · 실패한 곳은 다시 안 감
            self.blacklist.append(self.goal_xy)
        self.goal_handle, self.goal_xy = None, None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
