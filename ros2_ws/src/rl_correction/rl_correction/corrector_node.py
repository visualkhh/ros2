"""Movement correction RL node (documents/isaac/task-spec.md).

두 모드를 지원한다 (ONNX 입력 차원으로 자동 판별):

  track (obs 6)
    [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change] -> ONNX -> [dV,dW]
    /cmd_vel_in -> 보정 -> /cmd_vel_out

  goal (obs 10)
    [gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_dv, prev_dw]
    /rl_goal (Point, 시작 자세 기준 목표)을 받으면
    nominal controller + ONNX 보정 -> /cmd_vel_out

공통:
  /odom twist·pose, /imu angular z 사용. zero 명령은 zero 출력.
  No model / dim mismatch -> track은 passthrough, goal은 정지.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty

T_CHANGE_MAX = 2.0   # [s] same clip as the sim
CHANGE_EPS = 1e-3    # command counts as changed above this
GOAL_TOPIC = 'rl_goal'
STOP_TOPIC = 'rl_goal_stop'


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def correct(cmd, delta, max_lin, max_ang):
    """Add correction and clip. Zero command stays zero (sim applies the same rule)."""
    lin, ang = cmd
    if lin == 0.0 and ang == 0.0:
        return 0.0, 0.0
    lin, ang = lin + delta[0], ang + delta[1]
    return max(-max_lin, min(max_lin, lin)), max(-max_ang, min(max_ang, ang))


class RlCorrector(Node):
    def __init__(self):
        super().__init__('rl_corrector')
        self.declare_parameter('enable', False)
        self.declare_parameter('model_path', '')
        self.declare_parameter('mode', 'auto')  # auto | track | goal
        self.declare_parameter('max_lin', 0.26)
        self.declare_parameter('max_ang', 1.82)
        # goal mode (학습 cfg와 같은 기본값)
        self.declare_parameter('goal_tol', 0.05)
        self.declare_parameter('k_dist', 1.0)
        self.declare_parameter('k_heading', 2.0)
        self.declare_parameter('turn_thresh', 0.5)
        self.declare_parameter('action_scale_lin', 0.10)
        self.declare_parameter('action_scale_ang', 0.60)

        self.session = None
        self.in_name = None
        self.obs_dim = 0
        self.mode = 'track'
        self.v_enc = self.w_enc = self.w_gyro = 0.0
        self.odom_pose = None          # (x, y, yaw)
        self.goal = None               # (gx, gy) 시작 자세 기준
        self.start_pose = None
        self.goal_active = False
        self.prev_action = (0.0, 0.0)  # 정규화 [-1,1]
        self.last_cmd = (0.0, 0.0)
        self.changed_at = time.monotonic()
        self._load_model()

        self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.create_subscription(Imu, 'imu', self.cb_imu, 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_in', self.cb, 10)
        self.create_subscription(Point, GOAL_TOPIC, self.cb_goal, 10)
        self.create_subscription(Empty, STOP_TOPIC, self.cb_goal_stop, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.create_timer(0.05, self.tick_goal)

    def _load_model(self):
        path = self.get_parameter('model_path').value
        if not path:
            self.get_logger().info('No model_path: passthrough mode')
            return
        try:
            import onnxruntime as ort
            self.session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
            self.in_name = self.session.get_inputs()[0].name
            self.obs_dim = int(self.session.get_inputs()[0].shape[1])
            requested = self.get_parameter('mode').value
            if requested == 'auto':
                self.mode = 'goal' if self.obs_dim == 10 else 'track'
            else:
                self.mode = requested
            if self.mode == 'goal' and self.obs_dim != 10:
                self.get_logger().warning(f'goal mode인데 obs_dim={self.obs_dim} (10이어야 함)')
            if self.mode == 'track' and self.obs_dim != 6:
                self.get_logger().warning(f'track mode인데 obs_dim={self.obs_dim} (6이어야 함)')
            self.get_logger().info(
                f'ONNX loaded: {path} (obs_dim={self.obs_dim}, mode={self.mode})')
        except Exception as e:  # keep node alive as passthrough
            self.get_logger().warning(f'ONNX load failed ({e}): passthrough')

    def cb_odom(self, msg):
        self.v_enc = msg.twist.twist.linear.x
        self.w_enc = msg.twist.twist.angular.z
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def cb_imu(self, msg):
        self.w_gyro = msg.angular_velocity.z

    def cb_goal(self, msg: Point):
        if self.mode != 'goal':
            self.get_logger().warning(f'/rl_goal 무시: mode={self.mode}')
            return
        if self.odom_pose is None:
            self.get_logger().warning('/rl_goal 무시: /odom 아직 없음')
            return
        self.goal = (float(msg.x), float(msg.y))
        self.start_pose = self.odom_pose
        self.goal_active = True
        self.prev_action = (0.0, 0.0)
        self.get_logger().info(
            f'goal 시작: g=({self.goal[0]:+.2f}, {self.goal[1]:+.2f}) m')

    def cb_goal_stop(self, _msg: Empty):
        self.goal_active = False
        self.pub.publish(Twist())
        self.get_logger().info('goal stop')

    def cb(self, msg):
        if self.mode != 'track':
            return
        cmd = (msg.linear.x, msg.angular.z)
        now = time.monotonic()
        if abs(cmd[0] - self.last_cmd[0]) > CHANGE_EPS or abs(cmd[1] - self.last_cmd[1]) > CHANGE_EPS:
            self.changed_at = now
        self.last_cmd = cmd
        delta = (0.0, 0.0)
        if self.get_parameter('enable').value and self.session is not None:
            obs = [cmd[0], cmd[1], self.v_enc, self.w_enc, self.w_gyro,
                   min(now - self.changed_at, T_CHANGE_MAX)]
            if len(obs) == self.obs_dim:
                out = self.session.run(None, {self.in_name: np.array([obs], dtype=np.float32)})[0][0]
                delta = (float(out[0]), float(out[1]))
            else:
                self.get_logger().warning(
                    f'obs dim {len(obs)} != model {self.obs_dim}: passthrough',
                    throttle_duration_sec=5.0)
        lin, ang = correct(cmd, delta, self.get_parameter('max_lin').value,
                           self.get_parameter('max_ang').value)
        out = Twist()
        out.linear.x, out.angular.z = lin, ang
        self.pub.publish(out)

    # ------------------------------------------------------------------ goal
    def _rel_pose(self):
        x, y, yaw = self.odom_pose
        x0, y0, yaw0 = self.start_pose
        dx, dy = x - x0, y - y0
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        return c * dx - s * dy, s * dx + c * dy, wrap_to_pi(yaw - yaw0)

    def _goal_cmd(self, dist, herr):
        c = self.get_parameter
        v = min(c('max_lin').value, c('k_dist').value * dist)
        w = max(-c('max_ang').value,
                min(c('max_ang').value, c('k_heading').value * herr))
        if abs(herr) > c('turn_thresh').value or dist < c('goal_tol').value:
            v = 0.0
        if dist < c('goal_tol').value:
            w = 0.0
        return v, w

    def tick_goal(self):
        if self.mode != 'goal' or not self.goal_active:
            return
        if not self.get_parameter('enable').value or self.session is None:
            self.pub.publish(Twist())
            return
        if self.odom_pose is None or self.start_pose is None or self.goal is None:
            self.pub.publish(Twist())
            return

        px, py, yaw = self._rel_pose()
        gx, gy = self.goal
        ex, ey = gx - px, gy - py
        dist = math.hypot(ex, ey)
        herr = wrap_to_pi(math.atan2(ey, ex) - yaw)
        v, w = self._goal_cmd(dist, herr)

        obs = [gx, gy, px, py, yaw, self.v_enc, self.w_enc, self.w_gyro,
               self.prev_action[0], self.prev_action[1]]
        if len(obs) != self.obs_dim:
            self.get_logger().warning(
                f'obs dim {len(obs)} != model {self.obs_dim}: stop', throttle_duration_sec=5.0)
            self.pub.publish(Twist())
            return

        out = self.session.run(None, {self.in_name: np.array([obs], dtype=np.float32)})[0][0]
        delta = (float(out[0]), float(out[1]))
        sl = max(1e-6, self.get_parameter('action_scale_lin').value)
        sa = max(1e-6, self.get_parameter('action_scale_ang').value)
        self.prev_action = (max(-1.0, min(1.0, delta[0] / sl)),
                            max(-1.0, min(1.0, delta[1] / sa)))

        lin, ang = correct((v, w), delta, self.get_parameter('max_lin').value,
                           self.get_parameter('max_ang').value)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = lin, ang
        self.pub.publish(cmd)

        if dist < self.get_parameter('goal_tol').value:
            self.goal_active = False
            self.pub.publish(Twist())
            self.get_logger().info(
                f'goal 도착: err={dist:.3f} m, p=({px:+.2f}, {py:+.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = RlCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
