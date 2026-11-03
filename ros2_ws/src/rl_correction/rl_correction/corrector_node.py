"""Movement correction RL node (documents/isaac/task-spec.md).

두 모드를 지원한다 (ONNX 입력 차원으로 자동 판별):

  track (obs 6)
    [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change] -> ONNX -> [-1, 1] 행동 -> clamp × action_scale -> [dV,dW]
    (Isaac Lab play.py가 내보낸 policy.onnx = 신경망 출력 그대로 — 스케일은 여기서. 2026-11-03 이전 파일은 스케일이 들어 있어 쓰지 말 것)
    /cmd_vel_in -> 보정 -> /cmd_vel_out

  goal (obs 10) — isaac/training/rl_correction_waypoint 와 같은 약속
    [gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_a0, prev_a1]
    자세(px, py, yaw) = 시작 자세 기준 추정 — 거리는 엔코더(/odom twist), 방향은 자이로(/imu) 적분 (20 Hz)
      (/odom pose는 엔코더만이라 바퀴 gain 차이로 휘어도 안 보인다 → 쓰지 않는다)
    /rl_goal (Point)을 받으면 nominal controller + ONNX 보정 (clamp × action_scale) -> /cmd_vel_out
    goal_mode=absolute (기본): stop 전까지 첫 goal의 시작 자세를 고정 — 웨이포인트를 하나씩 보낸다 (학습과 같음)
    goal_mode=relative: goal마다 현재 자세가 시작점

공통:
  /odom twist·pose, /imu angular z 사용. zero 명령은 zero 출력.
  No model / dim mismatch -> track은 passthrough, goal은 정지.

실행 중 전환 (ROS 파라미터 — 한 번 실행에 A/B):
  ros2 param set /rl_corrector enable false|true      입력마다 다시 읽음 (false = 바이패스)
  ros2 param set /rl_corrector model_path <onnx>      즉시 다시 로드. 없는 경로·빈 값이면 바이패스
  (ros2_ws/policy.sh 가 이걸 감싼다)
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float32MultiArray

T_CHANGE_MAX = 2.0   # [s] same clip as the sim
CHANGE_EPS = 1e-3    # command counts as changed above this
GOAL_TOPIC = 'rl_goal'
STOP_TOPIC = 'rl_goal_stop'


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def scale_action(out, scale_lin: float, scale_ang: float) -> tuple[float, float]:
    """정책 출력 [-1, 1] → 물리 보정량 [dV m/s, dW rad/s] (학습 env와 같은 clamp × action_scale)."""
    return (max(-1.0, min(1.0, float(out[0]))) * scale_lin, max(-1.0, min(1.0, float(out[1]))) * scale_ang)


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
        self.declare_parameter('goal_mode', 'absolute')  # absolute (웨이포인트, 학습과 같음) | relative
        self.declare_parameter('max_lin', 0.26)
        self.declare_parameter('max_ang', 1.82)
        # goal mode (학습 cfg와 같은 기본값)
        self.declare_parameter('goal_tol', 0.05)
        self.declare_parameter('k_dist', 1.0)
        self.declare_parameter('k_heading', 2.0)
        self.declare_parameter('turn_thresh', 0.5)
        self.declare_parameter('action_scale_lin', 0.10)
        self.declare_parameter('action_scale_ang', 0.60)
        self.declare_parameter('log_period', 1.0)  # [s] 0이면 매 스텝

        self.log_period = max(0.0, float(self.get_parameter('log_period').value))
        self.session = None
        self.in_name = None
        self.obs_dim = 0
        self.mode = 'track'
        self.v_enc = self.w_enc = self.w_gyro = 0.0
        self.odom_seen = False
        self.est = (0.0, 0.0, 0.0)     # 시작 자세 기준 추정 (x, y, yaw) — 엔코더 거리 + 자이로 방향
        self.est_t = None              # 마지막 적분 시각
        self.goal = None               # (gx, gy) 시작 자세 기준
        self.start_pose = None         # 추정을 시작했으면 True (absolute: stop 전까지 유지)
        self.goal_active = False
        self.prev_action = (0.0, 0.0)  # 정규화 [-1,1]
        self.last_cmd = (0.0, 0.0)
        self.changed_at = time.monotonic()
        self._load_model(self.get_parameter('model_path').value)
        self.add_on_set_parameters_callback(self._on_params)

        self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.create_subscription(Imu, 'imu', self.cb_imu, 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_in', self.cb, 10)
        self.create_subscription(Point, GOAL_TOPIC, self.cb_goal, 10)
        self.create_subscription(Empty, STOP_TOPIC, self.cb_goal_stop, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.pub_dbg = self.create_publisher(Float32MultiArray, 'rl_debug', 10)
        self.create_timer(0.05, self.tick_goal)

    def _on_params(self, params):
        """model_path가 바뀌면 바로 다시 로드 (실패해도 노드는 살아서 바이패스)."""
        for p in params:
            if p.name == 'model_path':
                self._load_model(p.value)
        return SetParametersResult(successful=True)

    def _load_model(self, path: str):
        self.session, self.in_name, self.obs_dim = None, None, 0   # 실패·빈 경로 = 바이패스
        if not path:
            self.get_logger().info('No model_path: passthrough mode')
            return
        try:
            import onnxruntime as ort
            sess = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
            in_name, obs_dim = sess.get_inputs()[0].name, int(sess.get_inputs()[0].shape[1])
            self.session, self.in_name, self.obs_dim = sess, in_name, obs_dim   # 다 읽은 뒤에만 교체
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
                f'ONNX loaded: {path} (obs_dim={self.obs_dim}, mode={self.mode}, '
                f'action_scale=({self.get_parameter("action_scale_lin").value:.3f}, '
                f'{self.get_parameter("action_scale_ang").value:.3f}), '
                f'goal_mode={self.get_parameter("goal_mode").value}, '
                f'log_period={self.log_period:.2f}s)')
        except Exception as e:  # keep node alive as passthrough
            self.get_logger().warning(f'ONNX load failed ({e}): passthrough')

    def cb_odom(self, msg):
        self.v_enc = msg.twist.twist.linear.x
        self.w_enc = msg.twist.twist.angular.z
        self.odom_seen = True

    def cb_imu(self, msg):
        self.w_gyro = msg.angular_velocity.z

    def cb_goal(self, msg: Point):
        if self.mode != 'goal':
            self.get_logger().warning(f'/rl_goal 무시: mode={self.mode}')
            return
        if not self.odom_seen:
            self.get_logger().warning('/rl_goal 무시: /odom 아직 없음')
            return
        self.goal = (float(msg.x), float(msg.y))
        # absolute: stop 전까지 첫 goal의 시작 자세를 고정 (waypoint 경로용)
        if self.get_parameter('goal_mode').value != 'absolute' or self.start_pose is None:
            self.start_pose, self.est, self.est_t = True, (0.0, 0.0, 0.0), time.monotonic()
        self.goal_active = True
        self.prev_action = (0.0, 0.0)
        self.get_logger().info(
            f'goal 시작: g=({self.goal[0]:+.2f}, {self.goal[1]:+.2f}) m '
            f'frame={self.get_parameter("goal_mode").value}')

    def cb_goal_stop(self, _msg: Empty):
        self.goal_active = False
        self.start_pose = None
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
                delta = scale_action(out, self.get_parameter('action_scale_lin').value,
                                     self.get_parameter('action_scale_ang').value)
            else:
                self.get_logger().warning(
                    f'obs dim {len(obs)} != model {self.obs_dim}: passthrough',
                    throttle_duration_sec=5.0)
        lin, ang = correct(cmd, delta, self.get_parameter('max_lin').value,
                           self.get_parameter('max_ang').value)
        out = Twist()
        out.linear.x, out.angular.z = lin, ang
        self.pub.publish(out)
        self.pub_dbg.publish(Float32MultiArray(data=[
            0.0, cmd[0], cmd[1], self.v_enc, self.w_enc, self.w_gyro, 0.0, 0.0,
            cmd[0], cmd[1], delta[0], delta[1], lin, ang,
            lin - cmd[0], ang - cmd[1],
        ]))

        self.get_logger().info(
            f'[track] in=({cmd[0]:+.3f},{cmd[1]:+.3f}) '
            f'odom=({self.v_enc:+.3f},{self.w_enc:+.3f}) gyro={self.w_gyro:+.3f} '
            f'model=({delta[0]:+.3f},{delta[1]:+.3f}) '
            f'out=({lin:+.3f},{ang:+.3f}) '
            f'applied=({lin - cmd[0]:+.3f},{ang - cmd[1]:+.3f})',
            throttle_duration_sec=self.log_period)

    # ------------------------------------------------------------------ goal
    def _rel_pose(self):
        """추정 자세 적분 (학습 env _sense_and_advance와 같은 식): 방향 += 자이로 · dt, 위치 += 엔코더 v · (중간 방향)."""
        now = time.monotonic()
        dt = min(0.2, now - self.est_t) if self.est_t is not None else 0.0
        self.est_t = now
        x, y, yaw = self.est
        yaw_mid = yaw + 0.5 * self.w_gyro * dt
        x += self.v_enc * math.cos(yaw_mid) * dt
        y += self.v_enc * math.sin(yaw_mid) * dt
        self.est = (x, y, wrap_to_pi(yaw + self.w_gyro * dt))
        return self.est

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
        if not self.odom_seen or self.start_pose is None or self.goal is None:
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
        delta = scale_action(out, self.get_parameter('action_scale_lin').value,
                             self.get_parameter('action_scale_ang').value)
        self.prev_action = (max(-1.0, min(1.0, float(out[0]))), max(-1.0, min(1.0, float(out[1]))))

        lin, ang = correct((v, w), delta, self.get_parameter('max_lin').value,
                           self.get_parameter('max_ang').value)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = lin, ang
        self.pub.publish(cmd)
        self.pub_dbg.publish(Float32MultiArray(data=[
            1.0, gx, gy, px, py, yaw, dist, herr,
            v, w, delta[0], delta[1], lin, ang, lin - v, ang - w,
        ]))

        self.get_logger().info(
            f'[goal] g=({gx:+.2f},{gy:+.2f}) p=({px:+.2f},{py:+.2f},{yaw:+.2f}) '
            f'dist={dist:.3f} herr={herr:+.2f} '
            f'nominal=({v:+.3f},{w:+.3f}) '
            f'model=({delta[0]:+.3f},{delta[1]:+.3f}) '
            f'out=({lin:+.3f},{ang:+.3f}) '
            f'applied=({lin - v:+.3f},{ang - w:+.3f})',
            throttle_duration_sec=self.log_period)

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
