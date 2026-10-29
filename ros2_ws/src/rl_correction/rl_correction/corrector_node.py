"""Movement correction RL node (documents/isaac/task-spec.md v2).

obs  [v_cmd, w_cmd, v_enc, w_enc, w_gyro, t_change] -> ONNX -> [dV, dW] (physical) -> add -> clip -> cmd_vel_out
  v_enc, w_enc : /odom twist (opencr_driver = wheel encoders x nominal radius/separation) — same as the sim
  w_gyro       : /imu angular_velocity.z (OpenCR on-board gyro)
  t_change     : seconds since the command changed (clipped to 2 s)
Zero command -> zero output (no creep). No model / dim mismatch -> transparent passthrough.
Limits: Waffle Pi 0.26 m/s, 1.82 rad/s (isaac/tasks/common/robot_spec.py).
"""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

T_CHANGE_MAX = 2.0   # [s] same clip as the sim
CHANGE_EPS = 1e-3    # command counts as changed above this


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
        self.declare_parameter('max_lin', 0.26)
        self.declare_parameter('max_ang', 1.82)
        self.session = None
        self.in_name = None
        self.obs_dim = 0
        self.v_enc = self.w_enc = self.w_gyro = 0.0
        self.last_cmd = (0.0, 0.0)
        self.changed_at = time.monotonic()
        self._load_model()
        self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.create_subscription(Imu, 'imu', self.cb_imu, 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_in', self.cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)

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
            self.get_logger().info(f'ONNX loaded: {path} (obs_dim={self.obs_dim})')
        except Exception as e:  # keep node alive as passthrough
            self.get_logger().warning(f'ONNX load failed ({e}): passthrough')

    def cb_odom(self, msg):
        self.v_enc = msg.twist.twist.linear.x
        self.w_enc = msg.twist.twist.angular.z

    def cb_imu(self, msg):
        self.w_gyro = msg.angular_velocity.z

    def cb(self, msg):
        cmd = (msg.linear.x, msg.angular.z)
        now = time.monotonic()
        if abs(cmd[0] - self.last_cmd[0]) > CHANGE_EPS or abs(cmd[1] - self.last_cmd[1]) > CHANGE_EPS:
            self.changed_at = now
        self.last_cmd = cmd
        delta = (0.0, 0.0)
        if self.get_parameter('enable').value and self.session is not None:
            obs = [cmd[0], cmd[1], self.v_enc, self.w_enc, self.w_gyro, min(now - self.changed_at, T_CHANGE_MAX)]
            if len(obs) == self.obs_dim:
                out = self.session.run(None, {self.in_name: np.array([obs], dtype=np.float32)})[0][0]
                delta = (float(out[0]), float(out[1]))
            else:
                self.get_logger().warning(
                    f'obs dim {len(obs)} != model {self.obs_dim}: passthrough (v1 모델? v2 = 6)', throttle_duration_sec=5.0)
        lin, ang = correct(cmd, delta, self.get_parameter('max_lin').value, self.get_parameter('max_ang').value)
        out = Twist()
        out.linear.x, out.angular.z = lin, ang
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RlCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
