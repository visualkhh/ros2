"""Nav2 controller-stage correction RL node (task-spec v1).

obs  [v_nav, w_nav, d_front, d_left, d_right] -> ONNX -> [dV, dW] -> clip
Distances from /scan sectors (m, clipped to max_obs_dist).
No model (or dim mismatch): transparent passthrough.

Placed BEFORE the base, on the Nav2 branch:
  controller_server /cmd_vel -> cmd_vel_nav_in -> [policy + clip] -> cmd_vel_nav_out
Safety: output always clipped to Nav2 command limits.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Nav2RlCorrector(Node):
    def __init__(self):
        super().__init__('nav2_rl_corrector')
        self.declare_parameter('enable', False)
        self.declare_parameter('model_path', '')
        self.declare_parameter('max_lin', 0.22)
        self.declare_parameter('max_ang', 2.84)
        self.declare_parameter('max_obs_dist', 8.0)  # task-spec clip
        self.declare_parameter('sector_half_deg', 15.0)  # front +-15 deg
        self.session = None
        self.in_name = None
        self.obs_dim = 0
        self.d_front = self.d_left = self.d_right = 8.0
        self._load_model()
        self.create_subscription(LaserScan, 'scan', self.cb_scan, 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_nav_in', self.cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_nav_out', 10)

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
        except Exception as e:
            self.get_logger().warning(f'ONNX load failed ({e}): passthrough')

    def _sector_min(self, msg, center, half):
        """Min valid range within [center-half, center+half] rad, else max_obs_dist."""
        max_d = self.get_parameter('max_obs_dist').value
        n = len(msg.ranges)
        if n == 0:
            return max_d
        a = msg.angle_min
        inc = msg.angle_increment
        best = max_d
        i0 = max(0, int((center - half - a) / inc))
        i1 = min(n - 1, int((center + half - a) / inc))
        for i in range(i0, i1 + 1):
            r = msg.ranges[i]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max and r < best:
                best = r
        return min(best, max_d)

    def cb_scan(self, msg):
        half = math.radians(self.get_parameter('sector_half_deg').value)
        # angle 0 = front (+x). left = +90deg, right = -90deg.
        self.d_front = self._sector_min(msg, 0.0, half)
        self.d_left = self._sector_min(msg, math.pi / 2, half)
        self.d_right = self._sector_min(msg, -math.pi / 2, half)

    def _infer_delta(self, obs):
        x = np.array([obs], dtype=np.float32)
        out = self.session.run(None, {self.in_name: x})[0]
        return float(out[0][0]), float(out[0][1])

    def cb(self, msg):
        lin, ang = msg.linear.x, msg.angular.z
        if self.get_parameter('enable').value and self.session is not None:
            obs = [lin, ang, self.d_front, self.d_left, self.d_right]
            if len(obs) == self.obs_dim:
                dlin, dang = self._infer_delta(obs)
                lin, ang = lin + dlin, ang + dang
            else:
                self.get_logger().warning(
                    f'obs dim {len(obs)} != model {self.obs_dim}: passthrough', throttle_duration_sec=5.0)
        max_lin = self.get_parameter('max_lin').value
        max_ang = self.get_parameter('max_ang').value
        out = Twist()
        out.linear.x = max(-max_lin, min(max_lin, lin))
        out.angular.z = max(-max_ang, min(max_ang, ang))
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2RlCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
