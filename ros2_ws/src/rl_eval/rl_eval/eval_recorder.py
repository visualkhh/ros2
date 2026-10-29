"""Before/After metrics recorder.

Subscribes raw vs corrected cmd_vel and odom, appends one CSV row per raw msg:
  t, lin_raw, ang_raw, lin_out, ang_out, odom_x, odom_y, odom_yaw, v_enc, w_enc, w_gyro, label
  (v_enc/w_enc = /odom twist = wheel encoders, w_gyro = /imu z — what the v2 policy is judged on)

Run twice (enable=false / enable=true) with different --ros-args -p output:=
and compare: python3 compare_csv.py /tmp/before.csv /tmp/after.csv
"""
import csv
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class EvalRecorder(Node):
    def __init__(self):
        super().__init__('eval_recorder')
        self.declare_parameter('output', '/tmp/rl_eval.csv')
        self.declare_parameter('label', 'before')
        self.raw = None
        self.out = None
        self.odom = None
        self.w_gyro = 0.0
        self.f = open(self.get_parameter('output').value, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'lin_raw', 'ang_raw', 'lin_out', 'ang_out',
                        'odom_x', 'odom_y', 'odom_yaw', 'v_enc', 'w_enc', 'w_gyro', 'label'])
        self.create_subscription(Twist, 'cmd_vel_raw', self.cb_raw, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cb_out, 10)
        self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.create_subscription(Imu, 'imu', lambda m: setattr(self, 'w_gyro', m.angular_velocity.z), 10)

    def _flush(self):
        if self.raw is None:
            return
        ox, oy, yaw, ve, we = 0.0, 0.0, 0.0, 0.0, 0.0
        if self.odom is not None:
            ve, we = self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z
            p = self.odom.pose.pose
            ox, oy = p.position.x, p.position.y
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
        lo = self.out.linear.x if self.out else 0.0
        ao = self.out.angular.z if self.out else 0.0
        self.w.writerow([self.get_clock().now().nanoseconds / 1e9,
                        self.raw.linear.x, self.raw.angular.z,
                        lo, ao, ox, oy, yaw, ve, we, self.w_gyro,
                        self.get_parameter('label').value])

    def cb_raw(self, msg):
        self.raw = msg
        self._flush()

    def cb_out(self, msg):
        self.out = msg

    def cb_odom(self, msg):
        self.odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = EvalRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()
