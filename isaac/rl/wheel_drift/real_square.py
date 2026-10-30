"""real goal square — RL goal 정책으로 보정값(sim_calib_floor와 같은 네모) 주행.

먼저 PC에서 `real_launch.sh`로 goal node를 띄우고 (goal_mode=absolute),
Pi는 `./run-cli route rl` 상태여야 한다.

  ./rl/wheel_drift/real_square.sh
  → steps/artifacts/real_goal_square.csv
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parents[1]
sys.path.insert(0, str(ISAAC))

from rl.wheel_drift.calib_profile import DEFAULT_SIDE, make_waypoints  # noqa: E402

DEFAULT_OUT = ISAAC / "steps" / "artifacts" / "real_goal_square.csv"


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SquareRunner(Node):
    def __init__(self):
        super().__init__("goal_square_runner")
        self.pose = None
        self.twist = (0.0, 0.0)
        self.gyro = 0.0
        self.cmd = (0.0, 0.0)
        self.jl = self.jr = 0.0
        self.jpl = self.jpr = 0.0
        self.create_subscription(Odometry, "/odom", self.cb_odom, 10)
        self.create_subscription(Imu, "/imu", self.cb_imu, 10)
        self.create_subscription(Twist, "/cmd_vel_rl_out", self.cb_cmd, 10)
        self.create_subscription(JointState, "/joint_states", self.cb_joint, 10)
        self.pub = self.create_publisher(Point, "/rl_goal", 10)
        self.stop = self.create_publisher(Empty, "/rl_goal_stop", 10)

    def cb_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.twist = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def cb_imu(self, m):
        self.gyro = m.angular_velocity.z

    def cb_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def cb_joint(self, m):
        for name, pos, vel in zip(m.name, m.position, m.velocity):
            if name == "wheel_left_joint":
                self.jpl, self.jl = pos, vel
            elif name == "wheel_right_joint":
                self.jpr, self.jr = pos, vel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", type=float, default=DEFAULT_SIDE)
    ap.add_argument("--tolerance", type=float, default=0.07)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args()

    rclpy.init()
    n = SquareRunner()
    while n.pose is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    # absolute mode 시작점 초기화
    n.stop.publish(Empty())
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.02)

    start = n.pose
    rows = []
    t0 = time.monotonic()

    def record(gx, gy):
        x, y, yaw = n.pose
        rows.append([round(time.monotonic() - t0, 3), gx, gy, n.cmd[0], n.cmd[1],
                     x, y, yaw, n.twist[0], n.twist[1], n.gyro,
                     n.jl, n.jr, n.jpl, n.jpr, "real_goal_square"])

    def rel():
        x, y, yaw = n.pose
        x0, y0, yaw0 = start
        dx, dy = x - x0, y - y0
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        return c * dx - s * dy, s * dx + c * dy, wrap(yaw - yaw0)

    side = args.side
    goals = make_waypoints(side)

    for k, (gx, gy) in enumerate(goals, 1):
        n.pub.publish(Point(x=gx, y=gy, z=0.0))
        print(f"[{k}/{len(goals)}] goal ({gx:+.2f},{gy:+.2f})", flush=True)
        t_goal = time.monotonic()
        px = py = 0.0
        reached = False
        while time.monotonic() - t_goal < args.timeout:
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.pose is None:
                continue
            px, py, _yaw = rel()
            record(gx, gy)
            d = math.hypot(gx - px, gy - py)
            if d < args.tolerance:
                reached = True
                break
            if math.hypot(px, py) > 2.0:
                print("  safety: too far -> stop", flush=True)
                break
        print(f"  reached={reached} err={d:.3f} p=({px:+.2f},{py:+.2f})", flush=True)
        if math.hypot(px, py) > 2.0:
            break
        # 다음 goal 전에 0.25 s 정지(기록은 계속)
        end = time.monotonic() + 0.25
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.pose is not None:
                record(gx, gy)

    n.stop.publish(Empty())
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.02)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(["t", "lin_raw", "ang_raw", "lin_out", "ang_out",
                     "odom_x", "odom_y", "odom_yaw", "v_enc", "w_enc", "w_gyro",
                     "wheel_l_vel", "wheel_r_vel", "wheel_l_pos", "wheel_r_pos", "label"])
        cw.writerows(rows)
    print(f"→ {out} ({len(rows)} rows)")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
