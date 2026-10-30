"""02 — 센서 싱크 (real, PC에서 실행).

로봇에 같은 9초 프로파일을 보내고 `/odom`·`/imu`를 CSV로 기록한다.
sim 쪽 `02_sensor_sync.py`와 같은 열/프로파일.

사전:
  Pi: base + cmd_manager, 제어 MANUAL
      cd ~/ros2/tool && ./run-cli base start && ./run-cli mode manual
  PC: source /opt/ros/lyrical/setup.bash
      export ROS_DOMAIN_ID=25
      source tool/peers.sh

실행:
  ./steps/02_sensor_sync_real.sh --label real_stand
  # 바닥이면 --label real_floor
출력 기본: steps/artifacts/<label>.csv
"""
import argparse
import csv
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

TOTAL = 9.0
HERE = Path(__file__).resolve().parent


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cmd_at(t, speed, turn):
    if t < 2.0:
        return 0.0, 0.0
    if t < 5.0:
        return speed, 0.0
    if t < 8.0:
        return 0.0, turn
    return 0.0, 0.0


class SensorSync(Node):
    def __init__(self):
        super().__init__("sensor_sync")
        self.state = None
        self.odom = None
        self.imu_w = 0.0
        self.imu_seen = False
        self.wheel_l_vel = self.wheel_r_vel = 0.0
        self.wheel_l_pos = self.wheel_r_pos = 0.0
        self.joint_seen = False
        self.out = (0.0, 0.0)
        self.pub = self.create_publisher(Twist, "/cmd_manual_vel", 10)
        # /state_control은 TRANSIENT_LOCAL (latched) — 기본 volatile이면 현재 상태를 못 받음
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/state_control", lambda m: setattr(self, "state", m.data), state_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Imu, "/imu", self.on_imu, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint, 10)
        self.create_subscription(Twist, "/cmd_vel_base", self.on_out, 10)

    def on_odom(self, m):
        self.odom = m

    def on_imu(self, m):
        self.imu_w = m.angular_velocity.z
        self.imu_seen = True

    def on_joint(self, m):
        self.joint_seen = True
        for name, pos, vel in zip(m.name, m.position, m.velocity):
            if name == "wheel_left_joint":
                self.wheel_l_pos, self.wheel_l_vel = pos, vel
            elif name == "wheel_right_joint":
                self.wheel_r_pos, self.wheel_r_vel = pos, vel

    def on_out(self, m):
        self.out = (m.linear.x, m.angular.z)

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


def main(args):
    rclpy.init()
    node = SensorSync()
    try:
        end = time.monotonic() + 5.0
        while node.state is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f"[02-real] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')} · state={node.state}")
        if args.check_only:
            return 0
        if node.state != "MANUAL":
            print("[02-real] 거부: MANUAL 아님 — Pi에서 ./run-cli mode manual 먼저")
            return 2
        end = time.monotonic() + 3.0
        while node.odom is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is None:
            print("[02-real] 거부: /odom 없음 — base/드라이버 확인")
            return 2

        rows = []
        rate = args.rate
        n = int(round(TOTAL * rate))
        t0 = time.monotonic()
        out = args.output or str(HERE / "artifacts" / f"{args.label}.csv")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        print(f"[02-real] 시작 · {TOTAL:.0f}s · {rate:.0f}Hz · {args.label} → {out}")
        for k in range(n):
            t = k / rate
            v, w = cmd_at(t, args.speed, args.turn)
            node.send(v, w)
            next_t = t0 + (k + 1) / rate
            while time.monotonic() < next_t:
                rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, next_t - time.monotonic())))

            o = node.odom
            p = o.pose.pose
            rows.append([round(t, 3), v, w, node.out[0], node.out[1],
                         p.position.x, p.position.y, yaw_of(p.orientation),
                         o.twist.twist.linear.x, o.twist.twist.angular.z,
                         node.imu_w,
                         node.wheel_l_vel, node.wheel_r_vel,
                         node.wheel_l_pos, node.wheel_r_pos,
                         args.label])

        for _ in range(3):
            node.send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

        with open(out, "w", newline="") as f:
            cw = csv.writer(f)
            cw.writerow(["t", "lin_raw", "ang_raw", "lin_out", "ang_out",
                         "odom_x", "odom_y", "odom_yaw", "v_enc", "w_enc", "w_gyro",
                         "wheel_l_vel", "wheel_r_vel", "wheel_l_pos", "wheel_r_pos", "label"])
            cw.writerows(rows)

        def seg(a, b):
            part = [r for r in rows if a <= r[0] < b]
            return (sum(r[8] for r in part) / len(part),
                    sum(r[9] for r in part) / len(part),
                    sum(r[10] for r in part) / len(part))
        vf, wf, gf = seg(2.5, 4.5)
        vt, wt, gt = seg(5.5, 7.5)
        def seg_wheel(a, b):
            part = [r for r in rows if a <= r[0] < b]
            return (sum(r[11] for r in part) / len(part),
                    sum(r[12] for r in part) / len(part))
        lf, rf = seg_wheel(2.5, 4.5)
        lt, rt = seg_wheel(5.5, 7.5)
        print(f"[02-real] 직진구간  v_enc={vf:+.3f} w_enc={wf:+.3f} w_gyro={gf:+.3f} "
              f"| 바퀴 wl={lf:+.2f} wr={rf:+.2f} rad/s")
        print(f"[02-real] 회전구간  v_enc={vt:+.3f} w_enc={wt:+.3f} w_gyro={gt:+.3f} "
              f"| 바퀴 wl={lt:+.2f} wr={rt:+.2f} rad/s")
        if not node.imu_seen:
            print("[02-real] 경고: /imu 없음 — w_gyro가 0")
        if not node.joint_seen:
            print("[02-real] 경고: /joint_states 없음 — 바퀴 raw가 0")
        print(f"[02-real] → {out}")
        return 0
    except KeyboardInterrupt:
        for _ in range(3):
            node.send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        print("\n[02-real] 중단 — 정지")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="02: real 센서 싱크")
    p.add_argument("--output", default=None, help="기본: steps/artifacts/<label>.csv")
    p.add_argument("--label", default="real")
    p.add_argument("--speed", type=float, default=0.15)
    p.add_argument("--turn", type=float, default=1.0)
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--check-only", action="store_true", help="상태만 확인하고 종료 (로봇 안 움직임)")
    raise SystemExit(main(p.parse_args()))
