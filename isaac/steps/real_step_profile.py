"""S5 — real 계단응답 기록 (PC에서 실행, Pi 로봇을 움직임).

사전:
  1) Pi: base + cmd_manager 떠 있고 제어 상태 MANUAL
     (예: ./run-cli base start  →  ./run-cli mode manual)
  2) PC: ROS 2 + 유니캐스트 피어
     source /opt/ros/lyrical/setup.bash
     export ROS_DOMAIN_ID=25
     source tool/peers.sh

실행:
  python3 isaac/steps/real_step_profile.py --output /tmp/real_step.csv

프로파일은 sim `s05_step_response.py`와 같다 (20 Hz, 9 s):
  0~1 s 정지 · 1~3 s v=0.15 · 3~5 s v=0.25 · 5~6 s 정지 · 6~8 s w=+1.0 · 8~9 s 정지

CSV 열은 eval_recorder와 같다. 끝나면 compare_step.py로 sim과 비교:
  python3 isaac/steps/compare_step.py isaac/steps/artifacts/sim_step.csv /tmp/real_step.csv
"""
import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

SEGMENTS = (  # (시작, 끝, v_kind, w_kind)
    (0.0, 1.0, 0.0, 0.0),
    (1.0, 3.0, "speed1", 0.0),
    (3.0, 5.0, "speed2", 0.0),
    (5.0, 6.0, 0.0, 0.0),
    (6.0, 8.0, 0.0, "turn"),
    (8.0, 9.0, 0.0, 0.0),
)
TOTAL = 9.0


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StepProfile(Node):
    def __init__(self, args):
        super().__init__("step_profile")
        self.args = args
        self.state = None
        self.odom = None
        self.imu_w = 0.0
        self.imu_seen = False
        self.out = (0.0, 0.0)
        self.pub = self.create_publisher(Twist, "/cmd_manual_vel", 10)
        self.create_subscription(String, "/state_control", lambda m: setattr(self, "state", m.data), 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Imu, "/imu", self.on_imu, 10)
        self.create_subscription(Twist, "/cmd_vel_base", self.on_out, 10)

    def on_odom(self, m):
        self.odom = m

    def on_imu(self, m):
        self.imu_w = m.angular_velocity.z
        self.imu_seen = True

    def on_out(self, m):
        self.out = (m.linear.x, m.angular.z)

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)


def main(args):
    rclpy.init()
    node = StepProfile(args)
    try:
        # MANUAL 확인
        end = time.monotonic() + 5.0
        while node.state is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f"[s5-real] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')} · state={node.state}")
        if node.state != "MANUAL":
            print("[s5-real] 거부: 제어 상태가 MANUAL이 아님 — Pi에서 ./run-cli mode manual 먼저")
            return 2

        # 첫 odom 대기
        end = time.monotonic() + 3.0
        while node.odom is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is None:
            print("[s5-real] 거부: /odom 없음 — base/드라이버 확인")
            return 2

        rows = []
        rate = args.rate
        n = int(round(TOTAL * rate))
        t0 = time.monotonic()
        print(f"[s5-real] 시작 · {TOTAL:.0f} s · {rate:.0f} Hz · 출력 {args.output}")
        for k in range(n):
            t = k / rate
            v, w = 0.0, 0.0
            for start, end_s, vk, wk in SEGMENTS:
                if t < end_s:
                    if vk == "speed1":
                        v = args.speed1
                    elif vk == "speed2":
                        v = args.speed2
                    elif isinstance(vk, (int, float)):
                        v = float(vk)
                    if wk == "turn":
                        w = args.turn
                    elif isinstance(wk, (int, float)):
                        w = float(wk)
                    break
            node.send(v, w)
            next_t = t0 + (k + 1) / rate
            while time.monotonic() < next_t:
                rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, next_t - time.monotonic())))

            o = node.odom
            p = o.pose.pose
            ve = o.twist.twist.linear.x
            we = o.twist.twist.angular.z
            rows.append([round(t, 3), v, w, node.out[0], node.out[1],
                         p.position.x, p.position.y, yaw_of(p.orientation), ve, we, node.imu_w, "real"])

        for _ in range(3):
            node.send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.imu_seen:
            print("[s5-real] 경고: /imu 메시지 없음 — w_gyro 열이 0. 펌웨어 I 줄/드라이버 확인")

        with open(args.output, "w", newline="") as f:
            cw = csv.writer(f)
            cw.writerow(["t", "lin_raw", "ang_raw", "lin_out", "ang_out",
                         "odom_x", "odom_y", "odom_yaw", "v_enc", "w_enc", "w_gyro", "label"])
            cw.writerows(rows)
        print(f"[s5-real] 끝 · {len(rows)} rows → {args.output}")
        print(f"[s5-real] 비교: python3 isaac/steps/compare_step.py "
              f"isaac/steps/artifacts/sim_step.csv {args.output}")
        return 0
    except KeyboardInterrupt:
        for _ in range(3):
            node.send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        print("\n[s5-real] 중단 — 정지")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="S5: real 계단응답 기록")
    p.add_argument("--output", default="/tmp/real_step.csv")
    p.add_argument("--speed1", type=float, default=0.15)
    p.add_argument("--speed2", type=float, default=0.25)
    p.add_argument("--turn", type=float, default=1.0)
    p.add_argument("--rate", type=float, default=20.0)
    raise SystemExit(main(p.parse_args()))
