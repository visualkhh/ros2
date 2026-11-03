"""real 측정 (PC에서) — sim과 같은 사각형 명령을 로봇에 보내고 /odom · /imu · /joint_states를 기록 → raw.csv · fit.json · report.png.

준비: Pi base + cmd_manager, 제어 MANUAL (cd ~/ros2/tool && ./run-cli base start && ./run-cli mode manual) · 경로 direct
      PC: ROS 환경 · ROS_DOMAIN_ID=25 (run-calibration.sh가 맞춤)
  ./run-calibration.sh sensor-sync-real --check-only     # 움직이지 않고 상태만
  ./run-calibration.sh sensor-sync-real --label real_floor
출력: artifacts/calibration/<시각>-<label>/ (--label 없으면 02-real)
"""
from __future__ import annotations

import argparse
import math
import os
import time

from calibration import profile
from calibration.record import record, run_path

TAG = "02-real"


def _sec(ts) -> float:
    if ts is None:
        return float("nan")
    if hasattr(ts, "nanoseconds"):   # rclpy Time
        return ts.nanoseconds / 1e9
    return float(ts.sec) + float(ts.nanosec) * 1e-9


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="real 측정")
    p.add_argument("--output", default=None, help="정확한 raw.csv 경로")
    p.add_argument("--label", default=None, help="폴더 이름 <시각>-<label> · CSV label (기본 02-real / real)")
    p.add_argument("--speed", type=float, default=0.15, help="기준 전진 [m/s] (sim과 같은 기본값)")
    p.add_argument("--turn", type=float, default=1.0)
    p.add_argument("--side", type=float, default=0.5, help="사각형 한 변 [m]")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--check-only", action="store_true", help="상태만 확인 (로봇 안 움직임)")
    args = p.parse_args(argv)
    segs = profile.make_profile(args.speed, args.turn, args.side)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("sensor_sync")
    s = {"state": None, "odom": None, "imu_w": 0.0, "imu_stamp": None, "joint_stamp": None,
         "wl": 0.0, "wr": 0.0, "pl": 0.0, "pr": 0.0, "out": (0.0, 0.0)}

    def on_joint(m):
        s["joint_stamp"] = m.header.stamp
        for name, pos, vel in zip(m.name, m.position, m.velocity):
            if name == "wheel_left_joint":
                s["pl"], s["wl"] = pos, vel
            elif name == "wheel_right_joint":
                s["pr"], s["wr"] = pos, vel

    def on_imu(m):
        s["imu_w"], s["imu_stamp"] = m.angular_velocity.z, m.header.stamp

    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/state_control", lambda m: s.update(state=m.data), latched)
    node.create_subscription(Odometry, "/odom", lambda m: s.update(odom=m), 10)
    node.create_subscription(Imu, "/imu", on_imu, 10)
    node.create_subscription(JointState, "/joint_states", on_joint, 10)
    node.create_subscription(Twist, "/cmd_vel_base", lambda m: s.update(out=(m.linear.x, m.angular.z)), 10)
    pub = node.create_publisher(Twist, "/cmd_manual_vel", 10)

    def send(v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        pub.publish(msg)

    def stop():
        for _ in range(3):
            send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

    def wait(key, sec):
        end = time.monotonic() + sec
        while s[key] is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    label = args.label or "real"

    def collect(t, v, w):
        o = s["odom"]
        pp, q = o.pose.pose.position, o.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return [round(t, 3), v, w, *s["out"], pp.x, pp.y, yaw, o.twist.twist.linear.x, o.twist.twist.angular.z,
                s["imu_w"], s["wl"], s["wr"], s["pl"], s["pr"], label,
                _sec(node.get_clock().now()), _sec(o.header.stamp), _sec(s["imu_stamp"]), _sec(s["joint_stamp"])]

    try:
        wait("state", 5.0)
        print(f"[{TAG}] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')} · state={s['state']}")
        if args.check_only:
            return 0
        if s["state"] != "MANUAL":
            print(f"[{TAG}] 거부: MANUAL 아님 — Pi에서 ./run-cli mode manual 먼저")
            return 2
        wait("odom", 3.0)
        if s["odom"] is None:
            print(f"[{TAG}] 거부: /odom 없음 — base/드라이버 확인")
            return 2
        out = args.output or run_path(args.label or TAG)
        total, t0, rate = profile.total(segs), time.monotonic(), args.rate
        print(f"[{TAG}] 시작 · {total:.0f}s · {rate:.0f}Hz · side={args.side:.2f}m · speeds {profile.speeds_text(segs)} m/s → {out}")

        def pace(k, _t):
            nxt = t0 + (k + 1) / rate
            while time.monotonic() < nxt:
                rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, nxt - time.monotonic())))

        record(TAG, send, collect, segs, total, out, 1.0 / rate, pace=pace)
        stop()
        if s["imu_stamp"] is None:
            print(f"[{TAG}] 경고: /imu 없음 — w_gyro가 0")
        if s["joint_stamp"] is None:
            print(f"[{TAG}] 경고: /joint_states 없음 — 바퀴 raw가 0")
        return 0
    except KeyboardInterrupt:
        stop()
        print(f"\n[{TAG}] 중단 — 정지")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
