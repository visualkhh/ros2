"""sysid real (PC에서) — RealRobot(<기본 로봇 폴더>/real_robot.py)에 sim과 같은 명령을 로봇에 보내고 /odom · /imu · /joint_states를 기록 → raw.csv · fit.json · report.png.

준비: Pi base + cmd_manager, 제어 MANUAL (cd ~/ros2/tool && ./run-cli base start && ./run-cli mode manual) · 경로 direct
      PC: ROS 환경 · ROS_DOMAIN_ID=25 (run-calibration.sh가 맞춤)
  ./run-calibration.sh sensor-sync-real --check-only     # 움직이지 않고 상태만
  ./run-calibration.sh sensor-sync-real --label real_floor
출력: artifacts/calibration/<시각>-<label>/ (--label 없으면 02-real)
"""
from __future__ import annotations

import argparse
import os

import robots
from calibration.sysid import add_command_args, make_command, record, run_path

TAG = "02-real"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="real 측정")
    add_command_args(p)
    p.add_argument("--output", default=None, help="정확한 raw.csv 경로")
    p.add_argument("--label", default=None, help="폴더 이름 <시각>-<label> · CSV label (기본 02-real / real)")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--check-only", action="store_true", help="상태만 확인 (로봇 안 움직임)")
    args = p.parse_args(argv)
    cmd = make_command(args)

    import rclpy

    rclpy.init()
    robot = robots.load(robots.DEFAULT, "real_robot").RealRobot(args.rate)   # 기본 로봇 폴더의 real_robot.py
    try:
        robot.wait("state", 5.0)
        print(f"[{TAG}] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')} · state={robot.s['state']}")
        if args.check_only:
            return 0
        if robot.s["state"] != "MANUAL":
            print(f"[{TAG}] 거부: MANUAL 아님 — Pi에서 ./run-cli mode manual 먼저")
            return 2
        robot.wait("odom", 3.0)
        if robot.s["odom"] is None:
            print(f"[{TAG}] 거부: /odom 없음 — base/드라이버 확인")
            return 2
        out = args.output or run_path(args.label or TAG)
        speeds = getattr(cmd, "speeds_text", lambda: "-")()
        print(f"[{TAG}] 시작 · {cmd.total:.0f}s · {args.rate:.0f}Hz · {args.cmd} side={args.side:.2f}m · speeds {speeds} m/s → {out}")
        record(robot, cmd, out, TAG, args.label or "real")
        robot.stop()
        if robot.s["imu_stamp"] is None:
            print(f"[{TAG}] 경고: /imu 없음 — w_gyro가 0")
        if robot.s["joint_stamp"] is None:
            print(f"[{TAG}] 경고: /joint_states 없음 — 바퀴 raw가 0")
        return 0
    except KeyboardInterrupt:
        robot.stop()
        print(f"\n[{TAG}] 중단 — 정지")
        return 130
    finally:
        robot.close()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
