"""calibration dummy — DummyRobot(robots/dummy_robot.py)로 Isaac/ROS 없이 calibration 흐름(기록 → fit.json → report)을 돌려볼 때. 표준 라이브러리만.

plant: 지연 → 좌/우 gain → 데드밴드 → 1차 지연 + 잡음 · 회전 슬립.
  ./run-calibration.sh dummy --gain-l 0.96 --gain-r 1.02 --out artifacts/calibration/dummy_real/raw.csv   # → fit.json에 같은 gain이 나오면 OK
"""
from __future__ import annotations

import argparse

from calibration.record import add_command_args, make_command, record, run_path, task_dir
from robots.dummy_robot import DummyRobot


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="가짜 로봇으로 calibration 한 바퀴")
    add_command_args(p)
    p.add_argument("--gain", type=float, default=0.99, help="모터 gain (1.0 = 이상)")
    p.add_argument("--gain-l", type=float, default=None)
    p.add_argument("--gain-r", type=float, default=None)
    p.add_argument("--tau", type=float, default=0.05, help="모터 1차 지연 [s]")
    p.add_argument("--delay", type=int, default=2, help="명령 지연 [20 Hz 스텝]")
    p.add_argument("--noise", type=float, default=0.002)
    p.add_argument("--slip", type=float, default=0.0, help="회전 슬립 비율 (0.1 = 몸체 회전 10%% 손실)")
    p.add_argument("--deadband", type=float, default=0.0, help="[rad/s]")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--label", default="dummy")
    p.add_argument("--out", default=None)
    args = p.parse_args(argv)
    gain_lr = (args.gain if args.gain_l is None else args.gain_l, args.gain if args.gain_r is None else args.gain_r)
    robot = DummyRobot(gain_lr, args.tau, args.delay, args.noise, args.slip, args.deadband, args.seed, robot=task_dir(args))
    record(robot, make_command(args), args.out or run_path("dummy"), "dummy", args.label)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
