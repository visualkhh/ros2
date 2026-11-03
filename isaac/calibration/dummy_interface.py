"""가짜 로봇 — Isaac/ROS 없이 calibration 흐름(기록 → fit.json → report)을 돌려볼 때. 표준 라이브러리만.

plant: 지연 → 좌/우 gain → 데드밴드 → 1차 지연 + 잡음 · 회전 슬립.
  ./run-calibration.sh dummy --gain-l 0.96 --gain-r 1.02 --out artifacts/calibration/dummy_real/raw.csv   # → fit.json에 같은 gain이 나오면 OK
"""
from __future__ import annotations

import argparse
import math
import random
from collections import deque

import robots
from calibration.robot_interface import RobotInterface
from calibration.sysid import add_command_args, make_command, record, run_path

CONTROL_HZ = 20.0   # 기록 주기 (= envs/base_env_cfg.py CONTROL_HZ — Isaac 없이 돌아서 따로, 바꾸면 같이)
robot_cfg = robots.load()
R, SEP, MAX_WHEEL, DT = robot_cfg.WHEEL_RADIUS, robot_cfg.WHEEL_SEP, robot_cfg.MAX_WHEEL, 1.0 / CONTROL_HZ


def _lag(state, target, tau):
    a = 1.0 - math.exp(-DT / tau) if tau > 1e-9 else 1.0
    return tuple(s + a * (x - s) for s, x in zip(state, target))


class DummyInterface(RobotInterface):
    """명령 → 가짜 모터 (드라이버 바퀴 식 → 지연 → gain → 데드밴드 → τ) → 엔코더 · 자이로."""

    tag = "dummy"

    def __init__(self, gain_lr=(0.99, 0.99), tau=0.05, delay=2, noise=0.002, slip=0.0, deadband=0.0,
                 seed=0, label="dummy"):
        super().__init__(DT)
        self.gain_lr, self.tau, self.noise, self.slip, self.deadband, self.label = gain_lr, tau, noise, slip, deadband, label
        self.rng = random.Random(seed)
        self.buf = deque([(0.0, 0.0)] * (delay + 1), maxlen=delay + 1)
        self.plant, self.odom = (0.0, 0.0), [0.0, 0.0, 0.0]

    def write(self, v, w):
        wl, wr = (v - w * SEP / 2.0) / R, (v + w * SEP / 2.0) / R
        k = min(1.0, MAX_WHEEL / max(abs(wl), abs(wr), 1e-9))
        self.buf.append((wl * k, wr * k))

    def read(self, t, v, w):
        (wl, wr), (gl, gr) = self.buf[0], self.gain_lr
        tl, tr = [0.0 if abs(x) < self.deadband else x for x in (wl * gl, wr * gr)]
        sl, sr = self.plant = _lag(self.plant, (tl, tr), self.tau)
        n = lambda: self.rng.uniform(-self.noise, self.noise)  # noqa: E731
        v_enc, w_enc = R * (sl + sr) / 2.0 + n(), R * (sr - sl) / SEP + n()
        w_gyro = R * (sr - sl) / SEP * (1.0 - self.slip) + n()
        o = self.odom
        o[2] += w_enc * DT
        o[0] += v_enc * math.cos(o[2]) * DT
        o[1] += v_enc * math.sin(o[2]) * DT
        return [round(t, 3), v, w, v, w, *o, v_enc, w_enc, w_gyro, sl, sr, 0.0, 0.0, self.label, t, t, t, t]


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
    robot = DummyInterface(gain_lr, args.tau, args.delay, args.noise, args.slip, args.deadband, args.seed, args.label)
    record(robot, make_command(args), args.out or run_path("dummy"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
