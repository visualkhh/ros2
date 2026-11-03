"""가짜 로봇 — Isaac/ROS 없이 calibration 흐름(기록 → fit.json → report)을 돌려볼 때. 표준 라이브러리만.

plant: 지연 → 좌/우 gain → 데드밴드 → 1차 지연 + 잡음 · 회전 슬립. 입력 보정(--fit)은 sim 바퀴 모델과 같은 순서.
  ./run-calibration.sh dummy --gain-l 0.96 --gain-r 1.02 --out artifacts/calibration/dummy_real/raw.csv
  ./run-calibration.sh dummy --fit artifacts/calibration/dummy_real/fit.json,<sim>/fit.json
"""
from __future__ import annotations

import argparse
import math
import random
from collections import deque

from calibration import profile
from calibration.fit import load_fit, wheel_of
from calibration.record import record, run_path
from urdf import robot_spec as spec

R, SEP, MAX_WHEEL, DT = spec.WHEEL_RADIUS, spec.WHEEL_SEP, spec.MAX_WHEEL, 1.0 / spec.CONTROL_HZ


def _lag(state, target, tau):
    a = 1.0 - math.exp(-DT / tau) if tau > 1e-9 else 1.0
    return tuple(s + a * (x - s) for s, x in zip(state, target))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="가짜 로봇으로 calibration 한 바퀴")
    p.add_argument("--side", type=float, default=0.5)
    p.add_argument("--speed", type=float, default=0.15)
    p.add_argument("--turn", type=float, default=1.0)
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
    p.add_argument("--fit", default=None, help="입력 보정 real/fit.json 또는 real,sim (차이)")
    args = p.parse_args(argv)
    base_fit = load_fit(args.fit) if args.fit else None
    b = wheel_of(base_fit) if base_fit else {"tau_s": 0.0, "delay_steps": 0, "gain_l": 1.0, "gain_r": 1.0, "deadband": 0.0}
    gl = args.gain if args.gain_l is None else args.gain_l
    gr = args.gain if args.gain_r is None else args.gain_r
    rng = random.Random(args.seed)
    base_buf = deque([(0.0, 0.0)] * (b["delay_steps"] + 1), maxlen=b["delay_steps"] + 1)
    buf = deque([(0.0, 0.0)] * (args.delay + 1), maxlen=args.delay + 1)
    st = {"base": (0.0, 0.0), "plant": (0.0, 0.0), "odom": [0.0, 0.0, 0.0]}

    def send(v, w):
        wl, wr = (v - w * SEP / 2.0) / R, (v + w * SEP / 2.0) / R
        k = min(1.0, MAX_WHEEL / max(abs(wl), abs(wr), 1e-9))
        base_buf.append((wl * k, wr * k))   # 입력 보정 — sim 바퀴 모델과 같은 순서
        wl, wr = base_buf[0]
        wl, wr = [0.0 if abs(x) < b["deadband"] else x for x in (wl * b["gain_l"], wr * b["gain_r"])]
        st["base"] = _lag(st["base"], (wl, wr), b["tau_s"])
        buf.append(st["base"])

    def collect(t, v, w):
        wl, wr = buf[0]
        tl, tr = [0.0 if abs(x) < args.deadband else x for x in (wl * gl, wr * gr)]
        sl, sr = st["plant"] = _lag(st["plant"], (tl, tr), args.tau)
        n = lambda: rng.uniform(-args.noise, args.noise)  # noqa: E731
        v_enc, w_enc = R * (sl + sr) / 2.0 + n(), R * (sr - sl) / SEP + n()
        w_gyro = R * (sr - sl) / SEP * (1.0 - args.slip) + n()
        o = st["odom"]
        o[2] += w_enc * DT
        o[0] += v_enc * math.cos(o[2]) * DT
        o[1] += v_enc * math.sin(o[2]) * DT
        return [round(t, 3), v, w, v, w, *o, v_enc, w_enc, w_gyro, sl, sr, 0.0, 0.0, args.label, t, t, t, t]

    segs = profile.make_profile(args.speed, args.turn, args.side)
    record("dummy", send, collect, segs, profile.total(segs), args.out or run_path("dummy"), DT, base_fit=base_fit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
