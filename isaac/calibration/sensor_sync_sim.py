"""sim 측정 — 사각형 명령을 calibration env(로봇 1대)에 넣고 v_enc · w_enc · w_gyro를 기록 → raw.csv · fit.json · report.png.

  ./run-calibration.sh sensor-sync-sim --viz none                                   # raw plant 측정
  ./run-calibration.sh sensor-sync-sim --viz none --fit <real>/fit.json,<sim>/fit.json   # real처럼 맞춘 sim
  ./run-calibration.sh sensor-sync-sim --watch --fit <real>,<sim> --policy artifacts/real/policy.onnx --label B   # sim play
창(--watch): 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 노란 점 = 목표 경로. 기록 후 창 닫을 때까지 같은 경로 반복.
출력: artifacts/calibration/<시각>-<label>/ (--label 없으면 02-sim) · --out으로 정확한 경로.
"""
from __future__ import annotations

import argparse
import math
import os
import time
from pathlib import Path

from isaaclab.app import AppLauncher

from commands.square import SquareCommand
from calibration.fit import describe, load_fit, wheel_of
from calibration.record import ISAAC, record, run_path
from urdf import robot_spec as spec

TAG = "02-sim"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="sim 측정 (calibration env)")
    p.add_argument("--speed", type=float, default=0.15, help="기준 전진 [m/s] (변마다 1.0/0.67/1.33/0.8배)")
    p.add_argument("--turn", type=float, default=1.0, help="회전 [rad/s]")
    p.add_argument("--side", type=float, default=0.5, help="사각형 한 변 [m]")
    p.add_argument("--room", type=float, default=0.0, help="방 크기 [m] (0 = 장애물 없는 빈 바닥, 측정은 0 권장)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=None, help="정확한 raw.csv 경로 (isaac/ 기준 상대경로 가능)")
    p.add_argument("--label", default=None, help="폴더 이름 <시각>-<label> · CSV label (기본 02-sim / sim_sensor_floor)")
    p.add_argument("--fit", default=None, help="입력 보정 real/fit.json 또는 real,sim (차이). 없으면 raw plant")
    p.add_argument("--policy", default=None, help="학습한 ONNX로 보정하며 달림 (ros2 corrector와 같은 규칙)")
    p.add_argument("--watch", action="store_true", help="기록 후 창 닫을 때까지 같은 경로 반복")
    AppLauncher.add_app_launcher_args(p)
    args = p.parse_args(argv)

    # sim 띄우기 전에 실패하게
    cmd = SquareCommand(args.speed, args.turn, args.side)
    base_fit = load_fit(args.fit) if args.fit else None
    sess = None
    if args.policy:
        import onnxruntime as ort
        path = Path(args.policy) if os.path.isabs(args.policy) else ISAAC / args.policy
        if not path.is_file():
            p.error(f"--policy 파일 없음: {path}")
        sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        if sess.get_inputs()[0].shape[1] != 6:
            p.error("--policy 입력이 6차원(track 정책)이 아님")
    out = args.out and (args.out if os.path.isabs(args.out) else str(ISAAC / args.out))
    out = out or run_path(args.label or TAG)

    app = AppLauncher(args).app
    try:
        _run(args, app, cmd, base_fit, sess, out)
    except BaseException:
        import traceback
        traceback.print_exc()   # app.close()가 프로세스를 바로 끝내 예외가 안 보임 → 먼저 출력
        raise
    finally:
        app.close()
    return 0


def _run(args, app, cmd, base_fit, sess, out):
    import numpy as np
    import torch

    from calibration.calib_env import CalibEnv
    from calibration.calib_env_cfg import CalibEnvCfg, WheelModelCfg

    headless = "kit" not in (getattr(args, "visualizer", None) or [])
    cfg = CalibEnvCfg()
    cfg.sim.device = args.device
    if base_fit is not None:
        cfg.wheel = WheelModelCfg(**wheel_of(base_fit))
    if args.room > 0:
        cfg.rooms, cfg.room_size, cfg.room_seed = 1, (args.room, args.room), args.seed
    cfg.debug_vis = not headless
    env = CalibEnv(cfg)
    env.reset()
    dt = env.policy_dt
    zero = torch.zeros(1, 2, device=env.device)
    for _ in range(10):   # 0.5 s — 스폰 높이에서 떨어져 바퀴·캐스터로 서게
        env.step(zero)
    p0 = env.robot.data.root_pos_w.torch[0]
    if not headless:
        env.set_path(cmd.path())   # 노란 점 = 사각형을 로봇 시작 자세로
    label = args.label or "sim_sensor_floor"
    mode = f"FIT {', '.join(base_fit['source'])}" if base_fit else "RAW (no --fit)"
    print(f"[{TAG}] 높이 {float(p0[2]):.3f} m · {label} · {mode}{' · POLICY ' + args.policy if args.policy else ''} · "
          f"{'room %.1f m' % args.room if args.room > 0 else 'empty floor'}")
    print(f"[{TAG}] wheel: {describe(cfg.wheel.to_dict())}")
    print(f"[{TAG}] square: side={args.side:.2f} m · {cmd.total:.1f}s · speeds {cmd.speeds_text()} m/s")

    st = {"t": 0.0, "changed": 0.0, "last": (0.0, 0.0), "out": (0.0, 0.0), "odom": [0.0, 0.0, 0.0]}

    def send(v, w):
        """ros2 corrector와 같은 규칙: obs 6개 → ONNX 보정량 더하고 clip · 정지 명령엔 보정 0."""
        if abs(v - st["last"][0]) > 1e-3 or abs(w - st["last"][1]) > 1e-3:
            st["changed"] = st["t"]
        st["last"] = (v, w)
        lo, ao = v, w
        if sess is not None and (v or w):
            s = env.sense()
            obs = np.array([[v, w, s["v_enc"], s["w_enc"], s["w_gyro"], min(st["t"] - st["changed"], 2.0)]], np.float32)
            dv, dw = sess.run(None, {sess.get_inputs()[0].name: obs})[0][0]
            lo = max(-spec.MAX_LIN, min(spec.MAX_LIN, v + float(dv)))
            ao = max(-spec.MAX_ANG, min(spec.MAX_ANG, w + float(dw)))
        st["out"] = (lo, ao)
        env.step(torch.tensor([[lo, ao]], device=env.device))
        st["t"] += dt

    def collect(t, v, w):
        s = env.sense()
        o = st["odom"]   # 드라이버 /odom처럼 엔코더 적분
        o[2] += s["w_enc"] * dt
        o[0] += s["v_enc"] * math.cos(o[2]) * dt
        o[1] += s["v_enc"] * math.sin(o[2]) * dt
        # sim은 단일 시계 — 메시지 시각 = 기록 시각
        return [round(t, 3), v, w, *st["out"], *o, s["v_enc"], s["w_enc"], s["w_gyro"], *s["jv"], *s["jp"], label,
                t, t, t, t]

    record(TAG, send, collect, cmd, out, dt, base_fit=base_fit)

    if args.watch and not headless:
        print(f"[{TAG}] WATCH — 창 닫을 때까지 반복")
        k = 0
        while app.is_running():
            send(*cmd((k * dt) % cmd.total))
            k += 1
            time.sleep(dt)
    env.close()


if __name__ == "__main__":
    raise SystemExit(main())
