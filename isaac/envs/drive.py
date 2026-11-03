"""기본 환경(envs/default: DefaultEnv) 키보드 운전 — 로봇 1대 · 평평한 바닥 · 아무것도 없음. 카메라가 로봇을 따라간다.

  ./run-world.sh empty [--speed 0.15] [--turn 1.0]
키: W/A/S/D(화살표) 주행 · Shift 느리게 · X 정지 · R 원점으로 · 창 닫으면 끝
터미널: 움직이는 동안 0.5 s마다 명령 / 엔코더 v·w / 자이로 w
"""
from __future__ import annotations

import argparse
import time

from isaaclab.app import AppLauncher


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="기본 세상 키보드 운전")
    p.add_argument("--speed", type=float, default=0.15, help="전진 [m/s]")
    p.add_argument("--turn", type=float, default=1.0, help="회전 [rad/s]")
    AppLauncher.add_app_launcher_args(p)
    args = p.parse_args(argv)
    if "kit" not in (args.visualizer or []):
        args.visualizer = ["kit"]   # 키보드는 창이 있어야 한다
    app = AppLauncher(args).app
    try:
        import torch

        from envs import keyboard as kb
        from envs.default.default_env import DefaultEnv
        from envs.default.default_env_cfg import DefaultEnvCfg

        cfg = DefaultEnvCfg()
        cfg.sim.device = args.device
        env = DefaultEnv(cfg)
        env.reset()
        print("[drive] W/A/S/D(화살표) · Shift 느리게 · X 정지 · R 원점 · 창 닫으면 끝", flush=True)
        prev, t_print = set(), 0.0
        while app.is_running():
            t = time.monotonic()
            keys = kb.pressed()
            new, prev = keys - prev, keys
            if "R" in new:
                env.place((0.0, 0.0, 0.0))
            v, w = kb.drive_cmd(keys, args.speed, args.turn)
            env.step(torch.tensor([[v, w]], device=env.device))
            if (v or w) and t - t_print > 0.5:
                t_print = t
                s = env.sense()
                print(f"[drive] 명령 v {v:+.2f} w {w:+.2f} | 엔코더 v {s['v_enc']:+.3f} w {s['w_enc']:+.3f} | "
                      f"자이로 w {s['w_gyro']:+.3f}", flush=True)
            time.sleep(max(0.0, env.policy_dt - (time.monotonic() - t)))   # 실시간
        env.close()
    except BaseException:
        import traceback
        traceback.print_exc()   # app.close()가 프로세스를 바로 끝내 예외가 안 보임 → 먼저 출력
        raise
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
