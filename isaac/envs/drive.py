"""키보드 운전 — 로봇(USD)과 환경(env · env cfg)을 골라 띄운다. 기본 = envs/default (아무것도 없는 평평한 바닥).

  ./run-world.sh [--usd-dir DIR|FILE] [--env MOD:CLASS] [--env-cfg MOD:CLASS]
행동이 [v, w] 명령인 env(BaseEnv 계열)면 된다. 키: W/A/S/D · Shift 느리게 · X 정지 · R 원점 · F 카메라 · 창 닫으면 끝
"""
from __future__ import annotations

import argparse
import glob
import importlib
import os
import time

from isaaclab.app import AppLauncher

ISAAC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(spec: str):
    """'pkg.mod:Class' 또는 'pkg.mod.Class' → 클래스."""
    mod, _, name = spec.partition(":") if ":" in spec else spec.rpartition(".")
    return getattr(importlib.import_module(mod), name)


def resolve_usd(path: str) -> str:
    """--usd-dir → USD 파일. urdf.xacro가 있는 urdf/<robot> 폴더면 변환, USD 폴더면 안의 .usd(a), 파일이면 그대로."""
    from urdf.robot_spec import ROBOTS_DIR, robot_usd

    p = os.path.abspath(path if os.path.isabs(path) else os.path.join(ISAAC, path))
    if os.path.isfile(p):
        return p
    if os.path.isfile(os.path.join(p, "urdf.xacro")):
        if os.path.dirname(p) != ROBOTS_DIR:
            raise SystemExit(f"--usd-dir: urdf.xacro 폴더는 {ROBOTS_DIR}/<robot> 이어야 함: {p}")
        return robot_usd(os.path.basename(p))
    found = sorted(glob.glob(os.path.join(p, "**", "*.usd*"), recursive=True))
    if not found:
        raise SystemExit(f"--usd-dir: USD 없음: {p}")
    return found[0]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="키보드 운전 (로봇 · 환경 선택)")
    p.add_argument("--usd-dir", default=None, help="로봇: urdf/<robot> 폴더 · USD 폴더 · .usd/.usda (기본: env cfg의 로봇)")
    p.add_argument("--env", default="envs.default.default_env:DefaultEnv", help="환경 클래스 MOD:CLASS")
    p.add_argument("--env-cfg", default="envs.default.default_env_cfg:DefaultEnvCfg", help="환경 설정 클래스 MOD:CLASS")
    p.add_argument("--speed", type=float, default=0.15, help="전진 [m/s]")
    p.add_argument("--turn", type=float, default=1.0, help="회전 [rad/s]")
    AppLauncher.add_app_launcher_args(p)
    args = p.parse_args(argv)
    if not os.path.exists(os.path.join(ISAAC, args.usd_dir) if args.usd_dir and not os.path.isabs(args.usd_dir) else args.usd_dir or ISAAC):
        p.error(f"--usd-dir 없음: {args.usd_dir}")   # sim 띄우기 전에 실패하게
    if "kit" not in (args.visualizer or []):
        args.visualizer = ["kit"]   # 키보드는 창이 있어야 한다
    app = AppLauncher(args).app
    try:
        import torch

        from envs import keyboard as kb

        cfg = load(args.env_cfg)()
        cfg.sim.device = args.device
        if args.usd_dir:
            usd = resolve_usd(args.usd_dir)
            cfg.robot_cfg = cfg.robot_cfg.replace(spawn=cfg.robot_cfg.spawn.replace(usd_path=usd))
        print(f"[drive] env {args.env} · cfg {args.env_cfg} · 로봇 {cfg.robot_cfg.spawn.usd_path}", flush=True)
        env = load(args.env)(cfg)
        env.reset()
        env.look_at((0.0, 0.0, 0.0))
        print("[drive] W/A/S/D(화살표) · Shift 느리게 · X 정지 · R 원점 · F 카메라를 로봇 쪽으로 · 마우스 휠 줌 · 창 닫으면 끝", flush=True)
        prev, t_print = set(), 0.0
        while app.is_running():
            t = time.monotonic()
            keys = kb.pressed()
            new, prev = keys - prev, keys
            if "R" in new:
                env.place((0.0, 0.0, 0.0))
            if "F" in new:   # 카메라는 평소 자유 (따라가게 고정하면 마우스 줌·회전이 매 프레임 덮어씌워짐)
                env.look_at(env.robot.data.root_pos_w.torch[0].tolist())
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
