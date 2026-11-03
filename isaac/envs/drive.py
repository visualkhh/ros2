"""운전 창 — 로봇(USD) · 환경(env · env cfg) · 명령(commands)을 골라 띄운다. 기본 = 빈 바닥 + 키보드.

  ./run-world.sh [--robot NAME] [--env MOD:CLASS] [--env-cfg MOD:CLASS] [--cmd keyboard|square|MOD:CLASS]
행동이 [v, w] 명령인 env(BaseEnv 계열)면 된다. 키: W/A/S/D(keyboard) · R 원점 + 명령 처음부터 · F 카메라 · 창 닫으면 끝
"""
from __future__ import annotations

import argparse
import importlib
import time

from isaaclab.app import AppLauncher

CMDS = {"keyboard": "commands.keyboard:KeyboardCommand", "square": "commands.square:SquareCommand"}


def load(spec: str):
    """'pkg.mod:Class' 또는 'pkg.mod.Class' → 클래스."""
    mod, _, name = spec.partition(":") if ":" in spec else spec.rpartition(".")
    return getattr(importlib.import_module(mod), name)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="키보드 운전 (로봇 · 환경 선택)")
    p.add_argument("--robot", default=None, help="로봇 = robots/<이름> (config.py · spec.py · urdf/robot.urdf.xacro). 기본: env cfg의 로봇 (beebot)")
    p.add_argument("--env", default="envs.default.default_env:DefaultEnv", help="환경 클래스 MOD:CLASS")
    p.add_argument("--env-cfg", default="envs.default.default_env_cfg:DefaultEnvCfg", help="환경 설정 클래스 MOD:CLASS")
    p.add_argument("--cmd", default="keyboard", help="명령: keyboard(직접 운전) · square(네모 54 s, 노란 점 = 목표) · MOD:CLASS")
    p.add_argument("--speed", type=float, default=0.15, help="전진 [m/s]")
    p.add_argument("--turn", type=float, default=1.0, help="회전 [rad/s]")
    AppLauncher.add_app_launcher_args(p)
    args = p.parse_args(argv)
    import robots

    if args.robot and args.robot not in robots.names():   # sim 띄우기 전에 실패하게
        p.error(f"--robot 없음: {args.robot} (있는 것: {', '.join(robots.names())})")
    if "kit" not in (args.visualizer or []):
        args.visualizer = ["kit"]   # 키보드는 창이 있어야 한다
    app = AppLauncher(args).app
    try:
        import torch

        from commands import keyboard as kb

        cfg = load(args.env_cfg)()
        cfg.sim.device = args.device
        if args.robot:
            robots.use(cfg, args.robot)
        print(f"[drive] env {args.env} · cfg {args.env_cfg} · 로봇 {cfg.robot_cfg.spawn.usd_path}", flush=True)
        cmd = load(CMDS.get(args.cmd, args.cmd))(speed=args.speed, turn=args.turn)
        env = load(args.env)(cfg)
        env.reset()
        env.look_at((0.0, 0.0, 0.0))

        def start():
            for _ in range(10):   # 0.5 s 서게 한 뒤 그 자세를 명령 경로의 시작으로
                env.step(torch.zeros(1, 2, device=env.device))
            cmd.reset()
            env.set_path(cmd.path())
            return 0.0

        sim_t = start()
        print(f"[drive] 명령 {type(cmd).__name__} · W/A/S/D(keyboard) · R 원점 + 처음부터 · F 카메라를 로봇 쪽으로 · "
              "마우스 휠 줌 · 창 닫으면 끝", flush=True)
        prev, t_print, ended = set(), 0.0, False
        while app.is_running():
            t = time.monotonic()
            keys = kb.pressed()
            new, prev = keys - prev, keys
            if "R" in new:
                env.place((0.0, 0.0, 0.0))
                sim_t, ended = start(), False
            if "F" in new:   # 카메라는 평소 자유 (따라가게 고정하면 마우스 줌·회전이 매 프레임 덮어씌워짐)
                env.look_at(env.robot.data.root_pos_w.torch[0].tolist())
            v, w = (0.0, 0.0) if cmd.done(sim_t) else cmd(sim_t)
            if cmd.done(sim_t) and not ended:
                ended = True
                print(f"[drive] 명령 끝 ({cmd.total:.0f} s) — R = 다시", flush=True)
            env.step(torch.tensor([[v, w]], device=env.device))
            sim_t += env.policy_dt
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
