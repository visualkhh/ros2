"""운전 창 — task 하나를 골라 띄운다 (./run-world.sh <task> [--calibration] [--command=NAME]).

  <task>                  training/<task>/ — env.py + command.py (task env: 기본 제어기가 랜덤 웨이포인트를 따라감, 보정 0)
  --calibration           training/<task>/calibration/ — CalibrationEnv + CalibrationCommand (네모)
  --command=NAME          명령만 바꿈: keyboard (commands/keyboard.py 직접 운전) · square · MOD:CLASS (BaseCommand)
                          [v, w] 명령이라 task env(보정 행동)가 아니라 그 task의 CalibrationEnv(빈 바닥)에서
로봇 = task 폴더 (robot_cfg.py · usda/robot.usda). 키: W/A/S/D(keyboard) · R 처음부터 · F 카메라를 로봇 쪽으로 · 창 닫으면 끝
"""
from __future__ import annotations

import argparse
import importlib
import os
import time

from isaaclab.app import AppLauncher

from commands import CMDS, command_class


def load(spec: str):
    """'pkg.mod:Class' → 클래스."""
    mod, _, name = spec.partition(":")
    return getattr(importlib.import_module(mod), name)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="run-world.sh", description="task 하나를 창으로 (task env · calibration · 직접 운전)")
    p.add_argument("task", help="training/ 아래 task 폴더 이름 (예: rl_correction_waypoint)")
    p.add_argument("--calibration", action="store_true", help="task의 calibration/ (CalibrationEnv + CalibrationCommand 네모)")
    p.add_argument("--command", default=None, help=f"명령 바꾸기: {' · '.join(CMDS)} · MOD:CLASS (BaseCommand) — CalibrationEnv에서")
    p.add_argument("--speed", type=float, default=0.15, help="전진 [m/s] (keyboard: 키 하나 · square: 기준 속도)")
    p.add_argument("--turn", type=float, default=1.0, help="회전 [rad/s]")
    AppLauncher.add_app_launcher_args(p)
    args = p.parse_args(argv)

    from training import task_dir

    tdir = task_dir(args.task)   # sim 띄우기 전에 실패하게
    name = os.path.basename(tdir)
    camel = "".join(w.capitalize() for w in name.split("_"))
    vw = args.calibration or args.command is not None   # [v, w] 명령으로 직접 몬다
    if vw:
        env_spec, cfg_spec = f"training.{name}.calibration.env:CalibrationEnv", f"training.{name}.calibration.env_cfg:CalibrationEnvCfg"
        cmd_spec = args.command or f"training.{name}.calibration.command:CalibrationCommand"
    else:
        env_spec, cfg_spec, cmd_spec = f"training.{name}.env:{camel}Env", f"training.{name}.env_cfg:{camel}EnvCfg", None
    shown = bool(args.visualizer) and args.visualizer != ["none"]   # 기본 headless — 창은 --viz
    if args.command == "keyboard" and "kit" not in (args.visualizer or []):
        raise SystemExit("--command=keyboard는 창이 있어야 함 — --viz 를 붙이세요")
    app = AppLauncher(args).app
    running = app.is_running if shown else (lambda: True)   # 창 없으면 Ctrl+C까지 (headless는 is_running이 False)
    try:
        cfg = load(cfg_spec)()
        cfg.robot = tdir   # 로봇 = task 폴더
        cfg.scene.num_envs = 1
        cfg.sim.device = args.device
        cfg.debug_vis = shown
        env = load(env_spec)(cfg)
        print(f"[drive] {args.task} · env {env_spec} · 명령 {cmd_spec or f'training.{name}.command (랜덤 웨이포인트)'} · "
              f"로봇 {cfg.robot_cfg.spawn.usd_path}", flush=True)
        if vw:
            _drive(running, env, command_class(cmd_spec)(speed=args.speed, turn=args.turn))
        else:
            _task(running, env)
        env.close()
    except BaseException:
        import traceback
        traceback.print_exc()   # app.close()가 프로세스를 바로 끝내 예외가 안 보임 → 먼저 출력
        raise
    finally:
        app.close()
    return 0


def _keys(env, prev):
    """눌린 키 중 새로 눌린 것 + F(카메라) 처리. 창이 없으면 키도 없다."""
    if not env.cfg.debug_vis:
        return set(), prev
    from commands import keyboard as kb

    keys = kb.pressed()
    new = keys - prev
    if "F" in new:   # 카메라는 평소 자유 (따라가게 고정하면 마우스 줌·회전이 매 프레임 덮어씌워짐)
        env.look_at(env.robot.data.root_pos_w.torch[0].tolist())
    return new, keys


def _task(running, env) -> None:
    """task env 그대로 — 행동 0 (정책 보정 없음) = 기본 제어기만으로 웨이포인트. 에피소드가 끝나면 env가 새 웨이포인트로 리셋."""
    import torch

    env.reset()
    env.look_at((0.0, 0.0, 0.0))
    zero = torch.zeros(env.num_envs, env.cfg.action_space, device=env.device)
    print("[drive] 노란 점 = 웨이포인트 경로 · 보정 0 · R 새 에피소드 · F 카메라 · 창 닫으면 끝", flush=True)
    prev = set()
    while running():
        t = time.monotonic()
        new, prev = _keys(env, prev)
        if "R" in new:
            env.reset()
        env.step(zero)
        time.sleep(max(0.0, env.step_dt - (time.monotonic() - t)))   # 실시간


def _drive(running, env, cmd) -> None:
    """[v, w] 명령(BaseCommand)으로 직접 — keyboard · CalibrationCommand(네모) 등."""
    import torch

    def start():
        env.start(cmd.path())   # 0.5 s 서게 한 뒤 그 자세를 명령 경로의 시작으로
        cmd.reset()
        return 0.0

    env.look_at((0.0, 0.0, 0.0))
    sim_t = start()
    print(f"[drive] 명령 {type(cmd).__name__} · W/A/S/D(keyboard) · R 원점 + 처음부터 · F 카메라 · 마우스 휠 줌 · 창 닫으면 끝",
          flush=True)
    prev, t_print, ended = set(), 0.0, False
    while running():
        t = time.monotonic()
        new, prev = _keys(env, prev)
        if "R" in new:
            env.place((0.0, 0.0, 0.0))
            sim_t, ended = start(), False
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


if __name__ == "__main__":
    raise SystemExit(main())
