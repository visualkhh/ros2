"""운전 창 — task 하나를 골라 띄운다 (./run-world.sh <task> [--calibration] [--command NAME] [--onnx PATH] [--viz]).

  <task>                  training/<task>/ — env.py + command.py (task env: 기본 제어기가 랜덤 웨이포인트를 따라감, 보정 0)
  --calibration           training/<task>/calibration/ — CalibrationEnv + CalibrationCommand (네모)
  --onnx PATH             기본 모드에서 정책 보정 (exported/policy.onnx) · P = 켬/끔 (정책 vs 기본 제어기만)
  --command NAME          명령만 바꿈: keyboard (commands/keyboard.py 직접 운전) · square · MOD:CLASS (BaseCommand)
                          [v, w] 명령이라 task env(보정 행동)가 아니라 CalibrationEnv로 몰고,
                          task env가 direct_drive면(rl_maze) 그 env 자체에서 [v,w]로 몬다 (미로 배경).
                          (--calibration과 같이 쓰면 측정 그대로 = 빈 평면)
로봇 = task 폴더 (robot_cfg.py · usda/robot.usda). 키: W/A/S/D(keyboard) · R 처음부터 · F 카메라를 로봇 쪽으로 · L 라이다 점군 · 창 닫으면 끝
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
    p.add_argument("--onnx", default=None, help="학습한 정책 ONNX (exported/policy.onnx, 옆에 .onnx.data) — task 기본 모드에서만 (웨이포인트 보정). P = 켬/끔")
    p.add_argument("--speed", type=float, default=0.15, help="전진 [m/s] (keyboard: 키 하나 · square: 기준 속도)")
    p.add_argument("--turn", type=float, default=1.0, help="회전 [rad/s]")
    AppLauncher.add_app_launcher_args(p)
    args = p.parse_args(argv)

    from training import task_dir

    tdir = task_dir(args.task)   # sim 띄우기 전에 실패하게
    if args.onnx and (args.calibration or args.command is not None):
        raise SystemExit("--onnx는 task 기본 모드(학습 command = 웨이포인트)에서만 — 정책이 웨이포인트 목표를 보고 보정한다")
    if args.onnx and not os.path.isfile(args.onnx):
        raise SystemExit(f"정책 없음: {args.onnx}")
    name = os.path.basename(tdir)
    camel = "".join(w.capitalize() for w in name.split("_"))
    shown = bool(args.visualizer) and args.visualizer != ["none"]   # 기본 headless — 창은 --viz
    if args.command == "keyboard" and "kit" not in (args.visualizer or []):
        raise SystemExit("--command keyboard는 창이 있어야 함 — --viz 를 붙이세요")
    app = AppLauncher(args).app
    running = app.is_running if shown else (lambda: True)   # 창 없으면 Ctrl+C까지 (headless는 is_running이 False)
    try:
        task_env = f"training.{name}.env:{camel}Env"
        task_cfg = f"training.{name}.env_cfg:{camel}EnvCfg"

        # --command는 보통 CalibrationEnv에서 [v,w]로 모는 방식이다.
        #   task env가 direct_drive면(rl_maze) 그 env 자체에서 직접 [v,w]로 몬다 — 미로 배경 그대로.
        direct_task = False
        if args.command is not None and not args.calibration:
            direct_task = bool(getattr(load(task_env), "direct_drive", False))
        task_keyboard = args.command == "keyboard" and not args.calibration and direct_task
        vw = args.calibration or (args.command is not None and not task_keyboard)   # [v, w] 명령으로 직접 몬다
        if vw:
            env_spec, cfg_spec = f"training.{name}.calibration.env:CalibrationEnv", f"training.{name}.calibration.env_cfg:CalibrationEnvCfg"
            cmd_spec = args.command or f"training.{name}.calibration.command:CalibrationCommand"
        else:
            env_spec, cfg_spec, cmd_spec = task_env, task_cfg, None

        ground = None
        if args.command is not None and not args.calibration and not direct_task:
            try:
                ground = importlib.import_module(f"training.{name}.ground")
            except ModuleNotFoundError as exc:
                if exc.name != f"training.{name}.ground":
                    raise

        cfg = load(cfg_spec)()
        cfg.robot = tdir   # 로봇 = task 폴더
        cfg.scene.num_envs = 1
        cfg.sim.device = args.device
        cfg.debug_vis = shown
        env_cls = load(env_spec)
        if ground is not None:
            env_cls = _on_training_ground(env_cls, ground)   # 보정 task: 학습 바닥 위의 CalibrationEnv
        env = env_cls(cfg)
        print(f"[drive] {args.task} · env {env_spec} · 명령 {cmd_spec or f'training.{name}.command (랜덤 웨이포인트)'} · "
              f"로봇 {cfg.robot_cfg.spawn.usd_path}", flush=True)
        if vw:
            _drive(running, env, command_class(cmd_spec)(speed=args.speed, turn=args.turn))
        elif task_keyboard:
            _task_keyboard(running, env, args.speed, args.turn)
        else:
            _task(running, env, args.onnx)
        env.close()
    except BaseException:
        import traceback
        traceback.print_exc()   # app.close()가 프로세스를 바로 끝내 예외가 안 보임 → 먼저 출력
        raise
    finally:
        app.close()
    return 0


def _on_training_ground(env_cls, ground):
    """env_cls의 배경만 task의 학습 배경(ground.py)으로 바꾼 클래스 — 측정 env 코드는 건드리지 않는다."""
    g = ground.GroundCfg()

    class OnTrainingGround(env_cls):
        def ground_cfg(self):
            return ground.make_terrain(g)

        def ground_objects(self) -> None:
            ground.spawn_friction_patches(self, g)

    return OnTrainingGround


def _keys(env, prev):
    """눌린 키 중 새로 눌린 것 + F(카메라) · L(라이다 점군) 처리. 창이 없으면 키도 없다."""
    if not env.cfg.debug_vis:
        return set(), prev
    from commands import keyboard as kb

    keys = kb.pressed()
    new = keys - prev
    if "F" in new:   # 카메라는 평소 자유 (따라가게 고정하면 마우스 줌·회전이 매 프레임 덮어씌워짐)
        env.look_at(env.robot.data.root_pos_w.torch[0].tolist())
    if "L" in new:
        toggle = getattr(env, "toggle_lidar_vis", None)
        if toggle is not None:
            toggle()
    return new, keys


def _task(running, env, policy_path: str | None = None) -> None:
    """task env 그대로 — 기본 제어기가 웨이포인트로 몰고, 행동(보정) = 정책(ONNX) 또는 0. 에피소드가 끝나면 env가 새 웨이포인트로 리셋.
    direct_drive env(rl_maze)는 보정이 아니라 [v,w] 그 자체라, ONNX가 없으면 간단한 웨이포인트 추종으로 몬다."""
    import torch

    sess = None
    if policy_path:
        import onnxruntime as ort

        sess = ort.InferenceSession(policy_path)   # 옆의 policy.onnx.data(가중치)도 같이 읽는다
        in_name = sess.get_inputs()[0].name
        print(f"[drive] 정책 {policy_path} · 입력 {sess.get_inputs()[0].shape} → 출력 {sess.get_outputs()[0].shape}", flush=True)
    direct = bool(getattr(env, "direct_drive", False))
    obs, _ = env.reset()
    env.look_at(env.robot.data.root_pos_w.torch[0].tolist() if direct else (0.0, 0.0, 0.0))
    zero = torch.zeros(env.num_envs, env.cfg.action_space, device=env.device)
    on = sess is not None
    kind = "정책" if direct else "보정"
    print(f"[drive] 노란 점 = 웨이포인트 경로 · {kind} {'정책' if on else ('웨이포인트 추종' if direct else '0')} · "
          f"{'P 정책 켬/끔 · ' if sess is not None else ''}R 새 에피소드 · F 카메라 · 창 닫으면 끝", flush=True)
    prev = set()
    while running():
        t = time.monotonic()
        new, prev = _keys(env, prev)
        if "R" in new:
            obs, _ = env.reset()
            if direct:
                env.look_at(env.robot.data.root_pos_w.torch[0].tolist())
        if "P" in new and sess is not None:
            on = not on
            print(f"[drive] {kind} {'정책' if on else '웨이포인트 추종' if direct else '0 (기본 제어기만)'}", flush=True)
        if on:
            act = sess.run(None, {in_name: obs["policy"].cpu().numpy()})[0]
            action = torch.as_tensor(act, device=env.device)
        elif direct:   # 학습 정책 대신 간단한 P 제어 (화면 확인용)
            xy, yaw = env.pose()
            e = env.goal_point() - xy
            dist = torch.linalg.norm(e, dim=-1)
            herr = torch.atan2(e[:, 1], e[:, 0]) - yaw
            herr = torch.atan2(torch.sin(herr), torch.cos(herr))
            v = torch.clamp(0.8 * dist, max=env.cfg.max_lin) * (herr.abs() < 1.0)
            w = torch.clamp(1.5 * herr, -env.cfg.max_ang, env.cfg.max_ang)
            action = torch.stack([v / env.cfg.max_lin, w / env.cfg.max_ang], dim=-1)
        else:
            action = zero
        obs, _, _, _, _ = env.step(action)
        time.sleep(max(0.0, env.step_dt - (time.monotonic() - t)))   # 실시간


def _task_keyboard(running, env, speed: float, turn: float) -> None:
    """task env 자체(예: rl_maze 미로)에서 키보드로 직접 운전. 행동은 정책과 같은 정규화 [-1,1]."""
    import torch

    from commands import keyboard as kb

    c = env.cfg
    env.reset()
    env.look_at(env.robot.data.root_pos_w.torch[0].tolist())   # 시작부터 로봇이 보이게
    speed = max(0.05, min(float(speed), c.max_lin))
    prev: set = set()
    t_print = 0.0
    print(f'[drive] 학습 배경에서 직접 운전 · {type(env).__name__} · W/A/S/D · [ ] 속도 {speed:.2f} m/s · '
          f'R 새 에피소드 · F 카메라 · L 라이다 점군 · 창 닫으면 끝', flush=True)
    while running():
        t = time.monotonic()
        new, keys = _keys(env, prev)
        prev = keys
        if "R" in new:
            env.reset()
        if "RIGHT_BRACKET" in new:
            speed = min(c.max_lin, speed + 0.02)
            print(f'[drive] 속도 {speed:.2f} / {c.max_lin:.2f} m/s', flush=True)
        if "LEFT_BRACKET" in new:
            speed = max(0.05, speed - 0.02)
            print(f'[drive] 속도 {speed:.2f} / {c.max_lin:.2f} m/s', flush=True)
        v, w = kb.drive_cmd(keys, speed, turn)
        v_max = c.max_lin if v >= 0 else abs(c.v_min_ratio) * c.max_lin   # 후진은 정책과 같은 한계
        v_eff = v if v >= 0 else max(v, c.v_min_ratio * c.max_lin)
        action = torch.tensor([[v / v_max, w / c.max_ang]], device=env.device)
        env.step(action)
        if (v or w) and t - t_print > 0.5:
            t_print = t
            s = env.sense()
            print(f'[drive] 명령 v {v_eff:+.2f} w {w:+.2f} | 엔코더 v {s["v_enc"]:+.3f} w {s["w_enc"]:+.3f} | '
                  f'자이로 w {s["w_gyro"]:+.3f}', flush=True)
            _, jv, fric = env.wheel_state()
            f, j = fric[0].tolist(), jv[0].tolist()
            print(f'[drive] wheel fric L {f[0]:.2f} R {f[1]:.2f} | jv L {j[0]:+.2f} R {j[1]:+.2f} rad/s',
                  flush=True)
        time.sleep(max(0.0, env.policy_dt - (time.monotonic() - t)))


def _drive(running, env, cmd) -> None:
    """[v, w] 명령(BaseCommand)으로 직접 — keyboard · CalibrationCommand(네모) 등.
    task env가 direct_drive면([v,w] 정규화 행동) 그 env를, 아니면 CalibrationEnv(실제 단위)를 쓴다."""
    import torch

    direct = bool(getattr(env, "direct_drive", False))

    def action_of(v, w):
        if not direct:
            return torch.tensor([[v, w]], device=env.device)
        v_max = env.cfg.max_lin if v >= 0 else abs(env.cfg.v_min_ratio) * env.cfg.max_lin
        return torch.tensor([[v / v_max, w / env.cfg.max_ang]], device=env.device)

    def start():
        if direct:
            env.reset()
        else:
            env.start(cmd.path())   # 0.5 s 서게 한 뒤 그 자세를 명령 경로의 시작으로
        cmd.reset()
        return 0.0

    env.look_at((0.0, 0.0, 0.0))
    sim_t = start()
    if direct:
        env.look_at(env.robot.data.root_pos_w.torch[0].tolist())
    print(f"[drive] 명령 {type(cmd).__name__} · W/A/S/D(keyboard) · R 처음부터 · F 카메라 · L 라이다 점군 · "
          f"마우스 휠 줌 · 창 닫으면 끝", flush=True)
    prev, t_print, ended = set(), 0.0, False
    while running():
        t = time.monotonic()
        new, prev = _keys(env, prev)
        if "R" in new:
            if not direct:
                env.place((0.0, 0.0, 0.0))
            sim_t, ended = start(), False
        v, w = (0.0, 0.0) if cmd.done(sim_t) else cmd(sim_t)
        if cmd.done(sim_t) and not ended:
            ended = True
            print(f"[drive] 명령 끝 ({cmd.total:.0f} s) — R = 다시", flush=True)
        env.step(action_of(v, w))
        sim_t += env.policy_dt
        if (v or w) and t - t_print > 0.5:
            t_print = t
            s = env.sense()
            print(f"[drive] 명령 v {v:+.2f} w {w:+.2f} | 엔코더 v {s['v_enc']:+.3f} w {s['w_enc']:+.3f} | "
                  f"자이로 w {s['w_gyro']:+.3f}", flush=True)
        time.sleep(max(0.0, env.policy_dt - (time.monotonic() - t)))   # 실시간


if __name__ == "__main__":
    raise SystemExit(main())
