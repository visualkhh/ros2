"""calibration 실행기 (공식 train.py 역할) — ./run-calibration.sh <task> --robot=sim|real|dummy [옵션...].

training/<task>/calibration/ 명령 · env와 그 task 폴더의 로봇(sim_robot.py · real_robot.py)으로 한 번 측정 → raw.csv · fit.json · report.png.
  sim    SimRobot (task 폴더 sim_robot.py) + CalibrationEnv — Isaac venv · 창 기본 끔, --viz로 켬 · --watch (기록 후 창 닫을 때까지 반복)
  real   RealRobot (task 폴더 real_robot.py) — ROS .venv · 준비: Pi base + cmd_manager MANUAL · --check-only (안 움직임)
  dummy  DummyRobot (task 폴더 dummy_robot.py) — 표준 라이브러리만 (가짜 모터: --gain-l --gain-r --tau --delay --deadband --slip --noise)
출력: artifacts/<task>/calibration/<시각>_<label>/ (--label 없으면 02-sim · 02-real · dummy) · --out으로 정확한 경로.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from calibration.record import ISAAC, add_command_args, make_command, record, run_path, task_class, task_dir

MODES = ("sim", "real", "dummy")
TAGS = {"sim": "02-sim", "real": "02-real", "dummy": "dummy"}
LABELS = {"sim": "sim_sensor_floor", "real": "real", "dummy": "dummy"}   # CSV label 기본


def _args(mode: str, argv):
    p = argparse.ArgumentParser(prog="run-calibration.sh", description=f"calibration 측정 (--robot={mode})")
    add_command_args(p)
    p.add_argument("--robot", choices=MODES, required=True, help="sim (task sim_robot.py) · real (task real_robot.py) · dummy (가짜 모터)")
    p.add_argument("--label", default=None, help=f"폴더 이름 <시각>-<label> · CSV label (기본 {TAGS[mode]} / {LABELS[mode]})")
    p.add_argument("--out", "--output", default=None, help="정확한 raw.csv 경로 (isaac/ 기준 상대경로 가능)")
    if mode == "sim":
        from isaaclab.app import AppLauncher

        p.add_argument("--watch", action="store_true", help="기록 후 창 닫을 때까지 같은 경로 반복")
        AppLauncher.add_app_launcher_args(p)
    elif mode == "real":
        p.add_argument("--rate", type=float, default=20.0)
        p.add_argument("--check-only", action="store_true", help="상태만 확인 (로봇 안 움직임)")
    else:
        p.add_argument("--gain", type=float, default=0.99, help="모터 gain (1.0 = 이상)")
        p.add_argument("--gain-l", type=float, default=None)
        p.add_argument("--gain-r", type=float, default=None)
        p.add_argument("--tau", type=float, default=0.05, help="모터 1차 지연 [s]")
        p.add_argument("--delay", type=int, default=2, help="명령 지연 [20 Hz 스텝]")
        p.add_argument("--noise", type=float, default=0.002)
        p.add_argument("--slip", type=float, default=0.0, help="회전 슬립 비율 (0.1 = 몸체 회전 10%% 손실)")
        p.add_argument("--deadband", type=float, default=0.0, help="[rad/s]")
        p.add_argument("--seed", type=int, default=0)
    return p.parse_args(argv)


def _robot(mode: str, args, cmd):
    """mode → (RobotInterface, 기록 뒤 할 일, Isaac 앱 — sim만)."""
    if mode == "real":
        import robots

        return robots.load(task_dir(args), "real_robot").RealRobot(args.rate), None, None
    if mode == "dummy":
        import robots

        g = args.gain
        gain_lr = (g if args.gain_l is None else args.gain_l, g if args.gain_r is None else args.gain_r)
        dummy = robots.load(task_dir(args), "dummy_robot").DummyRobot   # task 폴더의 dummy_robot.py
        return dummy(gain_lr, args.tau, args.delay, args.noise, args.slip, args.deadband, args.seed), None, None
    import robots
    from isaaclab.app import AppLauncher

    app = AppLauncher(args).app
    cfg = task_class(args, "env_cfg", "CalibrationEnvCfg")()
    cfg.robot = task_dir(args)   # 로봇 = task 폴더 (robot_cfg.py · usda/robot.usda)
    cfg.sim.device = args.device
    cfg.debug_vis = "kit" in (getattr(args, "visualizer", None) or [])
    env = task_class(args, "env", "CalibrationEnv")(cfg)
    print(f"[{TAGS[mode]}] 높이 {env.start(cmd.path()):.3f} m · 순수 sim (보정 없음)")
    robot = robots.load(task_dir(args), "sim_robot").SimRobot(env)   # task 폴더의 sim_robot.py
    return robot, (lambda: _watch(app, robot, cmd) if args.watch and cfg.debug_vis else None), app


def _watch(app, robot, cmd) -> None:
    print("[02-sim] WATCH — 창 닫을 때까지 반복")
    k = 0
    while app.is_running():
        robot.write(*cmd((k * robot.dt) % cmd.total))
        k += 1
        time.sleep(robot.dt)


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--robot", choices=MODES)
    mode = pre.parse_known_args(argv)[0].robot
    if mode is None:
        print("사용법: run-calibration.sh <task> --robot=" + "|".join(MODES) + " [옵션...]  (옵션: --robot=<mode> --help)")
        return 0 if {"-h", "--help"} & set(argv) else 2
    args = _args(mode, argv)
    tag, label = TAGS[mode], args.label or LABELS[mode]
    cmd = make_command(args)   # 로봇 · sim 띄우기 전에 실패하게
    out = args.out and (args.out if os.path.isabs(args.out) else str(ISAAC / args.out))
    out = out or run_path(os.path.basename(task_dir(args)), args.label or tag)
    robot, after, app = _robot(mode, args, cmd)
    try:
        if mode == "real":
            print(f"[{tag}] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')}")
        why = robot.ready()
        if getattr(args, "check_only", False):
            print(f"[{tag}] check-only — {'준비됨' if why is None else '거부: ' + why}")
            return 0
        if why is not None:
            print(f"[{tag}] 거부: {why}")
            return 2
        speeds = getattr(cmd, "speeds_text", lambda: "-")()
        print(f"[{tag}] task {args.task} · side={args.side:.2f} m · speeds {speeds} m/s → {out}")
        record(robot, cmd, out, tag, label)
        robot.stop()
        if after:
            after()
        return 0
    except KeyboardInterrupt:
        robot.stop()
        print(f"\n[{tag}] 중단 — 정지")
        return 130
    except BaseException:
        import traceback
        traceback.print_exc()   # sim: app.close()가 프로세스를 바로 끝내 예외가 안 보임 → 먼저 출력
        raise
    finally:
        robot.close()
        if app is not None:
            app.close()
