"""calibration 실행기 (공식 train.py 역할) — ./run-calibration.sh <task> --robot=sim|real|dummy [옵션...] = python -m calibration ...

training/<task>/calibration/ 명령 · env와 그 task 폴더의 로봇(sim_robot.py · real_robot.py · dummy_robot.py)으로 한 번 측정 → raw.csv.
  sim    SimRobot + CalibrationEnv — Isaac venv · 창 기본 끔, --viz로 켬 · --watch (기록 후 창 닫을 때까지 반복)
  real   RealRobot — ROS .venv · 준비: Pi base + cmd_manager MANUAL · --check-only (안 움직임)
  dummy  DummyRobot — 표준 라이브러리만 (가짜 모터: --gain-l --gain-r --tau --delay --deadband --slip --noise)
측정: 명령(BaseCommand)을 robot.dt마다 write → pace, 끝나면 robot.data()(로봇이 쌓은 기록) → raw.csv + 구간 표.
출력: artifacts/<task>/calibration/<시각>_<label>/raw.csv · 분석: python3 training/<task>/calibration/report.py <sim 폴더> <real 폴더>
"""
from __future__ import annotations

import argparse
import csv
import inspect
import math
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

from commands import CMDS, command_class

ISAAC = Path(__file__).resolve().parents[1]
ARTIFACTS = ISAAC / "artifacts"   # artifacts/<task>/{calibration, training, logs}/<시각>_<label>
STAMP = "%Y-%m-%d_%H-%M-%S"        # Isaac Lab 학습 폴더와 같은 시각 형식


def run_path(task: str, name: str) -> str:
    """artifacts/<task>/calibration/<시각>_<name>/raw.csv"""
    name = re.sub(r"[^0-9A-Za-z_.-]+", "_", str(name)).strip("._") or "run"
    return str(ARTIFACTS / task / "calibration" / f"{datetime.now():{STAMP}}_{name}" / "raw.csv")


def add_command_args(p) -> None:
    """task · --cmd · --speed · --turn · --side (sim · real · dummy 공통)."""
    p.add_argument("task", help="training/ 아래 task 폴더 이름 (예: rl_correction_waypoint) — 그 calibration/ 명령 · env, 로봇 = 그 폴더")
    p.add_argument("--cmd", default=None, help=f"명령 (기본 task의 calibration/command.py CalibrationCommand): {' · '.join(CMDS)} "
                                              "또는 MOD:CLASS (BaseCommand, 끝이 있는 것). fit · 구간 표는 segs가 있는 명령(square 계열)만")
    p.add_argument("--speed", type=float, default=0.15, help="기준 전진 [m/s] (square: 변마다 1.0/0.67/1.33/0.8배)")
    p.add_argument("--turn", type=float, default=1.0, help="회전 [rad/s]")
    p.add_argument("--side", type=float, default=0.5, help="사각형 한 변 [m] (side를 받는 명령만)")


def task_dir(args) -> str:
    """task → training/<task>/ (= 로봇 폴더: robot_cfg.py · sim_robot.py · real_robot.py · usda/)."""
    from training import task_dir as _task_dir

    return _task_dir(args.task)


def task_class(args, name: str, cls: str):
    """task 폴더의 calibration/<name>.py의 cls. 파일 경로로 읽는다 (task __init__의 gymnasium을 안 거치게)."""
    import robots

    return getattr(robots.load(task_dir(args), f"calibration/{name}"), cls)


def make_command(args):
    """add_command_args 옵션 → 명령 객체. 끝이 없는 명령(keyboard 등)은 측정 불가 → SystemExit."""
    cls = task_class(args, "command", "CalibrationCommand") if args.cmd is None else command_class(args.cmd)
    kw = {"side": args.side} if "side" in inspect.signature(cls).parameters else {}
    cmd = cls(speed=args.speed, turn=args.turn, **kw)
    if not math.isfinite(cmd.total):
        raise SystemExit(f"--cmd {args.cmd}: 끝이 없는 명령은 측정에 못 씀 (total = inf)")
    return cmd


def record(robot, cmd, out: str, tag: str = "calibration", label: str = "") -> list:
    """robot(RobotInterface)에 cmd(BaseCommand, 끝이 있는 것)를 robot.dt마다 write → pace. 끝나면 robot.data() → out에 CSV + 구간 표 (분석은 task의 report.py).

    로봇이 측정 중에 스스로 쌓은 기록(robot.data(), 키 = CSV 열 — 로봇마다 다를 수 있음)에 label만 붙여 저장한다.
    """
    dt = robot.dt
    segs = getattr(cmd, "segs", [])
    print(f"[{tag}] {type(robot).__name__} · {type(cmd).__name__}: {len(segs)} segs · {cmd.total:.1f}s")
    robot.start()
    for k in range(int(round(cmd.total / dt))):
        t = k * dt
        v, w = cmd(t)
        robot.write(v, w)
        robot.pace(k)
    rows = [{**r, "label": label} for r in robot.data()]
    _save(tag, segs, rows, out)
    return rows


def _save(tag: str, segs, rows, out: str) -> None:
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="") as f:
        cw = csv.DictWriter(f, fieldnames=list(rows[0]))
        cw.writeheader()
        cw.writerows(rows)
    print_segments(tag, segs, rows, out)
    print(f"[{tag}] 분석: python3 training/<task>/calibration/report.py <sim 폴더> <real 폴더>  (fit.json · report.png · compare.png · robot_cfg 값)")

SEG_KEYS = ("v_enc", "w_enc", "w_gyro", "wheel_l_vel", "wheel_r_vel")   # 구간 표 (차동 구동 — 이 키가 없는 로봇이면 표를 건너뜀)


def _average(rows, t0: float, t1: float, skip: float = 0.0):
    """[t0+skip, t1) 구간 SEG_KEYS 평균. 비면 [t0, t1)."""
    part = [r for r in rows if t0 + skip <= r["t"] < t1] or [r for r in rows if t0 <= r["t"] < t1]
    return tuple(sum(r[k] for r in part) / len(part) for k in SEG_KEYS)


def print_segments(tag: str, segs, rows, out: str) -> None:
    """구간별 전진/회전 표 — sim · real 같은 형식."""
    if not rows or not all(k in rows[0] for k in SEG_KEYS):
        print(f"[{tag}] 구간 표 스킵 (기록에 {', '.join(SEG_KEYS)} 없음) → {out}")
        return
    for name in ("left", "right"):
        for i, move in enumerate([s for s in segs if s[4] == f"move_{name}"], 1):
            vf, wf, gf, lf, rf = _average(rows, move[0], move[1], skip=0.5)
            print(f"[{tag}] {name.upper():5s} move{i} cmd={move[2]:.3f} v_enc={vf:+.3f} gain={vf / move[2]:.3f} "
                  f"w_enc={wf:+.3f} w_gyro={gf:+.3f} | 바퀴 wl={lf:+.2f} wr={rf:+.2f} rad/s")
        turn = next((s for s in segs if s[4] == f"turn_{name}"), None)
        if turn is not None:
            vt, wt, gt, lt, rt = _average(rows, turn[0], turn[1], skip=0.3)
            print(f"[{tag}] {name.upper():5s} turn  v_enc={vt:+.3f} w_enc={wt:+.3f} w_gyro={gt:+.3f} | "
                  f"바퀴 wl={lt:+.2f} wr={rt:+.2f} rad/s")
    print(f"[{tag}] → {out}")


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


if __name__ == "__main__":
    raise SystemExit(main() or 0)
