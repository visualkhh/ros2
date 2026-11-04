"""calibration 측정 1회 (system identification) — record(robot, cmd, out): 명령(excitation, 보통 SquareCommand)을 로봇(RobotInterface)에 흘려보내며 기록.

로봇이 sim · real · dummy 어느 것이든 기록 형식(CSV 한 줄 · 구간 표 · fit.json · report.png)은 같다. 이 파일은 표준 라이브러리만.
"""
from __future__ import annotations

import csv
import inspect
import math
import os
import re
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
    print(f"[{tag}] 분석: python3 training/<task>/report.py <sim 폴더> <real 폴더>  (fit.json · report.png · compare.png · robot_cfg 값)")

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
