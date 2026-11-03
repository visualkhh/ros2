"""측정 1회 기록 — sim · real · dummy 공용 (CSV 한 줄 모양 · 구간 표 · fit.json · report.png).

send(v, w)와 collect(t, v, w) → 행 을 받아 같은 명령 경로를 돈다. 런타임(Isaac / ROS / stdlib)이 달라도 기록 형식은 같다.
"""
from __future__ import annotations

import csv
import json
import os
import re
from datetime import datetime
from pathlib import Path


ISAAC = Path(__file__).resolve().parents[1]
RUNS_DIR = ISAAC / "artifacts" / "calibration"
CSV_HEADER = ["t", "lin_raw", "ang_raw", "lin_out", "ang_out",
              "odom_x", "odom_y", "odom_yaw", "v_enc", "w_enc", "w_gyro",
              "wheel_l_vel", "wheel_r_vel", "wheel_l_pos", "wheel_r_pos", "label",
              "t_ros", "t_odom", "t_imu", "t_joint"]


def run_path(name: str) -> str:
    """artifacts/calibration/<시각>-<name>/raw.csv"""
    name = re.sub(r"[^0-9A-Za-z_.-]+", "_", str(name)).strip("._") or "run"
    return str(RUNS_DIR / f"{datetime.now():%Y%m%d_%H%M%S}-{name}" / "raw.csv")


def record(tag: str, send, collect, cmd, out: str, dt: float, pace=None,
           base_fit: dict | None = None) -> list:
    """명령 경로(commands.BaseCommand, 끝이 있는 것 — 보통 SquareCommand)를 dt마다 send → (pace) → collect. out에 CSV + 구간 표 + fit.json + report.png. rows 반환.

    pace(k, t): real의 wall-clock 맞추기 (sim은 물리 스텝이 시계라 없음).
    base_fit: 넣은 입력 보정 → 같은 폴더 base-fit.json.
    """
    segs = getattr(cmd, "segs", [])
    print(f"[{tag}] {type(cmd).__name__}: {len(segs)} segs · {cmd.total:.1f}s")
    rows = []
    for k in range(int(round(cmd.total / dt))):
        t = k * dt
        v, w = cmd(t)
        send(v, w)
        if pace is not None:
            pace(k, t)
        rows.append(collect(t, v, w))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(CSV_HEADER)
        cw.writerows(rows)
    run_dir = os.path.dirname(out)
    if base_fit is not None:
        _write_json(os.path.join(run_dir, "base-fit.json"),
                    {"source": base_fit["source"], "fit": {k: v for k, v in base_fit.items() if k != "source"}})
    print_segments(tag, segs, rows, out)
    try:
        from calibration.report.report import fit_wheel, write_report_png
        fit = fit_wheel([dict(zip(CSV_HEADER, r)) for r in rows])
        if fit is not None:
            _write_json(os.path.join(run_dir, "fit.json"), {"source": os.path.basename(out), "fit": fit})
            print(f"[{tag}] fit.json (gain L/R={fit['wheel_gain_l']}/{fit['wheel_gain_r']} delay={fit['wheel_delay_steps']} "
                  f"tau={fit['wheel_tau_s']} db={fit['wheel_deadband']})")
        else:
            print(f"[{tag}] fit 스킵 (전진 구간 없음)")
        write_report_png(out, fit=fit)
    except Exception as exc:
        print(f"[{tag}] report.png/fit.json 스킵 ({exc})")
    return rows


def _write_json(path: str, doc: dict) -> None:
    with open(path, "w") as f:
        f.write(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
    print(f"→ {path}")


def _average(rows, t0: float, t1: float, skip: float = 0.0):
    """[t0+skip, t1) 구간 (v_enc, w_enc, w_gyro, wl, wr) 평균. 비면 [t0, t1)."""
    part = [r for r in rows if t0 + skip <= r[0] < t1] or [r for r in rows if t0 <= r[0] < t1]
    return tuple(sum(r[i] for r in part) / len(part) for i in (8, 9, 10, 11, 12))


def print_segments(tag: str, segs, rows, out: str) -> None:
    """구간별 전진/회전 표 — sim · real 같은 형식."""
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
