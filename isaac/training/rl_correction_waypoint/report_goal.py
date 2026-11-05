#!/usr/bin/env python3
"""Goal 모드 A/B 보고서 — 같은 `/rl_goal`을 A(바이패스) · B(정책)로 달린 기록 비교.

`rl-only.sh`가 쓴 CSV(또는 그 폴더의 raw.csv) 두 개를 받는다. 네모(track)용 `report.py`와 달리
`lin_raw/ang_raw`(입력 명령) 대신 **실제 궤적**(엔코더 v · 자이로 w 적분)을 목표 점과 비교한다.

  python3 training/rl_correction_waypoint/report_goal.py <A.csv|A폴더> <B.csv|B폴더> --goal 1.0,0.0
      → A/B 궤적 · 지표 · <A 옆>/compare_goal.png

지표 (작을수록 좋음):
  path_err  실제 궤적이 시작(0,0) → goal 직선에서 벗어난 평균 거리 [m]
  max_cross 위 직선에서 가장 크게 벗어난 거리 [m]
  end_err   실제 마지막 점에서 goal까지 [m]
  corr_v/w  정책 원출력 |model_dv|/|model_dw| 평균 (A 바이패스 ≈ 0)
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

METRICS = (("path_err", "경로 오차 [m]"), ("max_cross", "최대 이탈 [m]"), ("end_err", "끝점 오차 [m]"),
           ("corr_v", "정책 |Δv| [m/s]"), ("corr_w", "정책 |Δw| [rad/s]"))
LOWER_BETTER = ("path_err", "max_cross", "end_err")


def integrate(rows):
    """(v_enc, w_gyro) 시계열 → 평면 궤적 [(x, y)] (시작 = 원점 · 방향 0)."""
    x = y = yaw = 0.0
    out = [(0.0, 0.0)]
    for a, b in zip(rows, rows[1:]):
        dt = b["t"] - a["t"]
        yaw_mid = yaw + 0.5 * a["w_gyro"] * dt
        x += a["v_enc"] * math.cos(yaw_mid) * dt
        y += a["v_enc"] * math.sin(yaw_mid) * dt
        yaw += a["w_gyro"] * dt
        out.append((x, y))
    return out


def load(path):
    p = Path(path)
    if p.is_dir():
        p = p / "raw.csv"
    rows = []
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            row = {}
            for k, v in r.items():
                try:
                    row[k] = float(v)
                except (TypeError, ValueError):
                    row[k] = v
            rows.append(row)
    if not rows:
        raise SystemExit(f"빈 CSV: {p}")
    return rows


def seg_dist(x, y, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def mean_abs(rows, key):
    vals = [abs(r[key]) for r in rows if isinstance(r.get(key), float) and math.isfinite(r[key])]
    return sum(vals) / len(vals) if vals else float("nan")


def metrics(rows, goal):
    xy = integrate(rows)
    gx, gy = goal
    ds = [seg_dist(x, y, 0.0, 0.0, gx, gy) for x, y in xy]
    ex, ey = xy[-1]
    return {
        "path_err": sum(ds) / len(ds),
        "max_cross": max(ds),
        "end_err": math.hypot(ex - gx, ey - gy),
        "corr_v": mean_abs(rows, "model_dv"),
        "corr_w": mean_abs(rows, "model_dw"),
        "xy": xy,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description="goal 모드 A/B 보고서 (실제 궤적 vs /rl_goal)")
    ap.add_argument("a", help="A 기록 (raw.csv 또는 폴더)")
    ap.add_argument("b", help="B 기록 (raw.csv 또는 폴더)")
    ap.add_argument("--goal", required=True, help="목표 x,y [m] (시작 자세 기준, 예: 1.0,0.0)")
    ap.add_argument("--out", default=None, help="기본: <A 옆>/compare_goal.png")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args(argv)

    try:
        gx, gy = (float(v) for v in args.goal.split(","))
    except ValueError:
        ap.error("--goal은 'x,y' 형식 (예: 1.0,0.0)")

    a, b = load(args.a), load(args.b)
    ma, mb = metrics(a, (gx, gy)), metrics(b, (gx, gy))
    print(f"[goal-ab] A = {Path(args.a).name} · B = {Path(args.b).name} · goal=({gx:+.2f},{gy:+.2f})")
    print(f"{'지표':24s}{'A':>10s}{'B':>10s}{'B vs A':>10s}")
    better = 0
    for k, label in METRICS:
        d = mb[k] - ma[k]
        rel = (-d / ma[k] * 100.0) if (k in LOWER_BETTER and ma[k] > 1e-9) else float("nan")
        better += int(k in LOWER_BETTER and d < 0)
        print(f"{label:24s}{ma[k]:10.4f}{mb[k]:10.4f}" + (f"{rel:+9.1f}%" if rel == rel else f"{'':>10s}"))
    print(f"[goal-ab] 좋아진 지표 {better}/{len(LOWER_BETTER)} (경로 · 최대 이탈 · 끝점 — B가 작으면 좋아짐)")
    if mb["corr_v"] < 1e-4 and mb["corr_w"] < 1e-4:
        print("[goal-ab] 경고: B 정책 출력이 0 — 바이패스였거나 ONNX가 안 붙었는지 확인 (rl-only.sh log)")

    import matplotlib

    matplotlib.use("TkAgg" if args.show else "Agg")
    import matplotlib.pyplot as plt

    out = Path(args.out) if args.out else Path(args.a).parent / "compare_goal.png"
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    ax.plot(*zip(*ma["xy"]), color="tab:orange", lw=1.8, label=f"A (bypass) · end {ma['end_err']:.3f} m")
    ax.plot(*zip(*mb["xy"]), color="tab:blue", lw=1.8, label=f"B (policy) · end {mb['end_err']:.3f} m")
    ax.plot([0, gx], [0, gy], "k:", lw=1.2, label="goal line")
    ax.plot([gx], [gy], "r*", ms=14, label="goal")
    ax.plot([0], [0], "ko", ms=5, label="start")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m] (start-relative)")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print(f"[goal-ab] {out}")
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
