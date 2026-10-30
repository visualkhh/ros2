"""RL 전/후 사각형 궤적 비교 — real_floor.csv vs real_goal_square.csv.

  ./rl/wheel_drift/plot_goal_square.sh
  → steps/artifacts/compare_goal_square.png

- 각 궤적은 시작 위치·초기 yaw를 0으로 정렬.
- 회색 점선 = 이상적인 0.5 m 사각형.
- 최종 시작점 오차, 사각형 선에서 벗어난 RMS를 출력.
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parents[1]
sys.path.insert(0, str(ISAAC))

from rl.wheel_drift.calib_profile import DEFAULT_SIDE, square_path  # noqa: E402

ART = ISAAC / "steps" / "artifacts"


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: (v if k == "label" else float(v)) for k, v in r.items()})
    return rows


def norm_path(rows):
    x0, y0 = rows[0]["odom_x"], rows[0]["odom_y"]
    yaw0 = rows[0].get("odom_yaw", 0.0)
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    xs, ys = [], []
    for r in rows:
        dx, dy = r["odom_x"] - x0, r["odom_y"] - y0
        xs.append(c * dx - s * dy)
        ys.append(s * dx + c * dy)
    return xs, ys


def seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    t = 0.0 if vx * vx + vy * vy == 0 else max(0.0, min(1.0, (wx * vx + wy * vy) / (vx * vx + vy * vy)))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def square_dev(xs, ys, side):
    pts = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    ds = []
    for x, y in zip(xs, ys):
        ds.append(min(seg_dist(x, y, *pts[i], *pts[i + 1]) for i in range(4)))
    return (sum(d * d for d in ds) / len(ds)) ** 0.5


def stats(xs, ys, side):
    return {
        "final": math.hypot(xs[-1], ys[-1]),
        "max_r": max(math.hypot(x, y) for x, y in zip(xs, ys)),
        "rms_dev": square_dev(xs, ys, side),
        "len": sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                   for i in range(1, len(xs))),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", default=str(ART / "real_floor.csv"))
    ap.add_argument("--after", default=str(ART / "real_goal_square.csv"))
    ap.add_argument("--side", type=float, default=DEFAULT_SIDE)
    ap.add_argument("--out", default=str(ART / "compare_goal_square.png"))
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    before, after = load(args.before), load(args.after)
    bx, by = norm_path(before)
    ax, ay = norm_path(after)
    sb, sa = stats(bx, by, args.side), stats(ax, ay, args.side)

    fig, axp = plt.subplots(figsize=(8, 7))
    s = args.side
    ideal = square_path(s)
    axp.plot([p[0] for p in ideal], [p[1] for p in ideal],
             "k--", lw=1.2, label="ideal square")
    axp.plot(bx, by, color="tab:orange", lw=1.8, label="before (no RL)")
    axp.plot(ax, ay, color="tab:blue", lw=1.8, label="after (RL goal)")
    axp.plot(0, 0, "ko", ms=6, label="start")
    axp.plot(bx[-1], by[-1], "o", color="tab:orange", ms=6)
    axp.plot(ax[-1], ay[-1], "o", color="tab:blue", ms=6)
    axp.set_aspect("equal", adjustable="datalim")
    axp.grid(True, alpha=0.3)
    axp.set_xlabel("x [m]")
    axp.set_ylabel("y [m]")
    axp.set_title("real square: no RL vs goal RL")
    axp.legend(loc="best", fontsize=9)

    txt = (f"final err  before {sb['final']:.3f} m / after {sa['final']:.3f} m\n"
           f"RMS dev    before {sb['rms_dev']:.3f} m / after {sa['rms_dev']:.3f} m\n"
           f"path len   before {sb['len']:.2f} m / after {sa['len']:.2f} m")
    axp.text(0.02, 0.02, txt, transform=axp.transAxes, fontsize=9,
             bbox=dict(facecolor="white", alpha=0.8, edgecolor="0.7"))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"[plot] {out}")
    print(f"before: final={sb['final']:.3f} rms_dev={sb['rms_dev']:.3f} len={sb['len']:.2f}")
    print(f"after : final={sa['final']:.3f} rms_dev={sa['rms_dev']:.3f} len={sa['len']:.2f}")
    if args.show:
        import subprocess
        subprocess.run(["xdg-open", str(out)], check=False)


if __name__ == "__main__":
    main()
