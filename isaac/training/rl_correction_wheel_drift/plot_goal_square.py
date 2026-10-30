"""RL 전/후 사각형+삼각형 궤적 + 시간축 보정값 비교.

  ./training/rl_correction_wheel_drift/plot_goal_square.sh --show
  → training/artifacts/compare_goal_square.png

패널:
  1) x-y 궤적 (Before: open-loop / After: RL goal)
  2) 목표까지 거리 vs time
  3) 실제 적용된 보정 (out - nominal) vs time
  4) nominal vs 최종 명령 vs time
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
ISAAC = HERE.parents[2]
sys.path.insert(0, str(ISAAC))

from training.rl_correction_wheel_drift.calib_profile import DEFAULT_SIDE, profile_path  # noqa: E402

ART = ISAAC / "training" / "artifacts"


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({k: (v if k == "label" else float(v)) for k, v in r.items()})
    return rows


def norm_odom(rows):
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
    den = vx * vx + vy * vy
    t = 0.0 if den == 0 else max(0.0, min(1.0, (wx * vx + wy * vy) / den))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def profile_dev(xs, ys, side):
    pts = profile_path(side)
    ds = [min(seg_dist(x, y, *pts[i], *pts[i + 1]) for i in range(len(pts) - 1))
          for x, y in zip(xs, ys)]
    return (sum(d * d for d in ds) / len(ds)) ** 0.5


def stats(xs, ys, side):
    return {
        "final": math.hypot(xs[-1], ys[-1]),
        "max_r": max(math.hypot(x, y) for x, y in zip(xs, ys)),
        "rms_dev": profile_dev(xs, ys, side),
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
    bx, by = norm_odom(before)
    if "px" in after[0]:
        ax, ay = [r["px"] for r in after], [r["py"] for r in after]
    else:
        ax, ay = norm_odom(after)
    sb, sa = stats(bx, by, args.side), stats(ax, ay, args.side)

    t = [r["t"] for r in after]
    dist = [r.get("dist", 0.0) for r in after]
    dv = [r.get("applied_dv", 0.0) for r in after]
    dw = [r.get("applied_dw", 0.0) for r in after]
    nv = [r.get("nom_v", 0.0) for r in after]
    nw = [r.get("nom_w", 0.0) for r in after]
    ov = [r.get("out_v", 0.0) for r in after]
    ow = [r.get("out_w", 0.0) for r in after]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    axp, axd, axc, axv = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

    ideal = profile_path(args.side)
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
    axp.set_title("square + reverse + triangle")
    axp.legend(loc="best", fontsize=9)
    axp.text(0.02, 0.02,
             f"final  before {sb['final']:.3f} / after {sa['final']:.3f} m\n"
             f"RMS dev before {sb['rms_dev']:.3f} / after {sa['rms_dev']:.3f} m\n"
             f"len    before {sb['len']:.2f} / after {sa['len']:.2f} m",
             transform=axp.transAxes, fontsize=9,
             bbox=dict(facecolor="white", alpha=0.85, edgecolor="0.7"))

    axd.plot(t, dist, color="tab:blue", lw=1.4, label="distance to goal")
    axd.set_xlabel("time [s]")
    axd.set_ylabel("distance [m]")
    axd.set_title("goal distance")
    axd.grid(True, alpha=0.3)
    # goal이 바뀌는 시점 표시
    last = None
    for r in after:
        g = (r.get("gx"), r.get("gy"))
        if g != last:
            axd.axvline(r["t"], color="0.6", ls=":", lw=0.8)
            last = g
    axd.legend(loc="best", fontsize=9)

    axc.plot(t, dv, color="tab:red", lw=1.3, label="applied Δv [m/s]")
    axc.plot(t, dw, color="tab:purple", lw=1.3, label="applied Δw [rad/s]")
    axc.axhline(0, color="k", lw=0.6)
    axc.set_xlabel("time [s]")
    axc.set_ylabel("correction")
    axc.set_title("applied correction (out − nominal)")
    axc.grid(True, alpha=0.3)
    axc.legend(loc="best", fontsize=9)

    axv.plot(t, nv, color="tab:blue", lw=1.2, label="nominal v")
    axv.plot(t, ov, color="tab:cyan", lw=1.2, ls="--", label="out v")
    axv.plot(t, nw, color="tab:orange", lw=1.0, label="nominal w")
    axv.plot(t, ow, color="tab:red", lw=1.0, ls="--", label="out w")
    axv.axhline(0, color="k", lw=0.5)
    axv.set_xlabel("time [s]")
    axv.set_ylabel("m/s · rad/s")
    axv.set_title("nominal vs out")
    axv.grid(True, alpha=0.3)
    axv.legend(loc="best", fontsize=8, ncol=2)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"[plot] {out}")
    print(f"before: final={sb['final']:.3f} rms_dev={sb['rms_dev']:.3f} len={sb['len']:.2f}")
    print(f"after : final={sa['final']:.3f} rms_dev={sa['rms_dev']:.3f} len={sa['len']:.2f}")
    if dv:
        print(f"corr  : |dv| mean={sum(abs(x) for x in dv)/len(dv):.4f} "
              f"max={max(abs(x) for x in dv):.4f} | "
              f"|dw| mean={sum(abs(x) for x in dw)/len(dw):.4f} "
              f"max={max(abs(x) for x in dw):.4f}")
    if args.show:
        import subprocess
        subprocess.run(["xdg-open", str(out)], check=False)


if __name__ == "__main__":
    main()
