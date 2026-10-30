"""sim vs real 시간파형 비교 (같은 명령에서 같은 신호인지 눈으로).

  python3 steps/plot_compare.py /tmp/sim_step.csv /tmp/real_step.csv
  → steps/artifacts/compare.png

위에서부터:
  1) 직진: 명령 lin_raw vs v_enc (sim/real)
  2) 회전: 명령 ang_raw vs w_gyro (sim/real)  ← 부호가 반대면 IMU 축/부호 문제
  3) 직진 오차 (cmd − enc)
  4) 회전 오차 (cmd − gyro)
"""
import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent


def load(path: str):
    rows = []
    for r in csv.DictReader(open(path)):
        rows.append({k: (v if k == "label" else float(v)) for k, v in r.items()})
    if not rows:
        raise SystemExit(f"빈 CSV: {path}")
    t0 = rows[0]["t"]
    for r in rows:
        r["t"] -= t0
    return rows


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def percentile(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))]


def first_step(rows, cmd_key, resp_key, frac):
    for i in range(1, len(rows)):
        target = rows[i][cmd_key]
        prev = rows[i - 1][cmd_key]
        if abs(target) < 1e-6 or abs(prev - target) < 1e-6:
            continue
        goal = frac * target
        for j in range(i, len(rows)):
            if (target > 0 and rows[j][resp_key] >= goal) or (target < 0 and rows[j][resp_key] <= goal):
                return rows[j]["t"] - rows[i]["t"]
        return float("nan")
    return float("nan")


def metrics(rows):
    moving = [r for r in rows if abs(r["lin_raw"]) > 1e-9 or abs(r["ang_raw"]) > 1e-9]
    ev = [r["lin_raw"] - r["v_enc"] for r in moving]
    ew = [r["ang_raw"] - r["w_gyro"] for r in moving]
    ew_enc = [r["ang_raw"] - r["w_enc"] for r in moving]
    return {
        "rows": len(rows),
        "rms_v": rms(ev),
        "rms_w": rms(ew),
        "rms_w_enc": rms(ew_enc),
        "bias_v": sum(ev) / len(ev) if ev else float("nan"),
        "bias_w": sum(ew) / len(ew) if ew else float("nan"),
        "bias_w_enc": sum(ew_enc) / len(ew_enc) if ew_enc else float("nan"),
        "p95_v": percentile([abs(x) for x in ev], 0.95),
        "p95_w": percentile([abs(x) for x in ew], 0.95),
        "t10_v": first_step(rows, "lin_raw", "v_enc", 0.10),
        "t90_v": first_step(rows, "lin_raw", "v_enc", 0.90),
        "t10_w": first_step(rows, "ang_raw", "w_gyro", 0.10),
        "t90_w": first_step(rows, "ang_raw", "w_gyro", 0.90),
    }


def series(rows, key):
    return [r["t"] for r in rows], [r[key] for r in rows]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("sim")
    p.add_argument("real")
    p.add_argument("--out", default=str(HERE / "artifacts" / "compare.png"))
    args = p.parse_args()

    sim, real = load(args.sim), load(args.real)
    ms, mr = metrics(sim), metrics(real)
    t_s, cmd_v_s = series(sim, "lin_raw")
    _, v_s = series(sim, "v_enc")
    t_r, cmd_v_r = series(real, "lin_raw")
    _, v_r = series(real, "v_enc")
    _, cmd_w_s = series(sim, "ang_raw")
    _, w_s = series(sim, "w_gyro")
    _, we_s = series(sim, "w_enc")
    _, cmd_w_r = series(real, "ang_raw")
    _, w_r = series(real, "w_gyro")
    _, we_r = series(real, "w_enc")

    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for a in ax:
        a.grid(True, alpha=0.3)
    for a in ax[:3]:
        a.axvspan(1.0, 3.0, color="tab:green", alpha=0.06)
        a.axvspan(3.0, 5.0, color="tab:green", alpha=0.10)
        a.axvspan(6.0, 8.0, color="tab:orange", alpha=0.08)

    ax[0].plot(t_s, cmd_v_s, "k--", lw=1.2, label="cmd (lin_raw)")
    ax[0].plot(t_s, v_s, color="tab:blue", label=f"sim v_enc (rms {ms['rms_v']:.3f})")
    ax[0].plot(t_r, v_r, color="tab:orange", label=f"real v_enc (rms {mr['rms_v']:.3f})")
    ax[0].set_ylabel("v [m/s]")
    ax[0].legend(loc="lower right", fontsize=8)

    ax[1].plot(t_s, cmd_w_s, "k--", lw=1.2, label="cmd (ang_raw)")
    ax[1].plot(t_s, w_s, color="tab:blue", label=f"sim w_gyro (rms {ms['rms_w']:.3f})")
    ax[1].plot(t_r, w_r, color="tab:orange", label=f"real w_gyro (rms {mr['rms_w']:.3f})")
    ax[1].plot(t_s, we_s, color="tab:blue", ls=":", lw=1.2, label=f"sim w_enc (rms {ms['rms_w_enc']:.3f})")
    ax[1].plot(t_r, we_r, color="tab:orange", ls=":", lw=1.2, label=f"real w_enc (rms {mr['rms_w_enc']:.3f})")
    ax[1].set_ylabel("w [rad/s]")
    ax[1].legend(loc="lower right", fontsize=7)
    ax[1].text(0.01, 0.92, "sign flip -> IMU axis/sign issue", transform=ax[1].transAxes,
               fontsize=8, color="tab:red")

    ev_s = [c - v for c, v in zip(cmd_v_s, v_s)]
    ev_r = [c - v for c, v in zip(cmd_v_r, v_r)]
    ax[2].plot(t_s, ev_s, color="tab:blue", label=f"sim err (bias {ms['bias_v']:+.3f})")
    ax[2].plot(t_r, ev_r, color="tab:orange", label=f"real err (bias {mr['bias_v']:+.3f})")
    ax[2].axhline(0, color="k", lw=0.6)
    ax[2].set_ylabel("v_cmd − v_enc")
    ax[2].legend(loc="lower right", fontsize=8)

    ew_s = [c - w for c, w in zip(cmd_w_s, w_s)]
    ew_r = [c - w for c, w in zip(cmd_w_r, w_r)]
    ax[3].plot(t_s, ew_s, color="tab:blue", label=f"sim err (bias {ms['bias_w']:+.3f})")
    ax[3].plot(t_r, ew_r, color="tab:orange", label=f"real err (bias {mr['bias_w']:+.3f})")
    ax[3].axhline(0, color="k", lw=0.6)
    ax[3].set_ylabel("w_cmd − w_gyro")
    ax[3].set_xlabel("time [s]")
    ax[3].legend(loc="lower right", fontsize=8)

    fig.suptitle(f"sim vs real — {Path(args.real).name}", fontsize=12)
    fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    print(f"[plot] {out}")
    print(f"{'metric':12s}{'sim':>10s}{'real':>10s}{'diff':>10s}")
    for k in ("rms_v", "rms_w", "rms_w_enc", "bias_v", "bias_w", "bias_w_enc", "p95_v", "p95_w"):
        b, r = ms[k], mr[k]
        print(f"{k:12s}{b:10.4f}{r:10.4f}{r - b:+10.4f}")


if __name__ == "__main__":
    main()
