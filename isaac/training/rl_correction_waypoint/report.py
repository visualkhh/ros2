#!/usr/bin/env python3
"""이 task의 실기기 A/B 보고서 — 같은 명령을 A(보정 없음 · 바이패스) · B(정책 보정)로 달린 두 측정 폴더 비교.

  python3 training/rl_correction_waypoint/report.py <A 폴더> <B 폴더>    (../run-real-ab.sh가 끝에 부른다)
      → <A의 부모 폴더>/compare.png + 지표 표 (B가 A보다 나은지) · 폴더 하나면 그 측정 지표만
폴더 = raw.csv가 있는 곳 (RealRobot 기록: t · lin_raw · ang_raw · lin_out · ang_out · v_enc · w_enc · w_gyro · rl_onnx …).

지표 (작을수록 좋음 — 보정이 일하면 B < A):
  경로 오차   실제 궤적(거리 = 엔코더 v · 방향 = 자이로 적분)이 명령대로의 이상 궤적에서 떨어진 평균 [m]
  끝점 오차   끝에서 이상 궤적 끝점까지 [m]
  직진 중 휨  직진 명령(w = 0, v > 0) 동안 자이로 |w| 평균 [rad/s] — 바퀴 gain 차이로 휘는 것
  보정량      B에서 실제로 나간 명령 − 입력 명령 평균 |Δv| [m/s] · |Δw| [rad/s] (0이면 정책이 일 안 함)
odom 궤적은 쓰지 않는다 — 바퀴가 명령대로 돌면 실제로 휘어도 직선으로 나온다.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))   # isaac/ (reports 패키지)
from reports.base_report import BaseReport  # noqa: E402

METRICS = (("path_err", "경로 오차 [m]"), ("end_err", "끝점 오차 [m]"), ("straight_w", "직진 중 휨 |w| [rad/s]"),
           ("corr_v", "보정량 |Δv| [m/s]"), ("corr_w", "보정량 |Δw| [rad/s]"))
LOWER_BETTER = ("path_err", "end_err", "straight_w")


def integrate(rows, v_key: str, w_key: str):
    """(v, w) 시계열 → 평면 궤적 [(x, y)] (시작 = 원점 · 방향 0)."""
    x = y = yaw = 0.0
    out = [(0.0, 0.0)]
    for a, b in zip(rows, rows[1:]):
        dt = b["t"] - a["t"]
        yaw_mid = yaw + 0.5 * a[w_key] * dt
        x += a[v_key] * math.cos(yaw_mid) * dt
        y += a[v_key] * math.sin(yaw_mid) * dt
        yaw += a[w_key] * dt
        out.append((x, y))
    return out


def metrics(rows) -> dict:
    real = integrate(rows, "v_enc", "w_gyro")
    ideal = integrate(rows, "lin_raw", "ang_raw")
    path_err = sum(min(math.dist(p, q) for q in ideal) for p in real) / len(real)
    straight = [abs(r["w_gyro"]) for r in rows if r["lin_raw"] > 1e-6 and abs(r["ang_raw"]) < 1e-6]
    has_out = "lin_out" in rows[0]
    return {
        "path_err": path_err,
        "end_err": math.dist(real[-1], ideal[-1]),
        "straight_w": sum(straight) / len(straight) if straight else float("nan"),
        "corr_v": sum(abs(r["lin_out"] - r["lin_raw"]) for r in rows) / len(rows) if has_out else float("nan"),
        "corr_w": sum(abs(r["ang_out"] - r["ang_raw"]) for r in rows) / len(rows) if has_out else float("nan"),
        "real": real, "ideal": ideal,
    }


def policy_of(rows) -> str:
    return str(rows[0].get("rl_onnx", "")) or "?"


class AbReport(BaseReport):
    """실기기 A/B — 폴더 2개(A 바이패스 · B 정책): compare.png + 지표 표 · 1개: 그 측정 지표"""

    def single(self, run_dir):
        m = metrics(self.load(run_dir))
        print(f"[ab] {run_dir.name}: " + " · ".join(f"{label} {m[k]:.4f}" for k, label in METRICS))
        return m

    def compare(self, a, b, out=None, show=False):
        ra, rb = self.load(a), self.load(b)
        ma, mb = metrics(ra), metrics(rb)
        print(f"[ab] A = {a.name} ({policy_of(ra)}) · B = {b.name} ({policy_of(rb)})")
        print(f"{'지표':24s}{'A':>10s}{'B':>10s}{'B vs A':>10s}")
        better = 0
        for k, label in METRICS:
            d = mb[k] - ma[k]
            rel = (-d / ma[k] * 100.0) if (k in LOWER_BETTER and ma[k] > 1e-9) else float("nan")
            better += int(k in LOWER_BETTER and d < 0)
            print(f"{label:24s}{ma[k]:10.4f}{mb[k]:10.4f}" + (f"{rel:+9.1f}%" if rel == rel else f"{'':>10s}"))
        print(f"[ab] 좋아진 지표 {better}/{len(LOWER_BETTER)} (경로 · 끝점 · 직진 휨 — B가 작으면 좋아짐)")
        if mb["corr_v"] < 1e-4 and mb["corr_w"] < 1e-4:
            print("[ab] 경고: B 보정량이 0 — 정책이 일을 안 함 (goal 모드 정책이거나 바이패스였는지 rl_onnx 확인)")
        self._plot(a, b, ra, rb, ma, mb, out or str(Path(a).parent / "compare.png"), show)

    @staticmethod
    def _plot(a, b, ra, rb, ma, mb, out, show):
        import matplotlib

        matplotlib.use("TkAgg" if show else "Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
        for m, name, c in ((ma, "A (no correction)", "tab:orange"), (mb, "B (policy)", "tab:blue")):
            ax[0].plot(*zip(*m["real"]), color=c, lw=1.6, label=f"{name} · path err {m['path_err']:.3f} m")
        ax[0].plot(*zip(*ma["ideal"]), "k:", lw=1.4, label="ideal (command)")
        ax[0].set_aspect("equal", adjustable="datalim")
        ax[0].set_title("trajectory (distance = encoder, heading = gyro)")
        ax[0].legend(loc="best", fontsize=8)
        ax[0].grid(True, alpha=0.3)
        for rows, name, c in ((ra, "A", "tab:orange"), (rb, "B", "tab:blue")):
            t = [r["t"] for r in rows]
            ax[1].plot(t, [r["w_gyro"] for r in rows], color=c, lw=1.0, label=f"{name} gyro w")
        ax[1].plot([r["t"] for r in rb], [r["ang_out"] - r["ang_raw"] for r in rb], color="tab:green", lw=1.0,
                   label="B correction Δw")
        ax[1].set_xlabel("t [s]")
        ax[1].set_title("rotation: gyro w (A, B) · B correction")
        ax[1].legend(loc="best", fontsize=8)
        ax[1].grid(True, alpha=0.3)
        fig.suptitle(f"real A/B — {Path(a).name} vs {Path(b).name}")
        fig.tight_layout()
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        fig.savefig(out, dpi=130)
        print(f"[ab] {out}")
        if show:
            plt.show()


if __name__ == "__main__":
    AbReport().main()
