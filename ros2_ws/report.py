#!/usr/bin/env python3
"""rl-only 기록(eval_recorder CSV) → report.png. ROS 노드가 아니다 — 파일만 읽는다.
그림 형식은 isaac calibration 리포트(calibration/report/report.py write_report_png)와 맞춘다.

  ./report.py                                          # artifacts/real/raw.csv → artifacts/real/report.png
  ./report.py artifacts/real/raw.csv [--show]
  ./report.py before.csv after.csv [--out compare.png]  # 여러 개면 겹쳐 그림 (선 모양으로 구분)
  한 CSV 안에서 policy.sh로 A/B를 바꿨으면(rl_policy 칸) 구간마다 따로 겹쳐 그리고 지표도 구간별로 낸다.

구성: 맨 위 지표 표(A·B 열 + B vs A 변화율, ↓ 지표가 좋아지면 초록·나빠지면 빨강) → 주행 궤적 → 좌/우 짝 패널
  (속도 · 오차 · 바퀴 오차 · 바퀴 보정량 · 모터 전류 · 온도/지연). CSV에 없는 칸의 패널은 건너뛴다.
  구간마다 시작을 t=0 · odom (0,0) · 방향 0으로 맞춰 A와 B를 같은 축에 겹친다 (run 색 = A 회색 · B 파랑).
정상상태 지표: 명령이 바뀐 뒤 0.5 s는 뺀다 (계단 지연이 RMS를 지배하지 않게). 전체 RMS도 같이 낸다.
자이로 바이어스: 정지 행의 w_gyro 평균을 빼고 회전 지표를 계산하고, 0.02 rad/s 넘으면 경고
  (OpenCR 부팅 때 움직이면 자이로 영점이 틀어짐 — 정책 입력이 오염되므로 그 기록의 B는 무효).
필요: matplotlib (없으면 isaac/.venv/bin/python ./report.py ...)
"""
import argparse
import csv
import math
import sys
from pathlib import Path

WS = Path(__file__).resolve().parent
DEFAULT_CSV = WS / "artifacts" / "real" / "raw.csv"
TEXT = ("label", "state_control", "diag_msg", "rl_model", "rl_policy")
NAN = float("nan")
# 드라이버와 같은 값 (src/tb3_bringup/config/robot.yaml · opencr_driver 비례 축소)
WHEEL_R, WHEEL_SEP, MAX_WHEEL = 0.033, 0.287, 8.0
RUN_LS = ("-", "--", "-.", ":")   # run이 여러 개면 선 모양으로 구분


def load(path: Path) -> list[dict]:
    def num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return NAN
    rows = [{k: (v if k in TEXT else num(v)) for k, v in r.items()} for r in csv.DictReader(open(path))]
    if not rows:
        raise SystemExit(f"빈 CSV: {path}")
    t0 = rows[0]["t"]
    for r in rows:
        r["t_abs"], r["t"] = r["t"], r["t"] - t0
    return rows


def finite(xs):
    return [x for x in xs if x == x]


def unwrap(xs):
    """±π에서 꺾이는 각도를 이어 붙인다 (엔코더/IMU 방향을 같은 축으로 비교)."""
    out, off, prev = [], 0.0, None
    for x in xs:
        if x == x and prev is not None:
            off -= 2 * math.pi * round((x - prev) / (2 * math.pi))
        if x == x:
            prev = x
        out.append(x + off if x == x else x)
    return out


def wheel_cmd(v, w):
    """(v, w) → 좌/우 바퀴 명령 [rad/s]. 드라이버처럼 한쪽이라도 MAX_WHEEL 넘으면 같은 비율로 축소."""
    if v != v or w != w:
        return NAN, NAN
    wl = (v - w * WHEEL_SEP / 2) / WHEEL_R
    wr = (v + w * WHEEL_SEP / 2) / WHEEL_R
    k = min(1.0, MAX_WHEEL / max(abs(wl), abs(wr), 1e-9))
    return wl * k, wr * k


def wheel_corr(rows, dv_key=None, dw_key=None):
    """보정으로 바뀐 바퀴 명령 = 바퀴명령(보정 후) − 바퀴명령(입력). 0이면 보정 없음.
    기본은 실제 적용(lin/ang_out). dv/dw 키를 주면 입력 + 그 값(정책 원출력)."""
    cl, cr = [], []
    for r in rows:
        bl, br = wheel_cmd(r["lin_raw"], r["ang_raw"])
        if dv_key:
            al, ar = wheel_cmd(r["lin_raw"] + r.get(dv_key, NAN), r["ang_raw"] + r.get(dw_key, NAN))
        else:
            al, ar = wheel_cmd(r.get("lin_out", NAN), r.get("ang_out", NAN))
        cl.append(al - bl)
        cr.append(ar - br)
    return cl, cr


def integrate(rows, vw):
    """(v, w) 열을 odom 첫 자세에서 적분 → (xs, ys)."""
    first = next((r for r in rows if r.get("odom_x", NAN) == r.get("odom_x", NAN)), rows[0])
    x, y, yaw = first.get("odom_x", 0.0), first.get("odom_y", 0.0), first.get("odom_yaw", 0.0)
    xs, ys, prev_t = [], [], rows[0]["t"]
    for r, (v, w) in zip(rows, vw):
        dt = r["t"] - prev_t
        prev_t = r["t"]
        if v == v and w == w:
            yaw += w * dt
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
        xs.append(x)
        ys.append(y)
    return xs, ys


def true_path(rows, bias):
    """실측 궤적 — 거리는 v_enc, 방향은 자이로(−바이어스). odom 궤적은 쓰지 않는다:
    odom은 엔코더 회전이라 바퀴가 명령대로 돌면 실제로 휘어도 직선으로 나오고,
    B는 휨을 막으려고 바퀴를 일부러 명령과 다르게 돌리니 odom에선 오히려 휜 것처럼 보인다."""
    return integrate(rows, [(r.get("v_enc", NAN), r.get("w_gyro", NAN) - bias) for r in rows])


def mean_dev(ax_, ay_, bx, by) -> float:
    """같은 시각끼리 두 궤적 거리의 평균 [m]."""
    d = finite([math.hypot(a - c, b - e) for a, b, c, e in zip(ax_, ay_, bx, by)])
    return sum(d) / len(d) if d else NAN


def wheel_gains(rows):
    """실제 바퀴 속도 / 보정 출력의 바퀴 명령 — 움직이는 행 중앙값. 칸이 없으면 None."""
    if not finite([r.get("wheel_l_vel", NAN) for r in rows]):
        return None
    gl, gr = [], []
    for r in rows:
        cl, cr = wheel_cmd(r.get("lin_out", NAN), r.get("ang_out", NAN))
        if abs(cl) > 1.0 and r["wheel_l_vel"] == r["wheel_l_vel"]:
            gl.append(r["wheel_l_vel"] / cl)
        if abs(cr) > 1.0 and r["wheel_r_vel"] == r["wheel_r_vel"]:
            gr.append(r["wheel_r_vel"] / cr)
    if not gl or not gr:
        return None
    return sorted(gl)[len(gl) // 2], sorted(gr)[len(gr) // 2]


def uncorrected_path(rows, g):
    """입력 명령을 바퀴 gain g=(왼, 오)로 재생해 적분 — 보정 안 했다면 이렇게 갔을 것 (추정)."""
    vw = []
    for r in rows:
        cl, cr = wheel_cmd(r["lin_raw"], r["ang_raw"])
        vl, vr = cl * g[0], cr * g[1]
        vw.append((WHEEL_R * (vl + vr) / 2, WHEEL_R * (vr - vl) / WHEEL_SEP))
    return integrate(rows, vw)


BIAS_KEY = "자이로 정지 바이어스 [rad/s] (0이어야 함)"
SETTLE_S = 0.5   # 명령이 바뀐 뒤 이 시간은 정상상태 지표에서 뺀다 (계단 지연이 RMS를 지배하지 않게)


def gyro_bias(rows) -> float:
    """정지(명령 0 · 두 바퀴 정지) 행의 w_gyro 중앙값 — 0이 아니면 OpenCR 부팅 때 자이로 보정 실패 (움직이며 켜짐).

    평균이 아니라 중앙값: 멈춘 직후 몸체가 덜 멈춘 행(캐스터 흔들림)이 평균을 끌어 가짜 바이어스가 나온다."""
    w = finite([r.get("w_gyro", NAN) for r in rows if r["lin_raw"] == 0.0 and r["ang_raw"] == 0.0
                and abs(r.get("wheel_l_vel", 1)) < 0.01 and abs(r.get("wheel_r_vel", 1)) < 0.01])
    return sorted(w)[len(w) // 2] if len(w) >= 20 else 0.0


def metrics(rows: list[dict]) -> dict:
    """작을수록 좋은 지표는 이름 끝이 ↓ — 표에서 B가 A보다 좋으면 초록."""
    moving, steady, last_cmd, changed = [], [], None, -1e9
    for r in rows:
        if r["lin_raw"] != r["lin_raw"]:
            continue
        cmd = (r["lin_raw"], r["ang_raw"])
        if cmd != last_cmd:
            last_cmd, changed = cmd, r["t"]
        if cmd != (0.0, 0.0):
            moving.append(r)
            if r["t"] - changed >= SETTLE_S:
                steady.append(r)
    bias = gyro_bias(rows)

    def rms(sel, a, b, off=0.0):
        d = finite([r[a] - (r.get(b, NAN) - off) for r in sel])
        return math.sqrt(sum(x * x for x in d) / len(d)) if d else NAN

    def col(k):
        return finite([r[k] for r in rows if k in r])

    def mean_abs(xs):
        xs = finite(xs)
        return sum(map(abs, xs)) / len(xs) if xs else NAN

    cl, cr = wheel_corr(moving)
    lat = sorted(finite([(r["t_out"] - r["t_abs"]) * 1000 for r in rows if "t_out" in r]))
    cur = [abs(x) for x in col("wheel_l_cur") + col("wheel_r_cur")]
    sx, sy = true_path(rows, bias)
    cx, cy = integrate(rows, [(r["lin_raw"], r["ang_raw"]) for r in rows])
    fx, fy = finite(sx), finite(sy)
    g = wheel_gains(rows)
    ts = [r["t"] for r in rows]
    dts = [b - a for a, b in zip(ts, ts[1:])]
    dt = sorted(dts)[len(dts) // 2] if dts else NAN
    return {"주행 시간 [s] (명령≠0)": len(moving) * dt,
            "궤적오차 평균 (이상 대비) [m] ↓": mean_dev(sx, sy, cx, cy),
            "끝점 오차 (이상 대비) [m] ↓": math.hypot(fx[-1] - cx[-1], fy[-1] - cy[-1]) if fx else NAN,
            "직진 RMS 정상상태 엔코더 [m/s] (참고 — B는 일부러 다름)": rms(steady, "lin_raw", "v_enc"),
            "회전 RMS 정상상태 엔코더 [rad/s] (참고 — B는 일부러 다름)": rms(steady, "ang_raw", "w_enc"),
            "회전 RMS 정상상태 자이로-바이어스 [rad/s] ↓": rms(steady, "ang_raw", "w_gyro", bias),
            "직진 RMS 전체 (계단 지연 포함) [m/s] ↓": rms(moving, "lin_raw", "v_enc"),
            "회전 RMS 전체 자이로-바이어스 [rad/s] ↓": rms(moving, "ang_raw", "w_gyro", bias),
            "바퀴 gain 왼쪽 (1=정확)": g[0] if g else NAN,
            "바퀴 gain 오른쪽 (1=정확)": g[1] if g else NAN,
            "평균 보정량 왼/오 [rad/s]": (mean_abs(cl) + mean_abs(cr)) / 2,
            "응답 지연 p95 [ms] ↓": lat[int(0.95 * (len(lat) - 1))] if lat else NAN,
            "최대 전류 |A| ↓": max(cur) if cur else NAN,
            BIAS_KEY: bias,
            "비상정지 행 수": sum(1 for r in rows if r.get("state_estop") == 1.0)}


def korean_font(plt):
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    # Noto CJK JP = 같은 ttc(한글 포함) — Ubuntu fonts-noto-cjk는 matplotlib에 JP 이름으로만 잡힘
    for name in ("NanumGothic", "Noto Sans CJK KR", "Noto Sans KR", "Noto Sans CJK JP", "AppleGothic", "Malgun Gothic"):
        if name in names:
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            return


def has(runs, *keys) -> bool:
    """어느 run에든 그 칸들에 값이 하나라도 있으면 패널을 그린다. runs = [(이름, 행)]."""
    return any(all(finite([r.get(k, NAN) for r in run[1]]) for k in keys) for run in runs)


def rebase(rows: list[dict]) -> list[dict]:
    """구간 시작을 t=0 · odom (0,0,0)으로 — A와 B를 같은 축에 겹쳐 비교 (자리·방향이 달라도)."""
    rows = [dict(r) for r in rows]
    t0 = rows[0]["t"]
    first = next((r for r in rows if r.get("odom_x", NAN) == r.get("odom_x", NAN)), None)
    x0, y0, a0 = (first["odom_x"], first["odom_y"], first["odom_yaw"]) if first else (0.0, 0.0, 0.0)
    c, s_ = math.cos(-a0), math.sin(-a0)
    for r in rows:
        r["t"] -= t0
        if first and r.get("odom_x", NAN) == r.get("odom_x", NAN):
            dx, dy = r["odom_x"] - x0, r["odom_y"] - y0
            r["odom_x"], r["odom_y"], r["odom_yaw"] = c * dx - s_ * dy, s_ * dx + c * dy, r["odom_yaw"] - a0
    return rows


def split_phases(path: Path) -> list:
    """CSV 하나 → [(이름, 행)]. policy.sh로 바꾼 연속 구간(rl_policy)마다 하나 — 각 구간은 rebase."""
    rows = load(path)
    ghost = sum(1 for r in rows if r.get("rl_enable") == 1.0 and r.get("rl_policy") == "bypass")
    if ghost:   # 정책을 켰는데 모델 파일이 없어 실제로는 바이패스 — A/B가 아니다
        print(f"[report] 경고: {path.name} — 정책 켬(rl_enable=1)인데 실제로는 바이패스인 행 {ghost}개 "
              f"(모델 파일 없음: rl_model 확인). 이 구간은 '보정함'이 아니다.", file=sys.stderr)
    segs = []
    for r in rows:
        ph = r.get("rl_policy", "") or path.stem
        if not segs or segs[-1][0] != ph:
            segs.append((ph, []))
        segs[-1][1].append(r)
    segs = [(ph, rs) for ph, rs in segs if len(rs) >= 40]   # 2초 미만 조각(전환 순간)은 버림
    seen: dict[str, int] = {}
    out = []
    for ph, rs in segs:
        seen[ph] = seen.get(ph, 0) + 1
        name = ("A 바이패스" if ph == "bypass" else f"B {ph}") + (f" #{seen[ph]}" if seen[ph] > 1 else "")
        out.append((name if len(segs) > 1 else path.parent.name, rebase(rs)))
    return out


def fmt(v, nd=3) -> str:
    if isinstance(v, int):
        return str(v)
    return "—" if v != v else f"{v:.{nd}f}"


COLORS = ("#6b6b6b", "tab:blue", "tab:orange", "tab:green", "tab:purple")   # run 색 — A(바이패스) 회색, B 파랑


def report(paths: list[Path], out: Path, show: bool) -> Path:
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit("matplotlib 없음 — isaac/.venv/bin/python ./report.py ... 로 실행")
    korean_font(plt)

    runs = [run for p in paths for run in split_phases(p)]
    ms = [metrics(rows) for _, rows in runs]
    pairs = [("lin", "ang"), ("err_v", "err_w"), ("wheel_l", "wheel_r")]
    pairs += [("corr_l", "corr_r")] if has(runs, "lin_out") else []
    pairs += [("cur_l", "cur_r")] if has(runs, "wheel_l_cur") else []
    pairs += [("temp", "lat")] if has(runs, "motor_l_temp") or has(runs, "t_out") else []

    fig = plt.figure(figsize=(16, 9 + 2.6 * len(pairs)))
    gs = fig.add_gridspec(2 + len(pairs), 2, height_ratios=[2.6, 3.2] + [1] * len(pairs), hspace=0.42, wspace=0.12,
                          top=0.96, bottom=0.03)
    ax_tab = fig.add_subplot(gs[0, :])
    ax_path = fig.add_subplot(gs[1, :])
    ax = {}
    for i, (a, b) in enumerate(pairs):
        ax[a] = fig.add_subplot(gs[2 + i, 0])
        ax[b] = fig.add_subplot(gs[2 + i, 1], sharex=ax["lin"])
        if i:
            ax[a].sharex(ax["lin"])
    for a_ in ax.values():
        a_.grid(True, alpha=0.3)
    ax_path.grid(True, alpha=0.3)

    # ---- 지표 표 (맨 위) — 2개면 B vs A 변화율, ↓ 지표는 좋아지면 초록·나빠지면 빨강
    names = [n for n, _ in runs]
    keys = list(ms[0])
    two = len(runs) == 2
    cells, colors = [], []
    for k in keys:
        vals = [m[k] for m in ms]
        row = [fmt(v, 1 if "시간" in k else 0 if "ms" in k or "행 수" in k else 3) for v in vals]
        rc = ["white"] * len(vals)
        if two:
            a_, b_ = vals
            pct = (b_ - a_) / abs(a_) * 100 if a_ == a_ and b_ == b_ and abs(a_) > 1e-9 else NAN
            row.append("—" if pct != pct else f"{pct:+.0f}%" if abs(pct) < 300 else f"×{b_ / a_:.1f}")
            good = "↓" in k and pct == pct and abs(pct) >= 3
            rc.append(("#c8f0c8" if pct < 0 else "#f6c6c6") if good else "white")
        if k == BIAS_KEY and any(abs(v) > 0.02 for v in vals):
            rc = ["#ffe08a"] * len(rc)
        cells.append(row)
        colors.append(rc)
    ax_tab.axis("off")
    tab = ax_tab.table(cellText=cells, rowLabels=keys, cellColours=colors, cellLoc="center",
                       colLabels=names + (["B vs A"] if two else []), bbox=[0.3, 0, 0.7, 0.86],
                       colColours=list(COLORS[:len(names)]) + (["white"] if two else []))
    tab.auto_set_font_size(False)
    tab.set_fontsize(9)
    for j in range(len(names)):
        tab[0, j].get_text().set_color("white")
        tab[0, j].get_text().set_weight("bold")
    bad = [f"{n} {m[BIAS_KEY]:+.3f}" for n, m in zip(names, ms) if abs(m[BIAS_KEY]) > 0.02]
    warn = [f"⚠ 자이로 정지 바이어스 {' · '.join(bad)} rad/s (0이어야 함) — OpenCR 부팅 때 로봇이 움직였음. "
            f"가만히 둔 채 OpenCR 다시 켜고 재기록 (정책이 w_gyro를 입력으로 받아 B 결과 무효)"] if bad else []
    ax_tab.text(0, 1.0, "A/B 지표 — 각 구간 시작을 t=0 · 위치 (0,0) · 방향 0으로 맞춰 겹침.  ↓ = 작을수록 좋음 · "
                     f"정상상태 = 명령 바뀐 뒤 {SETTLE_S:g}s 제외", fontsize=10, transform=ax_tab.transAxes, va="bottom")
    if warn:
        ax_tab.text(0, 0.985, "\n".join(warn[:2]), transform=ax_tab.transAxes, va="top", fontsize=10,
                    color="#b00000", weight="bold")

    for i, (name, rows) in enumerate(runs):
        c = COLORS[i % len(COLORS)]
        t = [r["t"] for r in rows]
        bias = gyro_bias(rows)

        def col(k):
            return [r.get(k, NAN) for r in rows]

        # ---- 궤적: 실측 실선 · 명령 100% 점선 (같은 색)
        sx, sy = true_path(rows, bias)
        cx, cy = integrate(rows, zip(col("lin_raw"), col("ang_raw")))
        ax_path.plot(cx, cy, color=c, ls=":", lw=1.4, alpha=0.8)
        ax_path.plot(sx, sy, color=c, lw=2.0, label=f"{name} — 실측: 거리 엔코더 · 방향 자이로 (점선 = 명령대로 갔다면)")
        fx, fy = finite(sx), finite(sy)
        if fx:
            ax_path.plot(fx[-1], fy[-1], "o", color=c, ms=7, mec="k")

        # ---- 속도: 명령(검은 점선, 첫 run만) · 실측(run 색)
        if i == 0:
            ax["lin"].plot(t, col("lin_raw"), "k--", lw=0.9, label="명령")
            ax["ang"].plot(t, col("ang_raw"), "k--", lw=0.9, label="명령")
        ax["lin"].plot(t, col("v_enc"), color=c, lw=1.2, label=name)
        ax["ang"].plot(t, [w - bias for w in col("w_gyro")], color=c, lw=1.2, label=name)
        ax["err_v"].plot(t, [a - b for a, b in zip(col("lin_raw"), col("v_enc"))], color=c, lw=1.0, label=name)
        ax["err_w"].plot(t, [a - (b - bias) for a, b in zip(col("ang_raw"), col("w_gyro"))], color=c, lw=1.0, label=name)

        # ---- 바퀴: 실측 − 명령 (0 = 정확) — 겹쳐도 차이가 보이게 오차로
        bl = [wheel_cmd(r["lin_raw"], r["ang_raw"]) for r in rows]
        ax["wheel_l"].plot(t, [v - b[0] for v, b in zip(col("wheel_l_vel"), bl)], color=c, lw=1.0, label=name)
        ax["wheel_r"].plot(t, [v - b[1] for v, b in zip(col("wheel_r_vel"), bl)], color=c, lw=1.0, label=name)
        if "corr_l" in ax:
            al, ar = wheel_corr(rows)
            ax["corr_l"].plot(t, al, color=c, lw=1.0, label=name)
            ax["corr_r"].plot(t, ar, color=c, lw=1.0, label=name)
        if "cur_l" in ax:
            ax["cur_l"].plot(t, col("wheel_l_cur"), color=c, lw=0.9, label=name)
            ax["cur_r"].plot(t, col("wheel_r_cur"), color=c, lw=0.9, label=name)
        if "temp" in ax:
            ax["temp"].plot(t, col("motor_l_temp"), color=c, lw=1.0, label=f"{name} 왼쪽")
            ax["temp"].plot(t, col("motor_r_temp"), color=c, lw=1.0, ls="--", label=f"{name} 오른쪽")
            ax["lat"].plot(t, [(a - b) * 1000 for a, b in zip(col("t_out"), col("t_abs"))], color=c, lw=0.9, label=name)

    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.set_xlabel("x [m] (구간 시작 방향 = +x)")
    ax_path.set_ylabel("y [m]")
    ax_path.plot(0, 0, "k*", ms=12, label="시작 (모든 구간)")
    ax_path.set_title("주행 궤적 — ●끝 · 실선과 점선이 가까울수록 명령대로 감", fontsize=11, loc="left")
    ax_path.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0), fontsize=9)

    titles = {"lin": "직진 속도 [m/s] — v_enc", "ang": "회전 속도 [rad/s] — 자이로 (바이어스 뺌)",
              "err_v": "직진 오차 [m/s] = 명령 − v_enc", "err_w": "회전 오차 [rad/s] = 명령 − 자이로",
              "wheel_l": "왼쪽 바퀴 오차 [rad/s] = 실측 − 명령", "wheel_r": "오른쪽 바퀴 오차 [rad/s] = 실측 − 명령",
              "corr_l": "왼쪽 바퀴 보정량 [rad/s] (0 = 보정 없음)", "corr_r": "오른쪽 바퀴 보정량 [rad/s] (0 = 보정 없음)",
              "cur_l": "왼쪽 모터 전류 [A]", "cur_r": "오른쪽 모터 전류 [A]",
              "temp": "모터 온도 [°C] (실선 왼 · 점선 오)", "lat": "corrector 응답 지연 [ms]"}
    for k, a in ax.items():
        a.set_title(titles[k], fontsize=10, loc="left")
        if k not in ("temp", "lat"):
            a.axhline(0, color="k", lw=0.6)
        a.tick_params(labelsize=8)
    ax["lin"].legend(loc="upper right", fontsize=8)
    for k in pairs[-1]:
        ax[k].set_xlabel("구간 시작부터 시간 [s]")

    fig.suptitle("  vs  ".join(f"{p.parent.name}/{p.name}" for p in paths), fontsize=13, y=0.997)
    for k in keys:
        print(f"{k}: " + "  ".join(f"[{n}] {fmt(m[k], 4)}" for n, m in zip(names, ms)))
    for w in warn:
        print(f"[report] {w}", file=sys.stderr)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(f"[report] {out}")
    if show:
        plt.show()
    plt.close(fig)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("csv", nargs="*", type=Path, help=f"eval_recorder CSV (기본 {DEFAULT_CSV.relative_to(WS)})")
    ap.add_argument("--out", type=Path, default=None, help="기본: 첫 CSV 폴더/report.png")
    ap.add_argument("--show", action="store_true", help="저장 후 창으로 보기")
    args = ap.parse_args(argv)
    paths = args.csv or [DEFAULT_CSV]
    for p in paths:
        if not p.is_file():
            raise SystemExit(f"CSV 없음: {p}")
    report(paths, args.out or paths[0].parent / "report.png", args.show)


if __name__ == "__main__":
    main()
