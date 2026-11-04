"""이 task의 calibration 보고서 — fit(바퀴 gain · 지연 · τ · 데드밴드) + 단일 run 그림 + sim/real 비교.
이 로봇(차동 구동)의 기록 열(v_enc · w_enc · w_gyro · wheel_* · odom_* · t_*)을 쓴다 — 기록이 다른 task는 자기 report.py를.

단일: ./run-calibration.sh <task> plot-compare artifacts/<task>/calibration/<시각>_<label>/raw.csv [--out report.png]
비교: ./run-calibration.sh <task> plot-compare <sim.csv> <real.csv> [--out compare.png] [--calib-out sim_calib.json]
측정이 끝나면 calibration/record.py가 fit_wheel · write_report_png를 부른다.

비교 패널: 0) odom x-y 궤적 (왼쪽 사각형 → 180° 턴 → 같은 사각형 역주행)
  1) 직진: 명령 lin_raw vs v_enc (sim/real)
  2) 회전: 명령 ang_raw vs w_gyro (sim/real) ← 부호가 반대면 IMU 축/부호 문제
  3) 직진 오차 (cmd − enc)
  4) 회전 오차 (cmd − gyro)
  5) raw 바퀴 속도
  6) msg age (t_ros − t_msg, ms) — sim은 0, real은 WiFi 지터

calib: 모든 전진 구간(변마다 속도 다름)에서 gain/t10/tau를 뽑아 중앙값을 쓰고,
       속도별 값은 JSON `speed_fit`에 같이 저장한다.
"""
import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import matplotlib

# --show가 있을 때만 GUI 백엔드. 기본은 저장만(Agg).
_SHOW = "--show" in sys.argv
if _SHOW:
    try:
        matplotlib.use("TkAgg")
    except Exception:
        matplotlib.use("Agg")
else:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _korean_font():
    """한글 범례용 폰트. 시스템에 있는 것 중 첫 번째로 잡히는 걸 쓴다."""
    from matplotlib import font_manager, rcParams
    try:
        names = {f.name for f in font_manager.fontManager.ttflist}
    except Exception:
        return None
    for name in ("Apple SD Gothic Neo", "AppleGothic", "NanumGothic",
                 "Malgun Gothic", "Noto Sans CJK KR", "Noto Sans KR",
                 "Noto Sans CJK JP"):  # 같은 ttc(한글 포함) — Ubuntu fonts-noto-cjk는 matplotlib에 JP 이름으로만 잡힘
        if name in names:
            rcParams["font.family"] = [name, "DejaVu Sans"]
            rcParams["axes.unicode_minus"] = False
            return name
    return None


_KOREAN_FONT = _korean_font()

HERE = Path(__file__).resolve().parent
ISAAC = HERE.parents[2]          # .../isaac (HERE = isaac/training/<task>/calibration)
OUT_DIR = ISAAC / "artifacts" / HERE.parent.name / "calibration"   # 비교 그림 기본 위치
sys.path.insert(0, str(ISAAC))

def _robot_cfg():
    """이 task 폴더의 robot_cfg.py (바퀴 숫자)."""
    import os

    import robots

    return robots.load(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


CONTROL_HZ = 20.0   # 기록 · 정책 주기 (= envs/base_env_cfg.py CONTROL_HZ — Isaac 없이 돌아서 따로, 바꾸면 같이)


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


def _gain(fit, key):
    """좌/우 개별 gain. 없거나 None이면 공통 wheel_gain으로 대체."""
    v = fit.get(key)
    return float(fit.get("wheel_gain", 1.0)) if v is None else float(v)


def fit_wheels(rows, t, fit):
    """피팅 모델로 좌/우 바퀴 각속도 예측.

    v/w 명령 → (지연) → 차동기구학 → 8 rad/s 비례 축소 → gain → deadband → 1차지연.
    (학습 env 바퀴 모델 · wheel_targets와 같은 순서)
    """
    _rc = _robot_cfg()
    MAX_WHEEL, WHEEL_RADIUS, WHEEL_SEP = _rc.MAX_WHEEL, _rc.WHEEL_RADIUS, _rc.WHEEL_SEP
    dts = sorted(t[i + 1] - t[i] for i in range(len(t) - 1) if t[i + 1] > t[i])
    dt = dts[len(dts) // 2] if dts else 1.0 / CONTROL_HZ
    gl = _gain(fit, "wheel_gain_l")
    gr = _gain(fit, "wheel_gain_r")
    tau = max(float(fit.get("wheel_tau_s", 0.05)), 1e-3)
    dl = int(fit.get("wheel_delay_steps", 0))
    deadband = float(fit.get("wheel_deadband", 0.0))
    alpha = 1.0 - math.exp(-dt / tau)

    wheels = []
    for r in rows:
        wl = (r["lin_raw"] - r["ang_raw"] * WHEEL_SEP / 2.0) / WHEEL_RADIUS
        wr = (r["lin_raw"] + r["ang_raw"] * WHEEL_SEP / 2.0) / WHEEL_RADIUS
        peak = max(abs(wl), abs(wr))
        if peak > MAX_WHEEL:
            k = MAX_WHEEL / peak
            wl, wr = wl * k, wr * k
        wheels.append((wl, wr))

    pl, pr, yl, yr = [], [], 0.0, 0.0
    for i in range(len(rows)):
        wl, wr = wheels[i - dl] if i - dl >= 0 else (0.0, 0.0)
        tl, tr = wl * gl, wr * gr
        if abs(tl) < deadband:
            tl = 0.0
        if abs(tr) < deadband:
            tr = 0.0
        yl += alpha * (tl - yl)
        yr += alpha * (tr - yr)
        pl.append(yl)
        pr.append(yr)
    return pl, pr


def fit_v(rows, t, fit):
    """좌/우 바퀴 모델 → 차동기구학으로 몸체 선속도 v 예측. (wl+wr)·r/2."""
    _rc = _robot_cfg()
    WHEEL_RADIUS = _rc.WHEEL_RADIUS
    pl, pr = fit_wheels(rows, t, fit)
    return [WHEEL_RADIUS * (a + b) / 2.0 for a, b in zip(pl, pr)]


def fit_ang(rows, t, fit):
    """좌/우 바퀴 모델 → 차동기구학으로 각속도 w 예측. (wr − wl)·r/sep."""
    _rc = _robot_cfg()
    WHEEL_RADIUS, WHEEL_SEP = _rc.WHEEL_RADIUS, _rc.WHEEL_SEP
    pl, pr = fit_wheels(rows, t, fit)
    return [(r_ - l_) * WHEEL_RADIUS / WHEEL_SEP for l_, r_ in zip(pl, pr)]


def path(rows):
    """odom x-y를 시작 위치·초기 yaw 기준으로 정렬 (둘 다 +x에서 출발하게)."""
    x0, y0 = rows[0]["odom_x"], rows[0]["odom_y"]
    yaw0 = rows[0].get("odom_yaw", 0.0)
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    xs, ys = [], []
    for r in rows:
        dx, dy = r["odom_x"] - x0, r["odom_y"] - y0
        xs.append(dx * c - dy * s)
        ys.append(dx * s + dy * c)
    return xs, ys


def mean_dev(ax_, ay_, bx, by):
    """두 궤적의 평균 점간 거리 [m]. 길이가 다르면 짧은 쪽에 맞춘다."""
    n = min(len(ax_), len(bx))
    if n == 0:
        return float("nan")
    return sum(math.hypot(ax_[i] - bx[i], ay_[i] - by[i]) for i in range(n)) / n


def fit_path(rows, t, fit):
    """피팅 모델로 재구성한 궤적. v는 fit_v, yaw는 fit_ang(바퀴 모델) 적분."""
    dts = sorted(t[i + 1] - t[i] for i in range(len(t) - 1) if t[i + 1] > t[i])
    dt = dts[len(dts) // 2] if dts else 1.0 / CONTROL_HZ
    v_pred = fit_v(rows, t, fit)
    w_pred = fit_ang(rows, t, fit)
    xs, ys, x, y, yaw = [0.0], [0.0], 0.0, 0.0, 0.0
    for v, w in zip(v_pred, w_pred):
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        xs.append(x)
        ys.append(y)
    return xs, ys


def cmd_path(rows, t):
    """명령을 100% 그대로 따랐다고 가정한 이상 궤적 (gain=1, 지연·시정수 0)."""
    dts = sorted(t[i + 1] - t[i] for i in range(len(t) - 1) if t[i + 1] > t[i])
    dt = dts[len(dts) // 2] if dts else 1.0 / CONTROL_HZ
    xs, ys, x, y, yaw = [0.0], [0.0], 0.0, 0.0, 0.0
    for r in rows:
        yaw += r["ang_raw"] * dt
        x += r["lin_raw"] * math.cos(yaw) * dt
        y += r["lin_raw"] * math.sin(yaw) * dt
        xs.append(x)
        ys.append(y)
    return xs, ys


def corrected_path(rows, t, fit):
    """보정 후 예상 궤적: 좌/우 gain만 역보상(명령/gain)한 뒤 plant에 넣은 결과.

    fit model(빨강)은 실제를 재현하는 모델이고, 이건 그 gain을 역으로 보상했을 때
    실제(파랑)가 이상(초록) 쪽으로 얼마나 붙는지를 보여준다. 지연/tau는 남는다.
    """
    _rc = _robot_cfg()
    MAX_WHEEL, WHEEL_RADIUS, WHEEL_SEP = _rc.MAX_WHEEL, _rc.WHEEL_RADIUS, _rc.WHEEL_SEP
    dts = sorted(t[i + 1] - t[i] for i in range(len(t) - 1) if t[i + 1] > t[i])
    dt = dts[len(dts) // 2] if dts else 1.0 / CONTROL_HZ
    gl = _gain(fit, "wheel_gain_l")
    gr = _gain(fit, "wheel_gain_r")
    tau = max(float(fit.get("wheel_tau_s", 0.05)), 1e-3)
    dl = int(fit.get("wheel_delay_steps", 0))
    deadband = float(fit.get("wheel_deadband", 0.0))
    alpha = 1.0 - math.exp(-dt / tau)
    wl_ideal = [(r["lin_raw"] - r["ang_raw"] * WHEEL_SEP / 2.0) / WHEEL_RADIUS for r in rows]
    wr_ideal = [(r["lin_raw"] + r["ang_raw"] * WHEEL_SEP / 2.0) / WHEEL_RADIUS for r in rows]
    # fit gain으로 명령을 미리 역보상: u = 목표 / gain (지연·포화·데드밴드는 plant와 같이 남긴다)
    wl_in = [x / gl for x in wl_ideal]
    wr_in = [x / gr for x in wr_ideal]

    xs, ys, x, y, yaw = [0.0], [0.0], 0.0, 0.0, 0.0
    yl = yr = 0.0
    for i in range(len(rows)):
        if i - dl >= 0:
            ul, ur = wl_in[i - dl], wr_in[i - dl]
            peak = max(abs(ul), abs(ur))
            if peak > MAX_WHEEL:
                k = MAX_WHEEL / peak
                ul, ur = ul * k, ur * k
            tl, tr = ul * gl, ur * gr
        else:
            tl = tr = 0.0
        if abs(tl) < deadband:
            tl = 0.0
        if abs(tr) < deadband:
            tr = 0.0
        yl += alpha * (tl - yl)
        yr += alpha * (tr - yr)
        v = WHEEL_RADIUS * (yl + yr) / 2.0
        w = WHEEL_RADIUS * (yr - yl) / WHEEL_SEP
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        xs.append(x)
        ys.append(y)
    return xs, ys


def write_report_png(csv_path, out_path=None, fit=None):
    """단일 run CSV → 시간축 파형 + 궤적 PNG. matplotlib 없으면 None.

    fit이 있으면 (fit_wheel 결과 dict) 값을 그림 우상단에 박스로 표시한다.
    """
    try:
        import matplotlib as _mpl
        _mpl.use("Agg")
        import matplotlib.pyplot as _plt
    except Exception as exc:
        print(f"[report] matplotlib 없음 — PNG 스킵 ({exc})")
        return None
    global plt
    plt = _plt
    rows = load(str(csv_path))
    t = [r["t"] for r in rows]

    has_path = all(k in rows[0] for k in ("odom_x", "odom_y"))
    has_wheels = "wheel_l_vel" in rows[0]
    has_stamp = all(k in rows[0] for k in ("t_ros", "t_odom", "t_imu", "t_joint"))
    n_time = 5 if has_wheels else 3
    if has_stamp:
        n_time += 1

    if has_path:
        fig = plt.figure(figsize=(11, 4 + 2 * n_time))
        gs = fig.add_gridspec(n_time + 1, 1, height_ratios=[1.7] + [1] * n_time)
        ax_path = fig.add_subplot(gs[0])
        ax = [fig.add_subplot(gs[i + 1]) for i in range(n_time)]
        for a in ax[1:]:
            a.sharex(ax[0])
    else:
        fig, axs = plt.subplots(n_time, 1, figsize=(11, 3 + 2 * n_time), sharex=True)
        ax = list(axs)
        ax_path = None
    for a in ([ax_path] if ax_path else []) + ax:
        a.grid(True, alpha=0.3)

    if ax_path is not None:
        sx, sy = path(rows)
        cx, cy = cmd_path(rows, t)
        ax_path.plot(cx, cy, color="tab:green", ls=":", lw=1.8, label="명령 100% 궤적 (이상)")
        ax_path.plot(sx, sy, color="tab:blue", lw=1.2, label="odom 실측 궤적")
        if fit:
            fx, fy = fit_path(rows, t, fit)
            ax_path.plot(fx, fy, color="tab:red", ls="--", lw=1.2, label="fit model 피팅모델 (실제 재현)")
            qx, qy = corrected_path(rows, t, fit)
            ax_path.plot(qx, qy, color="magenta", lw=3.2, alpha=0.35,
                         label="보정 후 예상 궤적 (gain 역보상, 지연 남음)")
            me = mean_dev(sx, sy, cx, cy)
            mc = mean_dev(qx, qy, cx, cy)
            ax_path.text(0.02, 0.02,
                         f"이상 대비 평균 궤적오차: 실측 {me:.3f} m → 보정 후 {mc:.3f} m",
                         transform=ax_path.transAxes, fontsize=8, color="tab:purple",
                         bbox=dict(boxstyle="round", facecolor="white", alpha=0.7))
        ax_path.plot(sx[0], sy[0], "ko", ms=5, label="start")
        ax_path.plot(sx[-1], sy[-1], "r.", ms=8, label="end")
        ax_path.set_aspect("equal", adjustable="datalim")
        ax_path.set_xlabel("x 위치 [m]")
        ax_path.set_ylabel("y 위치 [m]")
        ax_path.set_title("odom 주행 궤적")
        ax_path.legend(loc="best", fontsize=8)

    ax[0].plot(t, [r["lin_raw"] for r in rows], "k--", lw=1.0, label="cmd 명령 (lin_raw)")
    ax[0].plot(t, [r["v_enc"] for r in rows], color="tab:blue", label="v_enc 엔코더 속도")
    fit_v_line = fit_v(rows, t, fit) if fit else None
    if fit_v_line is not None:
        ax[0].plot(t, fit_v_line,
                    color="tab:red", ls="--", lw=1.2,
                    label=f"fit model 피팅모델 (g L/R={_gain(fit, 'wheel_gain_l'):.3f}/{_gain(fit, 'wheel_gain_r'):.3f})")
    ax[0].set_ylabel("직진 속도 v [m/s]\n(몸체 선속도)")
    ax[0].legend(loc="lower right", fontsize=8)

    ax[1].plot(t, [r["ang_raw"] for r in rows], "k--", lw=1.0, label="cmd 명령 (ang_raw)")
    ax[1].plot(t, [r["w_gyro"] for r in rows], color="tab:blue", label="w_gyro 자이로 실측")
    ax[1].plot(t, [r["w_enc"] for r in rows], color="tab:blue", ls=":", lw=1.0, label="w_enc 엔코더 계산")
    fit_w = fit_ang(rows, t, fit) if fit else None
    if fit_w is not None:
        ax[1].plot(t, fit_w, color="tab:red", ls="--", lw=1.2, label="fit model 피팅모델 (바퀴→각속도)")
    ax[1].set_ylabel("회전 속도 w [rad/s]\n(몸체 각속도)")
    ax[1].legend(loc="lower right", fontsize=8)

    ax[2].plot(t, [r["lin_raw"] - r["v_enc"] for r in rows], color="tab:blue", label="v_cmd − v_enc 직진 오차")
    ax[2].plot(t, [r["ang_raw"] - r["w_gyro"] for r in rows], color="tab:orange", label="w_cmd − w_gyro 회전 오차")
    if fit_v_line is not None:
        ax[2].plot(t, [c - f for c, f in zip([r["lin_raw"] for r in rows], fit_v_line)],
                    color="tab:red", ls=":", lw=1.2, label="직진 fit 잔차 (모델로 설명 안 되는 부분)")
    if fit_w is not None:
        ax[2].plot(t, [r["ang_raw"] - f for r, f in zip(rows, fit_w)],
                    color="tab:brown", ls=":", lw=1.2, label="회전 fit 잔차 (모델로 설명 안 되는 부분)")
    ax[2].axhline(0, color="k", lw=0.6)
    ax[2].set_ylabel("추종 오차 [m/s 또는 rad/s]\n(명령 − 실측)")
    ax[2].legend(loc="lower right", fontsize=8)

    i = 3
    if has_wheels:
        fwl = fwr = None
        if fit:
            fwl, fwr = fit_wheels(rows, t, fit)
        ax[i].plot(t, [r["wheel_l_vel"] for r in rows], color="tab:blue", label="실측 왼쪽 바퀴 (wheel L)")
        if fwl is not None:
            ax[i].plot(t, fwl, color="tab:red", ls="--", lw=1.0, label="fit 왼쪽바퀴")
        ax[i].axhline(0, color="k", lw=0.6)
        ax[i].set_ylabel("왼쪽 바퀴 각속도 [rad/s]")
        ax[i].legend(loc="lower right", fontsize=8)
        i += 1
        ax[i].plot(t, [r["wheel_r_vel"] for r in rows], color="tab:cyan", label="실측 오른쪽 바퀴 (wheel R)")
        if fwr is not None:
            ax[i].plot(t, fwr, color="tab:orange", ls="--", lw=1.0, label="fit 오른쪽바퀴")
        ax[i].axhline(0, color="k", lw=0.6)
        ax[i].set_ylabel("오른쪽 바퀴 각속도 [rad/s]")
        ax[i].legend(loc="lower right", fontsize=8)
        i += 1
    if has_stamp:
        for key, label, color in (("t_odom", "odom", "tab:blue"),
                                  ("t_imu", "imu", "tab:green"),
                                  ("t_joint", "joint", "tab:purple")):
            age = [(r["t_ros"] - r[key]) * 1000.0 if r[key] == r[key] else float("nan")
                   for r in rows]
            ax[i].plot(t, age, color=color, lw=1.0, label=label)
        ax[i].axhline(0, color="k", lw=0.6)
        ax[i].set_ylabel("메시지 지연 [ms]\n(수신 − 발행)")
        ax[i].legend(loc="lower right", fontsize=8)
    ax[-1].set_xlabel("시간 [s]")

    csv_path = Path(csv_path)
    out_path = Path(out_path) if out_path else csv_path.parent / "report.png"
    fig.suptitle(f"{csv_path.parent.name}/{csv_path.name}", fontsize=12)
    if fit:
        fig.text(0.99, 0.97,
                 f"fit: gain L/R={_gain(fit, 'wheel_gain_l'):.3f}/{_gain(fit, 'wheel_gain_r'):.3f} "
                 f"delay={fit.get('wheel_delay_steps', float('nan')):.0f} steps "
                 f"tau={fit.get('wheel_tau_s', float('nan')):.3f}s "
                 f"db={fit.get('wheel_deadband', 0.0):.3f}",
                 ha="right", va="top", fontsize=9,
                 bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))
    fig.tight_layout(rect=(0, 0, 1, 0.97) if ax_path is not None else None)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[report] {out_path}")
    return str(out_path)


def fit_wheel(real):
    """real CSV의 모든 구간(전진/후진/회전/단륜/고부하)에서 바퀴별 gain·dead time·tau·deadband를 뽑는다.

    - 명령은 드라이버와 같은 식으로 환산한다: v,w → 바퀴 rad/s → 8 rad/s 비례 축소.
    - 대표 gain/tau/delay는 **포화 구간과 저속(데드밴드 탐색) 구간을 뺀** 정상 주행
      샘플의 중앙값이다.
    - 단륜·저속·고부하 구간은 진단값(`speed_fit`, `power`, sat)으로 함께 남긴다.
    """
    _rc = _robot_cfg()
    MAX_WHEEL, WHEEL_RADIUS, WHEEL_SEP = _rc.MAX_WHEEL, _rc.WHEEL_RADIUS, _rc.WHEEL_SEP

    def wheel_cmds(r):
        wl = (r["lin_raw"] - r["ang_raw"] * WHEEL_SEP / 2.0) / WHEEL_RADIUS
        wr = (r["lin_raw"] + r["ang_raw"] * WHEEL_SEP / 2.0) / WHEEL_RADIUS
        peak = max(abs(wl), abs(wr))
        sat = peak > MAX_WHEEL
        if sat:
            k = MAX_WHEEL / peak
            wl, wr = wl * k, wr * k
        return wl, wr, sat

    cmds = [wheel_cmds(r) for r in real]
    active = [max(abs(a), abs(b)) > 0.05 for a, b, _ in cmds]

    runs, i = [], 0
    while i < len(real):
        if not active[i]:
            i += 1
            continue
        start = i
        while i < len(real) and active[i]:
            i += 1
        runs.append((start, i))

    samples = []
    for start, end in runs:
        run = real[start:end]
        if len(run) < 5:
            continue
        half = len(run) // 2
        steady = run[half:]
        n = len(steady)
        cmd_l = sum(cmds[k][0] for k in range(start + half, end)) / n
        cmd_r = sum(cmds[k][1] for k in range(start + half, end)) / n
        cmd_v = sum(r["lin_raw"] for r in steady) / n
        cmd_w = sum(r["ang_raw"] for r in steady) / n
        ss_l = sum(r["wheel_l_vel"] for r in steady) / n
        ss_r = sum(r["wheel_r_vel"] for r in steady) / n
        sat = cmds[start][2]
        peak_cmd = max(abs(cmd_l), abs(cmd_r))
        if sat:
            kind = "saturated"
        elif peak_cmd < 1.0:
            kind = "low"
        elif min(abs(cmd_l), abs(cmd_r)) < 0.2 * peak_cmd:
            kind = "single"
        elif abs(cmd_w) > 0.2:
            kind = "turn"
        else:
            kind = "straight"
        t_step = run[0]["t"]
        ages = [r["t_ros"] - r["t_joint"] for r in run
                if isinstance(r.get("t_joint"), float) and r["t_joint"] == r["t_joint"]
                and isinstance(r.get("t_ros"), float) and r["t_ros"] == r["t_ros"]]
        age = sorted(ages)[len(ages) // 2] if ages else 0.0

        for side, cmd, ss, key in (("L", cmd_l, ss_l, "wheel_l_vel"),
                                   ("R", cmd_r, ss_r, "wheel_r_vel")):
            if abs(cmd) < 0.02:
                continue
            t10 = t90 = tau = None
            # 정지에서 출발한 계단만. 10/90%는 명령이 아니라 정상상태(ss) 기준 —
            # cmd 기준이면 gain≠1일 때 tau가 틀어지고, gain<0.9면 90%에 영영 못 닿는다.
            if abs(run[0][key]) < 0.25 * abs(cmd) and ss * cmd > 0 and abs(ss) > 0.05:
                goal10, goal90 = 0.1 * ss, 0.9 * ss

                def cross(goal):
                    for r in run:
                        v = r[key]
                        if (ss > 0 and v >= goal) or (ss < 0 and v <= goal):
                            return r["t"] - t_step
                    return None

                t10, t90 = cross(goal10), cross(goal90)
                if t10 is not None and t90 is not None:
                    tau = max(0.01, min(0.3, (t90 - t10) / 2.2))
            samples.append({
                "kind": kind,
                "wheel": side,
                "cmd_v": round(cmd_v, 4),
                "cmd_w": round(cmd_w, 4),
                "cmd": round(cmd, 4),
                "wheel_ss": round(ss, 4),
                "gain": round(ss / cmd, 4),
                "t10": None if t10 is None else round(t10, 3),
                "tau": None if tau is None else round(tau, 4),
                "sat": sat,
                "age": round(age, 4),
            })

    if not samples:
        return None

    def median(vals):
        vals = sorted(vals)
        return vals[len(vals) // 2] if vals else None

    nominal = [s for s in samples
               if not s["sat"] and abs(s["cmd"]) >= 1.0 and s["gain"] is not None]
    if not nominal:
        nominal = [s for s in samples if s["gain"] is not None]
    if not nominal:
        return None

    gain = median([s["gain"] for s in nominal])
    gain_l = median([s["gain"] for s in nominal if s["wheel"] == "L"])
    gain_r = median([s["gain"] for s in nominal if s["wheel"] == "R"])
    delays = [max(0.0, s["t10"] - s["age"] - 0.105 * s["tau"])
              for s in nominal if s["t10"] is not None and s["tau"] is not None]
    taus = [s["tau"] for s in nominal if s["tau"] is not None]
    delay = median(delays) if delays else 0.0
    tau = median(taus) if taus else 0.05

    def deadband_of(side):
        rows = [s for s in samples if s["wheel"] == side and abs(s["cmd"]) < 1.0]

        def responds(s):
            return abs(s["wheel_ss"]) >= max(0.05, 0.3 * abs(s["cmd"]))

        dead = [abs(s["cmd"]) for s in rows if not responds(s)]
        live = [abs(s["cmd"]) for s in rows if responds(s)]
        if dead and live:
            return 0.5 * (max(dead) + min(live))
        if dead:
            return max(dead)
        return 0.0

    db_l, db_r = deadband_of("L"), deadband_of("R")

    def kind_gain(kinds, side, vmin=0.0):
        vals = [s["gain"] for s in samples
                if s["kind"] in kinds and s["wheel"] == side
                and s["gain"] is not None and abs(s["cmd_v"]) >= vmin]
        return None if not vals else round(sum(vals) / len(vals), 4)

    wheel_max = max(max(abs(r["wheel_l_vel"]), abs(r["wheel_r_vel"])) for r in real)
    return {
        "wheel_gain": round(max(0.8, min(1.2, gain)), 4),
        "wheel_gain_l": round(max(0.8, min(1.2, gain_l)), 4),
        "wheel_gain_r": round(max(0.8, min(1.2, gain_r)), 4),
        "wheel_delay_s": round(delay, 3),
        "wheel_delay_steps": int(round(delay * CONTROL_HZ)),
        "wheel_tau_s": round(tau, 4),
        "wheel_deadband": round(max(0.0, min(0.3, 0.5 * (db_l + db_r))), 4),
        "wheel_deadband_l": round(max(0.0, min(0.3, db_l)), 4),
        "wheel_deadband_r": round(max(0.0, min(0.3, db_r)), 4),
        "wheel_max_obs": round(wheel_max, 3),
        "saturated_samples": sum(1 for s in samples if s["sat"]),
        "power": {
            "straight_l": kind_gain(("straight",), "L", 0.25),
            "straight_r": kind_gain(("straight",), "R", 0.25),
            "turn_l": kind_gain(("turn", "saturated"), "L", 0.25),
            "turn_r": kind_gain(("turn", "saturated"), "R", 0.25),
        },
        "speed_fit": sorted(samples, key=lambda s: (s["cmd_v"], s["wheel"])),
    }


def tag_of(path: str) -> str:
    stem = Path(path).stem
    for prefix in ("sim_sensor_", "real_sensor_", "sim_", "real_"):
        if stem.startswith(prefix):
            return stem[len(prefix):] or "run"
    return stem or "run"


def run_label(path: str) -> str:
    """그래프 제목용 이름. `<run>/raw.csv`면 run 디렉터리명을 쓴다."""
    p = Path(path)
    if p.stem in ("raw", "data", "log") and p.parent.name not in ("", "artifacts", "calibration"):
        return p.parent.name
    return tag_of(path)


def compare(sim_path: str, real_path: str, out=None, calib_out=None, show: bool = False):
    sim, real = load(sim_path), load(real_path)
    ms, mr = metrics(sim), metrics(real)
    fit = fit_wheel(real)

    tag = tag_of(sim_path) if tag_of(sim_path) != "run" else tag_of(real_path)
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
    has_wheels = "wheel_l_vel" in sim[0] and "wheel_l_vel" in real[0]
    if has_wheels:
        _, wl_s = series(sim, "wheel_l_vel")
        _, wr_s = series(sim, "wheel_r_vel")
        _, wl_r = series(real, "wheel_l_vel")
        _, wr_r = series(real, "wheel_r_vel")

    has_path = all(k in sim[0] and k in real[0] for k in ("odom_x", "odom_y"))
    has_stamp = all(k in sim[0] and k in real[0]
                    for k in ("t_ros", "t_odom", "t_imu", "t_joint"))
    n_time = 6 if has_wheels else 4
    if has_stamp:
        n_time += 1
    ax_path = None
    if has_path:
        fig = plt.figure(figsize=(11, 14.5))
        gs = fig.add_gridspec(n_time + 1, 1, height_ratios=[1.7] + [1] * n_time)
        ax_path = fig.add_subplot(gs[0])
        ax = [fig.add_subplot(gs[i + 1]) for i in range(n_time)]
        for a in ax[1:]:
            a.sharex(ax[0])
    else:
        fig, axs = plt.subplots(n_time, 1, figsize=(11, 12 if has_wheels else 10),
                                sharex=True)
        ax = list(axs)
    for a in ax:
        a.grid(True, alpha=0.3)

    if ax_path is not None:
        sx, sy = path(sim)
        rx, ry = path(real)
        cx, cy = cmd_path(real, t_r)
        ax_path.plot(cx, cy, color="tab:green", ls=":", lw=1.8, label="명령 100% 궤적 (이상)")
        ax_path.plot(sx, sy, color="tab:blue", lw=1.6, label="sim odom")
        ax_path.plot(rx, ry, color="tab:orange", lw=1.6, label=f"real odom")
        if fit:
            fx, fy = fit_path(real, t_r, fit)
            ax_path.plot(fx, fy, color="tab:red", ls="--", lw=1.2, label="fit model 피팅모델 (실제 재현)")
            qx, qy = corrected_path(real, t_r, fit)
            ax_path.plot(qx, qy, color="magenta", lw=3.2, alpha=0.35,
                         label="보정 후 예상 궤적 (gain 역보상, 지연 남음)")
            me = mean_dev(rx, ry, cx, cy)
            mc = mean_dev(qx, qy, cx, cy)
            ax_path.text(0.02, 0.02,
                         f"이상 대비 평균 궤적오차: real {me:.3f} m → 보정 후 {mc:.3f} m",
                         transform=ax_path.transAxes, fontsize=8, color="tab:purple",
                         bbox=dict(boxstyle="round", facecolor="white", alpha=0.7))
        ax_path.plot(sx[0], sy[0], "ko", ms=5, label="start")
        ax_path.plot(sx[-1], sy[-1], "b^", ms=8, label="sim end")
        ax_path.plot(rx[-1], ry[-1], "r.", ms=8, label="real end")
        ax_path.axvline(0, color="k", lw=0.5)
        ax_path.axhline(0, color="k", lw=0.5)
        ax_path.set_aspect("equal", adjustable="datalim")
        ax_path.set_xlabel("x 위치 [m]")
        ax_path.set_ylabel("y 위치 [m]")
        ax_path.set_title("odom 주행 궤적 — 사각 → 180° 턴 → 역주행",
                          fontsize=10, pad=3)
        ax_path.legend(loc="best", fontsize=8)
        ax_path.grid(True, alpha=0.3)

    ax[0].plot(t_s, cmd_v_s, "k--", lw=1.2, label="cmd 명령 (lin_raw)")
    ax[0].plot(t_s, v_s, color="tab:blue", label=f"sim v_enc 엔코더 (rms {ms['rms_v']:.3f})")
    ax[0].plot(t_r, v_r, color="tab:orange", label=f"real v_enc 엔코더 (rms {mr['rms_v']:.3f})")
    fit_v_r = fit_v(real, t_r, fit) if fit else None
    if fit_v_r is not None:
        ax[0].plot(t_r, fit_v_r, color="tab:red", ls="--", lw=1.2,
                   label=f"fit model 피팅모델 (g L/R={_gain(fit, 'wheel_gain_l'):.3f}/{_gain(fit, 'wheel_gain_r'):.3f})")
    ax[0].set_ylabel("직진 속도 v [m/s]\n(몸체 선속도)")
    ax[0].legend(loc="lower right", fontsize=8)

    ax[1].plot(t_s, cmd_w_s, "k--", lw=1.2, label="cmd 명령 (ang_raw)")
    ax[1].plot(t_s, w_s, color="tab:blue", label=f"sim w_gyro 자이로 (rms {ms['rms_w']:.3f})")
    ax[1].plot(t_r, w_r, color="tab:orange", label=f"real w_gyro 자이로 (rms {mr['rms_w']:.3f})")
    ax[1].plot(t_s, we_s, color="tab:blue", ls=":", lw=1.2, label=f"sim w_enc 엔코더 (rms {ms['rms_w_enc']:.3f})")
    ax[1].plot(t_r, we_r, color="tab:orange", ls=":", lw=1.2, label=f"real w_enc 엔코더 (rms {mr['rms_w_enc']:.3f})")
    fit_w_r = fit_ang(real, t_r, fit) if fit else None
    if fit_w_r is not None:
        ax[1].plot(t_r, fit_w_r, color="tab:red", ls="--", lw=1.2, label="fit model 피팅모델 (바퀴→각속도)")
    ax[1].set_ylabel("회전 속도 w [rad/s]\n(몸체 각속도)")
    ax[1].legend(loc="lower right", fontsize=7)
    ax[1].text(0.01, 0.92, "부호 반전 -> IMU 축/방향 설정 문제", transform=ax[1].transAxes,
               fontsize=8, color="tab:red")

    ev_s = [c - v for c, v in zip(cmd_v_s, v_s)]
    ev_r = [c - v for c, v in zip(cmd_v_r, v_r)]
    ax[2].plot(t_s, ev_s, color="tab:blue", label=f"sim err 직진오차 (bias {ms['bias_v']:+.3f})")
    ax[2].plot(t_r, ev_r, color="tab:orange", label=f"real err 직진오차 (bias {mr['bias_v']:+.3f})")
    if fit_v_r is not None:
        ax[2].plot(t_r, [c - f for c, f in zip(cmd_v_r, fit_v_r)],
                   color="tab:red", ls=":", lw=1.2, label="직진 fit 잔차 (모델로 설명 안 되는 부분)")
    ax[2].axhline(0, color="k", lw=0.6)
    ax[2].set_ylabel("직진 추종오차 [m/s]\n(명령 − 엔코더)")
    ax[2].legend(loc="lower right", fontsize=8)

    ew_s = [c - w for c, w in zip(cmd_w_s, w_s)]
    ew_r = [c - w for c, w in zip(cmd_w_r, w_r)]
    ax[3].plot(t_s, ew_s, color="tab:blue", label=f"sim err 회전오차 (bias {ms['bias_w']:+.3f})")
    ax[3].plot(t_r, ew_r, color="tab:orange", label=f"real err 회전오차 (bias {mr['bias_w']:+.3f})")
    if fit_w_r is not None:
        ax[3].plot(t_r, [c - f for c, f in zip(cmd_w_r, fit_w_r)],
                   color="tab:brown", ls=":", lw=1.2, label="회전 fit 잔차 (모델로 설명 안 되는 부분)")
    ax[3].axhline(0, color="k", lw=0.6)
    ax[3].set_ylabel("회전 추종오차 [rad/s]\n(명령 − 자이로)")
    ax[3].legend(loc="lower right", fontsize=8)
    if has_wheels:
        fwl = fwr = None
        if fit:
            fwl, fwr = fit_wheels(real, t_r, fit)
        ax[4].plot(t_s, wl_s, color="tab:blue", label="sim 왼쪽 바퀴 (wheel L)")
        ax[4].plot(t_r, wl_r, color="tab:orange", ls="--", label="real 왼쪽 바퀴 (wheel L)")
        if fwl is not None:
            ax[4].plot(t_r, fwl, color="tab:green", ls=":", lw=1.5, label="fit 왼쪽바퀴")
        ax[4].axhline(0, color="k", lw=0.6)
        ax[4].set_ylabel("왼쪽 바퀴 각속도 [rad/s]")
        ax[4].legend(loc="lower right", fontsize=7)
        ax[5].plot(t_s, wr_s, color="tab:cyan", label="sim 오른쪽 바퀴 (wheel R)")
        ax[5].plot(t_r, wr_r, color="tab:red", ls="--", label="real 오른쪽 바퀴 (wheel R)")
        if fwr is not None:
            ax[5].plot(t_r, fwr, color="tab:purple", ls=":", lw=1.5, label="fit 오른쪽바퀴")
        ax[5].axhline(0, color="k", lw=0.6)
        ax[5].set_ylabel("오른쪽 바퀴 각속도 [rad/s]")
        ax[5].legend(loc="lower right", fontsize=7)
    if has_stamp:
        a = ax[-1]
        _, s_ros = series(sim, "t_ros")
        _, r_ros = series(real, "t_ros")
        for key, label, color in (("t_odom", "odom", "tab:blue"),
                                  ("t_imu", "imu", "tab:green"),
                                  ("t_joint", "joint", "tab:purple")):
            _, s_msg = series(sim, key)
            _, r_msg = series(real, key)
            a.plot(t_s, [(x - y) * 1000.0 if y == y else float("nan") for x, y in zip(s_ros, s_msg)],
                   color=color, lw=1.0, label=f"sim {label}")
            a.plot(t_r, [(x - y) * 1000.0 if y == y else float("nan") for x, y in zip(r_ros, r_msg)],
                   color=color, ls="--", lw=1.0, label=f"real {label}")
        a.axhline(0, color="k", lw=0.6)
        a.set_ylabel("메시지 지연 [ms]\n(수신 − 발행)")
        a.legend(loc="lower right", fontsize=7, ncol=2)
    ax[-1].set_xlabel("시간 [s]")

    out = Path(out) if out else OUT_DIR / f"compare_{tag}.png"
    fig.suptitle(f"{run_label(sim_path)} vs {run_label(real_path)}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.985) if ax_path is not None else None)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"[plot] {out}")

    if fit:
        calib_path = Path(calib_out) if calib_out else OUT_DIR / f"sim_calib_{tag}.json"
        old = {}
        try:
            old = json.loads(calib_path.read_text())
        except Exception:
            pass
        old.update(fit)
        old["condition"] = tag
        old["source"] = Path(real_path).name
        old["fitted_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        calib_path.parent.mkdir(parents=True, exist_ok=True)
        calib_path.write_text(json.dumps(old, indent=2, ensure_ascii=False) + "\n")
        print(f"[calib] {calib_path} · runs={len(fit.get('speed_fit', []))} "
              f"gain L/R={fit['wheel_gain_l']}/{fit['wheel_gain_r']} delay={fit['wheel_delay_s']}s "
              f"tau={fit['wheel_tau_s']}s")

    if show:
        if matplotlib.get_backend().lower() == "agg":
            import subprocess
            subprocess.run(["xdg-open", str(out)], check=False)
        else:
            plt.show()
    print(f"{'metric':12s}{'sim':>10s}{'real':>10s}{'diff':>10s}")
    for k in ("rms_v", "rms_w", "rms_w_enc", "bias_v", "bias_w", "bias_w_enc", "p95_v", "p95_w"):
        b, r = ms[k], mr[k]
        print(f"{k:12s}{b:10.4f}{r:10.4f}{r - b:+10.4f}")
    return {"metrics": (ms, mr), "fit": fit, "out": str(out)}


def main(argv=None):
    p = argparse.ArgumentParser(description="run 리포트 (CSV 1개) / sim-vs-real 비교 (CSV 2개)")
    p.add_argument("csv", nargs="+", help="1개: 단건 리포트 / 2개: sim real 비교")
    p.add_argument("--out", default=None, help="단건: 기본 <csv와 같은 폴더>/report.png · 비교: 기본 artifacts/compare_<tag>.png")
    p.add_argument("--show", action="store_true", help="저장 후 창으로 띄우기")
    p.add_argument("--calib-out", default=None,
                   help="비교 전용: 기본 artifacts/sim_calib_<tag>.json (random_world가 읽음)")
    args = p.parse_args(argv)
    if len(args.csv) == 1:
        rows = load(args.csv[0])
        fit = fit_wheel(rows)
        if fit is not None:
            print(f"fit: gain L/R={fit['wheel_gain_l']}/{fit['wheel_gain_r']} "
                  f"delay={fit['wheel_delay_steps']} tau={fit['wheel_tau_s']}")
        else:
            print("fit 스킵 (전진 구간 없음)")
        write_report_png(args.csv[0], args.out, fit=fit)
    elif len(args.csv) == 2:
        compare(args.csv[0], args.csv[1], args.out, args.calib_out, args.show)
    else:
        p.error("CSV 1개(단건) 또는 2개(sim real) 지정")


if __name__ == "__main__":
    main()
