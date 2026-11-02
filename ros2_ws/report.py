#!/usr/bin/env python3
"""rl-only 기록(eval_recorder CSV) → report.png. ROS 노드가 아니다 — 파일만 읽는다.
그림 형식은 isaac calibration 리포트(calibration/report/report.py write_report_png)와 맞춘다.

  ./report.py                                          # artifacts/real/raw.csv → artifacts/real/report.png
  ./report.py artifacts/real/raw.csv [--show]
  ./report.py before.csv after.csv [--out compare.png]  # 여러 개면 겹쳐 그림 (선 모양으로 구분)
  한 CSV 안에서 policy.sh로 A/B를 바꿨으면(rl_policy 칸) 구간마다 따로 겹쳐 그리고 지표도 구간별로 낸다.

패널 (CSV에 없는 칸의 패널은 건너뛴다 — 예전 12칸 CSV도 됨):
  odom 주행 궤적: 명령 100%(이상) · odom 실측(보정함) · 보정 안 했다면(추정) · 시작/끝
  · 직진 속도 · 회전 속도 · 추종 오차 · 보정량 왼쪽/오른쪽 바퀴 (0에 가까울수록 보정 없음)
  · 왼쪽/오른쪽 바퀴 각속도 · 모터 전류 · 모터 온도+배터리 · IMU 가속도 · 방향(엔코더 vs IMU) · 지연
  비상정지(state_estop=1) 구간은 빨강, 직결(state_rl=0) 구간은 회색 배경.
보정 안 했다면(추정): 이 기록에서 바퀴별 gain(= 실제 바퀴 속도 / 보정 출력의 바퀴 명령, 중앙값)을 뽑아
  입력 명령(lin/ang_raw)을 그 gain으로 재생해 적분한 궤적. 진짜 Before는 enable:=false로 따로 기록해 같이 넣는다.
지표 (터미널 출력 · 그림 우상단 요약): 직진/회전 추종 RMS · 평균 바퀴 보정량 · 바퀴 gain(추정)
  · 응답 지연 평균/p95 · 최대 전류·온도 · 배터리 강하 · 비상정지 행 수 (정지 명령 행 제외).
필요: matplotlib (없으면 isaac/.venv/bin/python ./report.py ...)
"""
import argparse
import csv
import math
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


def metrics(rows: list[dict]) -> dict:
    moving = [r for r in rows if r["lin_raw"] == r["lin_raw"] and (r["lin_raw"] != 0.0 or r["ang_raw"] != 0.0)]

    def rms(a, b):
        d = finite([r[a] - r[b] for r in moving if a in r and b in r])
        return math.sqrt(sum(x * x for x in d) / len(d)) if d else NAN

    def col(k):
        return finite([r[k] for r in rows if k in r])

    def mean_abs(xs):
        xs = finite(xs)
        return sum(map(abs, xs)) / len(xs) if xs else NAN

    cl, cr = wheel_corr(moving)
    lat = sorted(finite([(r["t_out"] - r["t_abs"]) * 1000 for r in rows if "t_out" in r]))
    cur = col("wheel_l_cur") + col("wheel_r_cur")
    temp = col("motor_l_temp") + col("motor_r_temp")
    bat = col("battery_v")
    g = wheel_gains(rows)
    return {"움직인 행 수": len(moving),
            "직진 추종 RMS [m/s]": rms("lin_raw", "v_enc"),
            "회전 추종 RMS-자이로 [rad/s]": rms("ang_raw", "w_gyro"),
            "회전 추종 RMS-엔코더 [rad/s]": rms("ang_raw", "w_enc"),
            "평균 왼쪽 바퀴 보정 [rad/s]": mean_abs(cl),
            "평균 오른쪽 바퀴 보정 [rad/s]": mean_abs(cr),
            "왼쪽 바퀴 gain (추정)": g[0] if g else NAN,
            "오른쪽 바퀴 gain (추정)": g[1] if g else NAN,
            "응답 지연 평균 [ms]": sum(lat) / len(lat) if lat else NAN,
            "응답 지연 p95 [ms]": lat[int(0.95 * (len(lat) - 1))] if lat else NAN,
            "최대 전류 [A]": max(cur) if cur else NAN,
            "최대 온도 [°C]": max(temp) if temp else NAN,
            "배터리 강하 [V]": (bat[0] - bat[-1]) if bat else NAN,
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
    """어느 run에든 그 칸들에 값이 하나라도 있으면 패널을 그린다. runs = [(이름, 짧은 이름, 행, 전체 행)]."""
    return any(all(finite([r.get(k, NAN) for r in run[2]]) for k in keys) for run in runs)


def split_phases(path: Path) -> list:
    """CSV 하나 → [(이름, 짧은 이름, 행, 전체 행)]. rl_policy(A/B 구간)가 여럿이면 구간마다 하나씩.

    구간 행은 시간축을 그대로 두고, 다른 구간 자리는 nan으로 비운다 (그래프에서 그 구간만 보이게).
    """
    rows = load(path)
    phases = list(dict.fromkeys(r.get("rl_policy", "") for r in rows))
    if len(phases) <= 1:
        return [(f"{path.parent.name}/{path.name}", path.stem, rows, rows)]
    out = []
    for ph in phases:
        masked = [r if r.get("rl_policy") == ph else
                  {k: (v if k in ("t", "t_abs") else ("" if k in TEXT else NAN)) for k, v in r.items()}
                  for r in rows]
        out.append((f"{path.name} · {ph}", ph, masked, rows))
    return out


def shade(a, rows, key, value, color):
    """state 칸이 value인 연속 구간을 배경색으로."""
    start = None
    for r in rows + [{"t": rows[-1]["t"], key: None}]:
        on = r.get(key) == value
        if on and start is None:
            start = r["t"]
        elif not on and start is not None:
            a.axvspan(start, r["t"], color=color, alpha=0.12, lw=0)
            start = None


def fmt(v, nd=3) -> str:
    if isinstance(v, int):
        return str(v)
    return "—" if v != v else f"{v:.{nd}f}"


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
    multi = len(runs) > 1
    names = ["lin", "ang", "err"]
    names += ["corr_l", "corr_r"] if has(runs, "lin_out") else []
    names += ["wheel_l", "wheel_r"] if has(runs, "wheel_l_vel") else []
    names += ["cur"] if has(runs, "wheel_l_cur") else []
    names += ["temp"] if has(runs, "motor_l_temp") or has(runs, "battery_v") else []
    names += ["acc"] if has(runs, "imu_ax") else []
    names += ["yaw"] if has(runs, "imu_yaw") else []
    names += ["lat"] if has(runs, "t_out") or has(runs, "t_odom") else []

    fig = plt.figure(figsize=(11, 4 + 2 * len(names)))
    gs = fig.add_gridspec(len(names) + 1, 1, height_ratios=[1.7] + [1] * len(names))
    ax_path = fig.add_subplot(gs[0])
    axes = [fig.add_subplot(gs[i + 1]) for i in range(len(names))]
    for a in axes[1:]:
        a.sharex(axes[0])
    ax = dict(zip(names, axes))
    for a in [ax_path] + axes:
        a.grid(True, alpha=0.3)

    badges = []
    for i, (name, stem, rows, full) in enumerate(runs):
        ls = RUN_LS[i % len(RUN_LS)]
        tag = f"[{name}] " if multi else ""
        t = [r["t"] for r in rows]

        def col(k):
            return [r.get(k, NAN) for r in rows]

        def diff(a, b):
            return [x - y for x, y in zip(col(a), col(b))]

        # ---- odom 주행 궤적
        sx, sy = col("odom_x"), col("odom_y")
        cx, cy = integrate(rows, zip(col("lin_raw"), col("ang_raw")))
        ax_path.plot(cx, cy, color="tab:green", ls=":", lw=1.8, label=f"{tag}명령 100% 궤적 (이상)")
        bypass = stem == "bypass"   # policy.sh off 구간 — 이미 보정 없음이라 추정선은 그리지 않는다
        kind = "바이패스" if bypass else "보정함"
        ax_path.plot(sx, sy, color="tab:gray" if bypass else "tab:blue", lw=1.4, ls=ls,
                     label=f"{tag}odom 실측 궤적 ({kind})")
        badge = f"{tag}이상 대비 평균 궤적오차: {kind} {mean_dev(sx, sy, cx, cy):.3f} m"
        g = None if bypass else wheel_gains(rows)
        if g:
            ux, uy = uncorrected_path(rows, g)
            ax_path.plot(ux, uy, color="tab:red", ls="--", lw=1.2,
                         label=f"{tag}보정 안 했다면 (추정 · 바퀴 gain 왼/오 {g[0]:.3f}/{g[1]:.3f})")
            badge += f" · 보정 안 했다면(추정) {mean_dev(ux, uy, cx, cy):.3f} m"
        badges.append(badge)
        fx, fy = finite(sx), finite(sy)
        if fx:
            ax_path.plot(fx[0], fy[0], "ko", ms=5, label="시작" if i == 0 else None)
            ax_path.plot(fx[-1], fy[-1], "r.", ms=9, label="끝" if i == 0 else None)

        # ---- 직진 · 회전 · 추종 오차
        if i == 0:   # 입력은 모든 run이 같은 패턴이라는 가정 — 첫 번째 CSV 전체로
            ft = [r["t"] for r in full]
            ax["lin"].plot(ft, [r["lin_raw"] for r in full], "k--", lw=1.0, label="입력 명령 (lin_raw)")
            ax["ang"].plot(ft, [r["ang_raw"] for r in full], "k--", lw=1.0, label="입력 명령 (ang_raw)")
            for k in ("lin", "ang"):
                shade(ax[k], full, "state_estop", 1.0, "red")
                shade(ax[k], full, "state_rl", 0.0, "gray")
        ax["lin"].plot(t, col("lin_out"), color="tab:orange", lw=1.0, ls=ls, label=f"{tag}보정 출력 (lin_out)")
        if has([(name, stem, rows, full)], "cmd_base_lin"):
            ax["lin"].plot(t, col("cmd_base_lin"), color="tab:purple", lw=0.8, ls=":",
                           label=f"{tag}드라이버 명령 (cmd_vel_base)")
        ax["lin"].plot(t, col("v_enc"), color="tab:blue", lw=1.2, ls=ls, label=f"{tag}v_enc 엔코더 속도")
        ax["ang"].plot(t, col("ang_out"), color="tab:orange", lw=1.0, ls=ls, label=f"{tag}보정 출력 (ang_out)")
        ax["ang"].plot(t, col("w_gyro"), color="tab:blue", lw=1.2, ls=ls, label=f"{tag}w_gyro 자이로 실측")
        ax["ang"].plot(t, col("w_enc"), color="tab:blue", lw=1.0, ls=":", label=f"{tag}w_enc 엔코더 계산")
        ax["err"].plot(t, diff("lin_raw", "v_enc"), color="tab:blue", ls=ls, label=f"{tag}v_cmd - v_enc 직진 오차")
        ax["err"].plot(t, diff("ang_raw", "w_gyro"), color="tab:orange", ls=ls, label=f"{tag}w_cmd - w_gyro 회전 오차")

        # ---- 보정량: 바퀴별 (0 = 보정 없음)
        if "corr_l" in ax:
            al, ar = wheel_corr(rows)
            for k, color, vals in (("corr_l", "tab:blue", al), ("corr_r", "tab:cyan", ar)):
                ax[k].plot(t, vals, color=color, lw=1.2, ls=ls, label=f"{tag}실제 적용 보정 (보정 출력 - 입력)")
                if not multi:
                    ax[k].fill_between(t, 0, [v if v == v else 0 for v in vals], color=color, alpha=0.25)
            if has([(name, stem, rows, full)], "model_dv"):
                ml, mr = wheel_corr(rows, "model_dv", "model_dw")
                ax["corr_l"].plot(t, ml, color="tab:red", lw=0.7, ls="--", label=f"{tag}정책 원출력 (제한 전)")
                ax["corr_r"].plot(t, mr, color="tab:orange", lw=0.7, ls="--", label=f"{tag}정책 원출력 (제한 전)")

        # ---- 바퀴 · 모터 · IMU · 지연
        if "wheel_l" in ax:
            ax["wheel_l"].plot(t, col("wheel_l_vel"), color="tab:blue", ls=ls, label=f"{tag}실측 왼쪽 바퀴 (wheel L)")
            ax["wheel_r"].plot(t, col("wheel_r_vel"), color="tab:cyan", ls=ls, label=f"{tag}실측 오른쪽 바퀴 (wheel R)")
            bl = [wheel_cmd(r["lin_raw"], r["ang_raw"]) for r in rows]
            ax["wheel_l"].plot(t, [b[0] for b in bl], color="tab:red", ls="--", lw=0.9, label=f"{tag}입력 명령의 바퀴 속도")
            ax["wheel_r"].plot(t, [b[1] for b in bl], color="tab:orange", ls="--", lw=0.9, label=f"{tag}입력 명령의 바퀴 속도")
        if "cur" in ax:
            ax["cur"].plot(t, col("wheel_l_cur"), color="tab:blue", lw=1.0, ls=ls, label=f"{tag}왼쪽 모터")
            ax["cur"].plot(t, col("wheel_r_cur"), color="tab:cyan", lw=1.0, ls=ls, label=f"{tag}오른쪽 모터")
        if "temp" in ax:
            ax["temp"].plot(t, col("motor_l_temp"), color="tab:blue", ls=ls, label=f"{tag}왼쪽 모터 온도")
            ax["temp"].plot(t, col("motor_r_temp"), color="tab:cyan", ls=ls, label=f"{tag}오른쪽 모터 온도")
            if i == 0:
                ax["bat"] = ax["temp"].twinx()
                ax["bat"].set_ylabel("배터리 전압 [V]")
            ax["bat"].plot(t, col("battery_v"), color="tab:green", lw=1.0, ls=ls, label=f"{tag}배터리 전압")
        if "acc" in ax:
            ax["acc"].plot(t, col("imu_ax"), color="tab:blue", lw=0.8, ls=ls, label=f"{tag}앞뒤 가속도 (ax)")
            ax["acc"].plot(t, col("imu_ay"), color="tab:orange", lw=0.8, ls=ls, label=f"{tag}좌우 가속도 (ay)")
        if "yaw" in ax:
            ax["yaw"].plot(t, unwrap(col("odom_yaw")), color="tab:blue", lw=1.2, ls=ls, label=f"{tag}엔코더 방향 (odom yaw)")
            ax["yaw"].plot(t, unwrap(col("imu_yaw")), color="tab:green", lw=1.2, ls=ls, label=f"{tag}IMU 방향 (imu yaw)")
        if "lat" in ax:
            ms = lambda a_, b_: [(x - y) * 1000 for x, y in zip(col(a_), col(b_))]
            ax["lat"].plot(t, ms("t_out", "t_abs"), color="tab:orange", lw=1.2, ls=ls, label=f"{tag}corrector 응답")
            for k, name, c_ in (("t_odom", "odom", "tab:blue"), ("t_imu", "imu", "tab:green"),
                                ("t_joint", "joint", "tab:purple")):
                ax["lat"].plot(t, ms("t_abs", k), color=c_, lw=0.8, ls=ls, label=f"{tag}{name}")

    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.set_xlabel("x 위치 [m]")
    ax_path.set_ylabel("y 위치 [m]")
    ax_path.set_title("odom 주행 궤적")
    ax_path.legend(loc="best", fontsize=8)
    if badges:
        ax_path.text(0.02, 0.02, "\n".join(badges), transform=ax_path.transAxes, fontsize=8, color="tab:purple",
                     bbox=dict(boxstyle="round", facecolor="white", alpha=0.7))

    ylabels = {"lin": "직진 속도 v [m/s]\n(몸체 선속도)", "ang": "회전 속도 w [rad/s]\n(몸체 각속도)",
               "err": "추종 오차 [m/s 또는 rad/s]\n(명령 - 실측)",
               "corr_l": "왼쪽 바퀴 보정량 [rad/s]\n(0 = 보정 없음)", "corr_r": "오른쪽 바퀴 보정량 [rad/s]\n(0 = 보정 없음)",
               "wheel_l": "왼쪽 바퀴 각속도 [rad/s]", "wheel_r": "오른쪽 바퀴 각속도 [rad/s]",
               "cur": "모터 전류 [A]", "temp": "모터 온도 [°C]", "acc": "IMU 가속도 [m/s²]",
               "yaw": "방향 [rad]\n(차이 = 바퀴 슬립)", "lat": "지연 [ms]\n(수신 - 발행)"}
    for k, a in ax.items():
        if k == "bat":
            continue
        a.set_ylabel(ylabels[k], fontsize=9)
        a.legend(loc="lower left" if k == "temp" else "lower right", fontsize=8)
        if k not in ("temp", "yaw"):
            a.axhline(0, color="k", lw=0.6)
    if "bat" in ax:
        ax["bat"].legend(loc="lower right", fontsize=8)
    axes[-1].set_xlabel("시간 [s]")

    # ---- 제목 · 우상단 요약 · 터미널 지표
    ms = [metrics(run[2]) for run in runs]
    fig.suptitle("  vs  ".join(f"{p.parent.name}/{p.name}" for p in paths), fontsize=12, x=0.3, y=0.995)
    summary = []
    for (_, stem, _, _), m in zip(runs, ms):
        head = f"{stem}: " if multi else ""
        summary.append(f"{head}추종 RMS 직진 {fmt(m['직진 추종 RMS [m/s]'])} · 회전 {fmt(m['회전 추종 RMS-자이로 [rad/s]'])}"
                       f"  |  바퀴 gain 왼/오 {fmt(m['왼쪽 바퀴 gain (추정)'])}/{fmt(m['오른쪽 바퀴 gain (추정)'])}"
                       f"  |  응답 지연 {fmt(m['응답 지연 평균 [ms]'], 0)} ms")
    fig.text(0.99, 0.997, "\n".join(summary), ha="right", va="top", fontsize=9,
             bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))
    for k in ms[0]:
        print(f"{k}: " + "  ".join(f"{run[1]}={fmt(m[k], 4)}" for run, m in zip(runs, ms)))

    fig.tight_layout(rect=(0, 0, 1, 0.985))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
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
