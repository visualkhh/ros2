#!/usr/bin/env python3
"""학습 run 폴더 → report.html — scalar plot + 요약표 + snapshot gallery + config.

  ./run-training-report.sh artifacts/<task>/training/<시각>_<label>
  ./run-training-report.sh <task>          # artifacts/<task>/training/의 가장 최근 run
  → <run>/report.html (이미지 base64 embed — 파일 하나로 열림)

TensorBoard event는 학습 중에도 읽는다 (마지막으로 기록된 iteration까지).
"""
from __future__ import annotations

import argparse
import base64
import html
import io
import json
import math
from datetime import datetime
from pathlib import Path

try:
    import yaml
except ImportError:   # PyYAML 없는 환경이면 env.yaml 해석만 생략
    yaml = None

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager, rcParams  # noqa: E402
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator  # noqa: E402


def _set_korean_font() -> None:
    for name in ("Noto Sans CJK KR", "Noto Sans KR", "Noto Sans CJK JP", "Malgun Gothic", "DejaVu Sans"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            rcParams["font.family"] = name
            break
    rcParams["axes.unicode_minus"] = False


_set_korean_font()

ISAAC = Path(__file__).resolve().parents[1]

# 표/그래프 위에 먼저 보여 줄 지표
KEY_TAGS = (
    "Train/mean_reward",
    "Metrics/track_err_v",
    "Metrics/track_err_w",
    "Metrics/combined_err",
    "Metrics/cross_track",
    "Metrics/waypoints_done",
    "Metrics/collision_rate",
    "Metrics/goal_rate",
    "Train/mean_episode_length",
    "Loss/value",
    "Loss/learning_rate",
    "Policy/mean_std",
)

# tag → (한글 이름, 설명, 좋아지는 방향)
TAG_INFO = {
    "Train/mean_reward": {
        "label": "에피소드 전체 보상 합",
        "desc": "에피소드 보상의 합. 이 값만으로 좋아졌다고 판단하면 안 됨 — "
                "에피소드 길이(mean_episode_length)가 같이 변하기 때문.",
        "low": "더 음수가 됨. 에피소드가 길어져서 총합이 커진 것일 수 있음. "
               "mean_reward ÷ episode_length(스텝당)가 같이 내려가는지 확인.",
        "high": "덜 음수가 됨. 스텝당 보상이 올라갔거나 에피소드가 짧아진 것. "
                "track_err/성공률이 같이 좋아져야 진짜 개선.",
        "good": "스텝당 보상이 올라가고 track_err가 내려갈 때",
    },
    "Metrics/track_err_v": {
        "label": "전진 속도 추종 오차 [m/s]",
        "desc": "몸체 전진 속도가 명령(정책 보정 후)을 얼마나 못 따라갔는지. 이 task의 실제 목표.",
        "low": "0에 가까워짐 → 명령을 잘 따라감 → 좋음. action_scale_lin(현재 0.05)보다 작아지면 "
               "정책 보정이 충분히 먹는 범위.",
        "high": "명령을 못 따라감(지연·슬립·포화). action_scale_lin보다 크면 정책 출력이 포화됐을 수 있고, "
                "max_lin(0.26)에 가까우면 심각.",
        "good": "작을수록 좋음 · 50~100 iteration 동안 우하향",
    },
    "Metrics/track_err_w": {
        "label": "회전 속도 추종 오차 [rad/s]",
        "desc": "몸체 회전 속도가 명령(정책 보정 후)을 얼마나 못 따라갔는지. 이 task의 실제 목표.",
        "low": "0에 가까워짐 → 회전 명령을 잘 따라감 → 좋음. action_scale_ang(현재 0.10)보다 작아지면 "
               "정책 보정이 충분한 범위.",
        "high": "회전이 처지거나 과함(지연·슬립·포화). action_scale_ang보다 크면 정책 출력이 포화됐을 수 있고, "
                "max_ang(1.82)에 가까우면 심각.",
        "good": "작을수록 좋음 · 50~100 iteration 동안 우하향",
    },
    "Metrics/combined_err": {
        "label": "종합 추종 오차 [0~1]",
        "desc": "0.5 × (|v오차|/max_lin + |w오차|/max_ang). v·w를 한 숫자로 합친 이 task의 대표 지표.",
        "low": "0%에 가까워짐 → 전진·회전 둘 다 명령을 잘 따라감 → 좋음. 0%가 완벽.",
        "high": "100%에 가까워짐 → 최대 속도 오차 수준으로 못 따라감. 10% 이상이면 개선 여지가 큼.",
        "good": "작을수록 좋음 · 리포트 맨 위 종합 그래프가 우하향",
    },
    "Metrics/cross_track": {
        "label": "경로 이탈 [m]",
        "desc": "웨이포인트 경로(앞 점 → 다음 점 직선)에서 벗어난 평균 거리 (rl_maze).",
        "low": "0에 가까워짐 → 경로에 딱 붙어 감 → 좋음. 0.1 m 이하면 좋은 편.",
        "high": "경로에서 멀어짐 → 벽/장애물 쪽으로 감. fail_cross(1.0 m)에 가까우면 실패 종료.",
        "good": "작을수록 좋음",
    },
    "Metrics/waypoints_done": {
        "label": "도착 웨이포인트 수",
        "desc": "에피소드에서 통과한 웨이포인트 수 (rl_maze).",
        "low": "0에 가까우면 출발점 근처에서 못 감 → 나쁨.",
        "high": "num_waypoints(4)에 가까워질수록 끝까지 간 것 → 좋음.",
        "good": "클수록 좋음",
    },
    "Metrics/collision_rate": {
        "label": "충돌 비율",
        "desc": "벽/장애물 충돌로 끝난 비율.",
        "low": "0에 가까울수록 충돌 없이 주행 → 좋음.",
        "high": "충돌이 많음 → 나쁨. 경로/라이다/속도 확인.",
        "good": "작을수록 좋음",
    },
    "Metrics/goal_rate": {
        "label": "최종 goal 도달 비율",
        "desc": "마지막 웨이포인트까지 간 비율.",
        "low": "0에 가까우면 최종 goal을 못 감 → 나쁨.",
        "high": "1에 가까울수록 끝까지 도달 → 좋음.",
        "good": "클수록 좋음",
    },
    "Train/mean_episode_length": {
        "label": "평균 에피소드 길이 [step]",
        "desc": "에피소드가 몇 step 만에 끝났는지. 최대 길이(20 s × 20 Hz = 400)에 붙는지 확인.",
        "low": "빨리 끝남. 성공/실패/충돌 중 무엇 때문인지 확인해야 함. 지나치게 낮으면 학습 불안정.",
        "high": "끝까지 버팀. 최대(400)에 붙었는데 track_err/성공률이 안 좋으면 '버티기'만 배운 것. "
                "최대 = 시간 초과.",
        "good": "최대에 붙어도 track_err/성공률이 같이 좋아져야 함",
    },
    "Loss/value": {
        "label": "critic value loss",
        "desc": "가치 함수가 return을 얼마나 못 맞추는지.",
        "low": "critic이 return을 잘 맞춤 → 좋음. 학습이 안정적.",
        "high": "reward scale이 크거나 critic이 return을 못 맞추는 중. 수백~수천이면 "
                "reward 가중치/normalization을 확인.",
        "good": "작을수록 좋음",
    },
    "Loss/surrogate": {
        "label": "PPO policy loss",
        "desc": "정책 업데이트 크기(클립 전/후).",
        "low": "0 근처 = 정책 업데이트가 작음. KL/clip 상태 확인. 학습이 정체됐을 수 있음.",
        "high": "업데이트가 큼 → 불안정. clip_param/learning_rate 확인.",
        "good": "0 근처에서 안정",
    },
    "Loss/entropy": {
        "label": "정책 엔트로피",
        "desc": "정책 분포의 탐색량.",
        "low": "탐색이 줄어듦. 0에 가까우면 탐색 붕괴 → 정책이 굳음.",
        "high": "탐색이 많음. 초기엔 정상. 너무 오래 높으면 보상 신호가 약하거나 학습이 안 되는 중.",
        "good": "서서히 감소 (대략 0.5~1.5)",
    },
    "Loss/learning_rate": {
        "label": "학습률 (adaptive면 자동 조절)",
        "desc": "PPO 학습률. adaptive schedule이면 KL 보고 자동으로 바뀜.",
        "low": "안정적. 너무 낮으면 학습이 느림.",
        "high": "발산 위험. adaptive가 0.01 max에 붙으면 위험 신호. fixed 3e-4 권장.",
        "good": "낮게 유지 (고정 시 3e-4 근처)",
    },
    "Policy/mean_std": {
        "label": "행동 표준편차",
        "desc": "정책 행동의 탐색 노이즈.",
        "low": "노이즈가 작음. 0.1 이하면 탐색 붕괴/확신 과다 → 새 상황에 약함.",
        "high": "노이즈가 큼. 학습 초기엔 정상. 너무 오래 높으면 보상 신호가 약함.",
        "good": "0.2~0.5 근처",
    },
    "Perf/total_fps": {
        "label": "학습 처리량 [step/s]",
        "desc": "초당 환경 스텝 수. 정책 성능이 아니라 학습 속도.",
        "low": "느림. env 수/렌더링/CPU 확인.",
        "high": "빠름. 좋지만 정책 성능과는 무관.",
        "good": "참고",
    },
    "Perf/collection_time": {
        "label": "수집 시간 [s/iter]",
        "desc": "한 iteration에서 환경 스텝을 모으는 시간.",
        "low": "수집이 빠름.",
        "high": "수집이 느림. env 수/렌더링 확인.",
        "good": "참고",
    },
    "Perf/learning_time": {
        "label": "학습 시간 [s/iter]",
        "desc": "PPO 업데이트 시간.",
        "low": "업데이트가 빠름.",
        "high": "업데이트가 느림. 네트워크/배치 확인.",
        "good": "참고",
    },
}


def tag_info(tag: str) -> dict:
    if tag in TAG_INFO:
        return TAG_INFO[tag]
    if tag.startswith("Metrics/"):
        return {"label": "task 지표", "desc": "task가 기록한 scalar. task README와 태그 이름을 참고.",
                "low": "낮아지면 task에 따라 좋을 수도/나쁠 수도 있음 — task README 확인.",
                "high": "높아지면 task에 따라 좋을 수도/나쁠 수도 있음 — task README 확인.",
                "good": "task에 따라"}
    if tag.startswith("Loss/"):
        return {"label": "loss", "desc": "학습 loss.", "low": "낮아지면 좋음.",
                "high": "높아지면 학습 불안정/정체 가능.", "good": "작을수록 좋음"}
    if tag.startswith("Train/"):
        return {"label": "학습 지표", "desc": "학습 중 집계된 값.", "low": "task/지표에 따라.",
                "high": "task/지표에 따라.", "good": "task에 따라"}
    if tag.startswith("Policy/"):
        return {"label": "정책 통계", "desc": "정책 분포/노이즈 통계.", "low": "너무 낮으면 탐색 붕괴 가능.",
                "high": "너무 높으면 학습 불안정 가능.", "good": "안정적"}
    if tag.startswith("Perf/"):
        return {"label": "성능", "desc": "처리량/시간.", "low": "참고.", "high": "참고.", "good": "참고"}
    return {"label": "scalar", "desc": "설명 없음 — 태그 이름을 참고.", "low": "—", "high": "—", "good": "—"}


# 학습 설정 표의 한글 설명
SETUP_INFO = {
    "run name": "이 학습 run의 폴더 이름 (`<시작시각>_<label>`).",
    "task": "gym에 등록된 task 이름. env/로봇/보상 세트를 결정.",
    "created": "run이 시작된 시각 (`run.json`).",
    "envs": "동시에 학습하는 로봇(환경) 수. 클수록 rollout이 커지고 GPU 메모리를 더 씀.",
    "iterations": "PPO 업데이트 전체 횟수. `last`는 지금까지 돈 횟수, %는 진행률.",
    "rollout / iter": "1 iteration에서 모으는 스텝 수 = envs × num_steps_per_env. "
                      "PPO는 이 rollout을 모은 뒤 한 번 업데이트함.",
    "PPO epochs / iter": "같은 rollout을 PPO가 몇 번 반복 학습하는지. "
                         "크면 더 많이 쓰지만 과적합/불안정해질 수 있음.",
    "mini-batches / iter": "한 rollout을 몇 조각으로 나눠 SGD하는지.",
    "mini-batch size": "한 mini-batch의 스텝 수 = rollout ÷ mini-batches.",
    "total steps (planned)": "전체 학습 계획 스텝 수 = iterations × rollout.",
    "steps done": "지금까지 실제로 돈 스텝 수 ≈ last iteration × rollout.",
    "learning rate / schedule": "PPO 학습률과 스케줄. adaptive는 KL을 보고 자동 조절, fixed는 고정. "
                                "adaptive가 0.01 max에 붙으면 발산 위험.",
    "gamma / lam": "gamma = 미래 보상 할인율(0.99), lam = GAE λ(0.95). "
                   "클수록 먼 미래를 더 크게 봄.",
    "clip / entropy / desired_kl": "clip = PPO 정책 변화 제한(0.2), entropy = 탐색 보너스(0.005), "
                                   "desired_kl = adaptive LR이 목표로 하는 KL(0.01).",
    "max_grad_norm": "gradient clipping. 1.0 = gradient 폭주 방지.",
    "sim dt / decimation / policy dt": "물리 스텝 = 1/480 s, decimation 24 → 정책 20 Hz(0.05 s). "
                                       "시뮬은 480 Hz로 돌고 정책은 20 Hz로 결정.",
    "episode": "한 에피소드 길이. 20 s × 20 Hz = 400 step.",
    "action_scale (lin/ang)": "정책 출력 [-1, 1]에 곱하는 최대 보정량. "
                              "선속도 0.05 m/s · 각속도 0.10 rad/s. 배포 노드와 같아야 함.",
    "max_lin / max_ang": "명령/로봇의 최대 선·각속도. 보정 후 명령을 이 범위로 자름.",
    "network": "actor/critic MLP hidden 차원과 초기 행동 표준편차(init_std).",
    "seed / device": "랜덤 시드와 학습 장치(cuda:0).",
    "save_interval / logger": "체크포인트 저장 주기(iteration)와 로거.",
    "fps (last)": "마지막 iteration의 초당 스텝 수. 학습 속도이며 정책 성능은 아님.",
}


def find_run(arg: str) -> Path:
    p = Path(arg).expanduser()
    if p.is_file():
        if p.name.startswith("events.out.tfevents"):
            p = p.parent
        else:
            raise SystemExit(f"run 폴더나 event 파일이 아님: {p}")
    if not p.is_dir():
        p = ISAAC / "artifacts" / arg / "training"
    if not p.is_dir():
        raise SystemExit(f"경로 없음: {arg} (여기도 없음: {p})")
    if list(p.glob("events.out.tfevents.*")):
        return p
    runs = sorted((d for d in p.iterdir() if d.is_dir()), key=lambda d: d.stat().st_mtime, reverse=True)
    for run in runs:
        if list(run.glob("events.out.tfevents.*")):
            return run
    raise SystemExit(f"event 파일 없음: {p}")


def sort_tags(tags: list[str]) -> list[str]:
    def key(tag: str):
        if tag in KEY_TAGS:
            return (0, KEY_TAGS.index(tag), tag)
        for i, prefix in enumerate(("Metrics/", "Train/", "Loss/", "Policy/", "Perf/")):
            if tag.startswith(prefix):
                return (1, i, tag)
        return (2, 0, tag)
    return sorted(tags, key=key)


def plot_png(tag: str, ev) -> str:
    label = tag_info(tag)["label"]
    fig, ax = plt.subplots(figsize=(5.4, 2.7), dpi=110)
    ax.plot([e.step for e in ev], [e.value for e in ev], lw=1.2)
    ax.set_title(f"{tag}\n{label}", fontsize=9)
    ax.grid(alpha=0.3)
    ax.tick_params(labelsize=7)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def image_b64(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode()


def b64img(src: str, alt: str, cls: str = "") -> str:
    return f'<img class="{cls}" src="data:image/png;base64,{src}" alt="{html.escape(alt)}">'


def fmt(v: float) -> str:
    if v is None or not math.isfinite(float(v)):
        return "—"
    return f"{float(v):.5g}"


def combined_series(series: dict, env_cfg: dict) -> list[tuple[int, float]]:
    """종합 점수 (0~1, 낮을수록 좋음).
    1) Metrics/combined_err가 있으면 그대로.
    2) 추종형: 0.5*(|v|/max_lin + |w|/max_ang).
    3) 미로형: 0.5*((1-goal_rate) + collision_rate) — 도달 실패 + 충돌."""
    ev = series.get("Metrics/combined_err")
    if ev:
        return [(e.step, e.value) for e in ev]
    v = series.get("Metrics/track_err_v")
    w = series.get("Metrics/track_err_w")
    max_lin, max_ang = env_cfg.get("max_lin"), env_cfg.get("max_ang")
    if v and w and max_lin and max_ang:
        wm = {e.step: e.value for e in w}
        return [(e.step, 0.5 * (abs(e.value) / float(max_lin) + abs(wm[e.step]) / float(max_ang)))
                for e in v if e.step in wm]
    g = series.get("Metrics/goal_rate")
    c = series.get("Metrics/collision_rate")
    if g and c:
        cm = {e.step: e.value for e in c}
        return [(e.step, 0.5 * ((1.0 - e.value) + cm[e.step])) for e in g if e.step in cm]
    return []


def combined_kind(series: dict, env_cfg: dict) -> str:
    """종합 점수 종류: 'track' | 'maze' | '' (데이터 없음)."""
    if series.get("Metrics/combined_err"):
        return "track"
    v = series.get("Metrics/track_err_v")
    w = series.get("Metrics/track_err_w")
    if v and w and env_cfg.get("max_lin") and env_cfg.get("max_ang"):
        return "track"
    if series.get("Metrics/goal_rate") and series.get("Metrics/collision_rate"):
        return "maze"
    return ""


def combined_texts(kind: str) -> tuple[str, str, str, str]:
    """(card_title, plot_title, ylabel, caption)."""
    if kind == "maze":
        return (
            "종합 실패율 (낮을수록)",
            "종합 실패율 — 0.5·((1-도달률) + 충돌률) × 100 [%]  (낮을수록 좋음)",
            "종합 실패율 [%]",
            "도달 실패와 충돌을 한 숫자로 합친 값입니다. <b>0% = 매번 도달·무충돌</b>. "
            "낮을수록 좋고, 선이 오른쪽으로 갈수록 내려가면 학습이 좋아지는 중입니다.",
        )
    return (
        "종합 추종 오차 (낮을수록)",
        "종합 추종 오차 — 0.5·(|v오차|/max_lin + |w오차|/max_ang) × 100 [%]  (낮을수록 좋음)",
        "종합 오차 [%]",
        "v·w 추종 오차를 한 숫자로 합친 값입니다. <b>0% = 완벽, 100% = 최대 속도 오차</b>. "
        "낮을수록 좋고, 선이 오른쪽으로 갈수록 내려가면 학습이 좋아지는 중입니다.",
    )


def combined_png(pairs: list[tuple[int, float]], title: str = "",
                 ylabel: str = "") -> str:
    fig, ax = plt.subplots(figsize=(11.0, 3.1), dpi=115)
    xs = [p[0] for p in pairs]
    ys = [100.0 * p[1] for p in pairs]
    ax.plot(xs, ys, lw=2.0, color="#2563eb")
    ax.fill_between(xs, ys, 0, color="#2563eb", alpha=0.10)
    ax.set_title(title or "종합 — 낮을수록 좋음", fontsize=10)
    ax.set_xlabel("학습 반복", fontsize=8)
    ax.set_ylabel(ylabel or "종합 [%]", fontsize=8)
    ax.grid(alpha=0.3)
    ax.tick_params(labelsize=7)
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def per_step_reward(series: dict) -> list[tuple[int, float]]:
    """mean_reward / mean_episode_length — 에피소드 길이가 변해도 비교 가능한 스텝당 보상."""
    r = series.get("Train/mean_reward")
    length = series.get("Train/mean_episode_length")
    if not r or not length:
        return []
    lm = {e.step: e.value for e in length}
    return [(e.step, e.value / lm[e.step]) for e in r if e.step in lm and lm[e.step] > 0]


def verdict_items(series: dict, env_cfg: dict) -> list[tuple[str, str]]:
    """first/last 자동 비교 — 좋아짐/정체/나빠짐을 색으로. 참고용."""
    items: list[tuple[str, str]] = []

    def add(cls: str, text: str):
        items.append((cls, text))

    def trend(tag: str):
        ev = series.get(tag)
        if not ev or len(ev) < 2:
            return None
        return ev[0].value, ev[-1].value, min(x.value for x in ev), max(x.value for x in ev)

    def judge(first: float, last: float, lower_better: bool) -> tuple[str, str]:
        if lower_better:
            if last < first * 0.90:
                return "good", "좋아짐"
            if last > first * 1.05:
                return "bad", "나빠짐"
        else:
            if last > first * 1.10:
                return "good", "좋아짐"
            if last < first * 0.90:
                return "bad", "나빠짐"
        return "warn", "정체"

    for tag, lower in (("Metrics/track_err_v", True), ("Metrics/track_err_w", True),
                       ("Metrics/cross_track", True), ("Metrics/collision_rate", True),
                       ("Metrics/waypoints_done", False), ("Metrics/goal_rate", False)):
        t = trend(tag)
        if t is None:
            continue
        first, last, lo, hi = t
        cls, word = judge(first, last, lower)
        add(cls, f"{tag}: first {first:.4g} → last {last:.4g} (min {lo:.4g}, max {hi:.4g}) — {word}")
        if tag.startswith("Metrics/track_err"):
            scale = env_cfg.get("action_scale")
            if isinstance(scale, (list, tuple)) and len(scale) >= 2:
                lim = scale[1] if tag.endswith("_w") else scale[0]
                if last > float(lim) * 1.05:
                    kind = "ang" if tag.endswith("_w") else "lin"
                    add("warn", f"{tag} last {last:.3g} > action_scale_{kind} {float(lim):.3g} — "
                                f"정책 출력이 포화됐을 수 있음. action_scale 키우기 검토.")

    comb = combined_series(series, env_cfg)
    if len(comb) >= 2:
        kind = combined_kind(series, env_cfg)
        noun = "종합 실패율" if kind == "maze" else "종합 추종 오차"
        first, last = 100.0 * comb[0][1], 100.0 * comb[-1][1]
        cls = "good" if last < first * 0.95 else "bad" if last > first * 1.05 else "warn"
        word = "좋아짐" if cls == "good" else "나빠짐" if cls == "bad" else "정체"
        add(cls, f"{noun}: {first:.1f}% → {last:.1f}% — {word} "
                 f"(0=완벽{', 100%=전부 실패' if kind == 'maze' else ', 100%=최대 속도 오차'})")

    ps = per_step_reward(series)
    if len(ps) >= 2:
        first, last = ps[0][1], ps[-1][1]
        cls = "good" if last > first * 1.02 else "bad" if last < first * 0.98 else "warn"
        word = "좋아짐" if cls == "good" else "나빠짐" if cls == "bad" else "정체"
        add(cls, f"스텝당 보상 (mean_reward ÷ episode_length): {first:.4g} → {last:.4g} — {word}")

    length = series.get("Train/mean_episode_length")
    ep_s = env_cfg.get("episode_length_s")
    if length and ep_s:
        max_steps = round(float(ep_s) / 0.05)   # 정책 20 Hz
        if length[-1].value >= max_steps * 0.99:
            add("warn", f"에피소드 길이가 최대({max_steps} step)에 붙어 있음 — "
                        f"track_err가 안 줄면 '버티기'만 배운 것일 수 있음.")

    lr = series.get("Loss/learning_rate")
    if lr and lr[-1].value >= 0.009:
        add("warn", f"학습률이 {lr[-1].value:.4g} (adaptive max 근처) — 발산 위험. "
                    f"agent.algorithm.schedule=fixed, learning_rate=3e-4 검토.")

    val = series.get("Loss/value")
    if val and len(val) >= 2:
        cls = "good" if val[-1].value < val[0].value else "warn"
        add(cls, f"Loss/value: {val[0].value:.4g} → {val[-1].value:.4g} — "
                 f"{'감소' if cls == 'good' else '감소 안 함/증가'}")

    std = series.get("Policy/mean_std")
    if std and std[-1].value < 0.1:
        add("bad", f"Policy/mean_std {std[-1].value:.3g} — 탐색 노이즈가 거의 0. 탐색 붕괴 위험.")

    if not items:
        add("warn", "비교할 scalar가 부족함.")
    return items


def build_html(run: Path, event: Path, tags: list[str], series: dict, snaps: list[Path],
               run_json: dict, cfg_files: list[tuple[str, Path]], env_cfg: dict,
               agent_cfg: dict) -> str:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    created = run_json.get("created_at", "")
    task = run_json.get("task", "")
    first_wall = min((ev[0].wall_time for ev in series.values() if ev), default=None)
    last_wall = max((ev[-1].wall_time for ev in series.values() if ev), default=None)
    elapsed = (last_wall - first_wall) if (first_wall and last_wall) else None
    main_ev = series.get("Train/mean_reward")
    last_step = main_ev[-1].step if main_ev else max((ev[-1].step for ev in series.values() if ev), default=None)

    combined = combined_series(series, env_cfg)
    combined_card = ""
    combined_block = ""
    if len(combined) >= 2:
        card_title, plot_title, ylabel, caption = combined_texts(combined_kind(series, env_cfg))
        c_first, c_last = 100.0 * combined[0][1], 100.0 * combined[-1][1]
        cls = "good" if c_last < c_first * 0.95 else "bad" if c_last > c_first * 1.05 else "warn"
        word = "좋아짐" if cls == "good" else "나빠짐" if cls == "bad" else "정체"
        combined_card = (
            f'<div class="card"><div class="k">{card_title}</div>'
            f'<div class="v">{c_last:.1f}% <span class="muted">← {c_first:.1f}%</span></div>'
            f'<div class="k">{word}</div></div>')
        combined_block = (
            '<h2>종합 지표 — 한 눈에</h2>'
            f'<p class="muted">{caption}</p>'
            f'<figure class="plot big">{b64img(combined_png(combined, plot_title, ylabel), "combined")}'
            f'<figcaption><b>{card_title}</b>'
            f' · first {c_first:.1f}% → last {c_last:.1f}% ({word})</figcaption></figure>')

    # 학습 설정 — envs/iterations/batch/epochs/PPO/로봇 config를 한 표로
    scene = env_cfg.get("scene") if isinstance(env_cfg.get("scene"), dict) else {}
    sim = env_cfg.get("sim") if isinstance(env_cfg.get("sim"), dict) else {}
    algo = agent_cfg.get("algorithm") if isinstance(agent_cfg.get("algorithm"), dict) else {}
    actor = agent_cfg.get("actor") if isinstance(agent_cfg.get("actor"), dict) else {}
    envs = scene.get("num_envs")
    steps = agent_cfg.get("num_steps_per_env")
    max_it = agent_cfg.get("max_iterations")
    epochs = algo.get("num_learning_epochs")
    mini = algo.get("num_mini_batches")
    rollout = int(envs) * int(steps) if isinstance(envs, (int, float)) and isinstance(steps, (int, float)) else None
    mini_size = int(rollout) // int(mini) if rollout and isinstance(mini, (int, float)) and int(mini) > 0 else None
    dt = sim.get("dt")
    decimation = env_cfg.get("decimation")
    policy_dt = float(dt) * int(decimation) if isinstance(dt, (int, float)) and isinstance(decimation, (int, float)) else None
    ep_s = env_cfg.get("episode_length_s")
    ep_steps = round(float(ep_s) / policy_dt) if ep_s and policy_dt else None
    scale = env_cfg.get("action_scale")
    hidden = actor.get("hidden_dims")
    init_std = (actor.get("distribution_cfg") or {}).get("init_std") if isinstance(actor.get("distribution_cfg"), dict) else None
    fps = series["Perf/total_fps"][-1].value if series.get("Perf/total_fps") else None
    progress = (100.0 * last_step / max_it) if (last_step is not None and isinstance(max_it, (int, float)) and max_it) else None
    total_steps = int(max_it) * rollout if isinstance(max_it, (int, float)) and rollout else None
    done_steps = int(last_step) * rollout if last_step is not None and rollout else None
    setup_pairs = [
        ("run name", run.name),
        ("task", task or "—"),
        ("created", created or "—"),
        ("envs", envs if envs is not None else "—"),
        ("iterations", f"{max_it} (last {last_step}, {progress:.1f}%)" if progress is not None else f"{max_it} (last {last_step})"),
        ("rollout / iter", f"{rollout:,} step (envs × num_steps_per_env)" if rollout else "—"),
        ("PPO epochs / iter", epochs if epochs is not None else "—"),
        ("mini-batches / iter", mini if mini is not None else "—"),
        ("mini-batch size", f"{mini_size:,} step" if mini_size else "—"),
        ("total steps (planned)", f"{total_steps:,}" if total_steps else "—"),
        ("steps done", f"{done_steps:,}" if done_steps else "—"),
        ("learning rate / schedule", f"{algo.get('learning_rate', '—')} / {algo.get('schedule', '—')}"),
        ("gamma / lam", f"{algo.get('gamma', '—')} / {algo.get('lam', '—')}"),
        ("clip / entropy / desired_kl", f"{algo.get('clip_param', '—')} / {algo.get('entropy_coef', '—')} / {algo.get('desired_kl', '—')}"),
        ("max_grad_norm", algo.get("max_grad_norm", "—")),
        ("sim dt / decimation / policy dt", f"{dt:.6g} s / {decimation} / {policy_dt:.4g} s ({1/policy_dt:.0f} Hz)" if policy_dt else "—"),
        ("episode", f"{ep_s} s = {ep_steps} step" if ep_steps else str(ep_s or "—")),
        ("action_scale (lin/ang)", f"{float(scale[0]):.3g} / {float(scale[1]):.3g}" if isinstance(scale, (list, tuple)) and len(scale) >= 2 else "—"),
        ("max_lin / max_ang", f"{env_cfg.get('max_lin', '—')} / {env_cfg.get('max_ang', '—')}"),
        ("network", f"{hidden} · init_std {init_std}" if hidden is not None else "—"),
        ("seed / device", f"{agent_cfg.get('seed', env_cfg.get('seed', '—'))} / {agent_cfg.get('device', '—')}"),
        ("save_interval / logger", f"{agent_cfg.get('save_interval', '—')} / {agent_cfg.get('logger', '—')}"),
        ("fps (last)", f"{fps:,.0f}" if fps else "—"),
    ]
    setup_rows = "".join(
        f"<tr><td>{html.escape(str(k))}</td><td>{html.escape(str(v))}</td>"
        f"<td class='desc'>{html.escape(SETUP_INFO.get(k, ''))}</td></tr>" for k, v in setup_pairs)
    setup_html = ('<table class="setup"><thead><tr><th>항목</th><th>값</th><th>설명</th></tr></thead>'
                  f'<tbody>{setup_rows}</tbody></table>')

    verdict = verdict_items(series, env_cfg)
    verdict_html = "".join(f'<li class="{cls}">{html.escape(text)}</li>' for cls, text in verdict)

    def metric_card(tag: str) -> str:
        info = tag_info(tag)
        return (
            f'<div class="metric"><h3><code>{html.escape(tag)}</code> — {html.escape(info["label"])}</h3>'
            f'<p>{html.escape(info["desc"])}</p><ul>'
            f'<li><b>낮아지면</b> {html.escape(info["low"])}</li>'
            f'<li><b>높아지면</b> {html.escape(info["high"])}</li>'
            f'<li><b>좋아지는 방향</b> {html.escape(info["good"])}</li></ul></div>')

    glossary_cards = []
    if combined and "Metrics/combined_err" not in tags:
        glossary_cards.append(metric_card("Metrics/combined_err"))
    glossary_cards.extend(metric_card(tag) for tag in tags)
    glossary = '<div class="glossary">' + "".join(glossary_cards) + "</div>"

    scale = env_cfg.get("action_scale")
    scale_card = ""
    if isinstance(scale, (list, tuple)) and len(scale) >= 2:
        scale_card = (f'<div class="card"><div class="k">action_scale (lin/ang)</div>'
                      f'<div class="v">{float(scale[0]):.3g} / {float(scale[1]):.3g}</div></div>')

    rows = []
    for tag in tags:
        ev = series.get(tag)
        if not ev:
            continue
        cls = ' class="key"' if tag in KEY_TAGS else ""
        rows.append(
            f"<tr{cls}><td>{html.escape(tag)}</td><td>{len(ev)}</td><td>{fmt(ev[0].value)}</td>"
            f"<td>{fmt(ev[-1].value)}</td><td>{fmt(min(x.value for x in ev))}</td>"
            f"<td>{fmt(max(x.value for x in ev))}</td></tr>")
    table = (
        "<table><thead><tr><th>tag</th><th>n</th><th>first</th><th>last</th><th>min</th><th>max</th></tr></thead>"
        f"<tbody>{''.join(rows)}</tbody></table>")

    plot_cards = []
    for tag in tags:
        ev = series.get(tag)
        if not ev:
            continue
        info = tag_info(tag)
        plot_cards.append(
            f'<figure class="plot"><figcaption><b>{html.escape(tag)}</b> — {html.escape(info["label"])}'
            f'<br><span class="muted">{html.escape(info["desc"])} · {html.escape(info["good"])}</span></figcaption>'
            f'{b64img(plot_png(tag, ev), tag)}</figure>')
    plots = "".join(plot_cards) if plot_cards else "<p>scalar 없음</p>"

    snap_cards = []
    for p in snaps:
        snap_cards.append(
            f'<figure class="snap">{b64img(image_b64(p), p.name)}'
            f'<figcaption>{html.escape(p.name)}</figcaption></figure>')
    snaps_html = "".join(snap_cards) if snap_cards else "<p>snapshot 없음</p>"

    cfg_html = []
    for name, p in cfg_files:
        try:
            text = p.read_text(errors="replace")
        except OSError as exc:
            text = f"(읽기 실패: {exc})"
        cfg_html.append(
            f"<details><summary>{html.escape(name)} <span class='muted'>({p.stat().st_size} B)</span></summary>"
            f"<pre>{html.escape(text)}</pre></details>")
    config = "".join(cfg_html) if cfg_html else "<p>config 파일 없음</p>"

    elapsed_s = f"{elapsed:.0f}s" if elapsed is not None else "—"
    last_time = datetime.fromtimestamp(last_wall).strftime("%Y-%m-%d %H:%M:%S") if last_wall else "—"
    title = f"{task or 'run'} · {run.name}"
    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<title>{html.escape(title)}</title>
<style>
  body {{font-family: system-ui, -apple-system, "Segoe UI", sans-serif; margin: 24px auto; max-width: 1240px;
         color: #202124; line-height: 1.45;}}
  h1 {{font-size: 22px; margin: 0 0 4px;}}
  h2 {{font-size: 16px; margin: 28px 0 10px; border-bottom: 1px solid #e5e7eb; padding-bottom: 4px;}}
  .muted {{color: #6b7280; font-weight: 400;}}
  .cards {{display: grid; grid-template-columns: repeat(auto-fit, minmax(165px, 1fr)); gap: 10px; margin: 14px 0 4px;}}
  .card {{background: #f8fafc; border: 1px solid #e5e7eb; border-radius: 6px; padding: 10px 12px;}}
  .card .k {{font-size: 11px; color: #6b7280;}}
  .card .v {{font-size: 16px; font-weight: 600; margin-top: 2px;}}
  table {{border-collapse: collapse; width: 100%; font-size: 13px;}}
  th, td {{border: 1px solid #e5e7eb; padding: 4px 7px; text-align: right;}}
  th:first-child, td:first-child {{text-align: left;}}
  th {{background: #f3f4f6;}}
  tr.key td {{background: #f8fafc; font-weight: 600;}}
  .verdict {{list-style: none; padding-left: 0; margin: 0;}}
  .verdict li {{margin: 5px 0; padding: 7px 10px; border-left: 4px solid #d1d5db; background: #f9fafb;
                border-radius: 3px;}}
  .verdict li.good {{border-left-color: #16a34a;}}
  .verdict li.warn {{border-left-color: #d97706;}}
  .verdict li.bad {{border-left-color: #dc2626;}}
  .glossary {{display: grid; grid-template-columns: repeat(auto-fill, minmax(560px, 1fr)); gap: 12px;}}
  .metric {{background: #fff; border: 1px solid #e5e7eb; border-radius: 6px; padding: 10px 12px;}}
  .metric h3 {{font-size: 13px; margin: 0 0 6px;}}
  .metric p {{font-size: 12.5px; color: #374151; margin: 4px 0 6px;}}
  .metric ul {{margin: 0; padding-left: 18px; font-size: 12.5px; color: #4b5563;}}
  .metric li {{margin: 3px 0;}}
  .setup th {{text-align: left;}}
  .setup td {{text-align: left; vertical-align: top;}}
  .setup td:first-child {{color: #4b5563; width: 22%; white-space: nowrap;}}
  .setup td:nth-child(2) {{width: 28%;}}
  .setup td.desc {{color: #4b5563; font-size: 12.5px;}}
  .setup tr:nth-child(even) td {{background: #fafafa;}}
  .grid {{display: grid; grid-template-columns: repeat(auto-fill, minmax(420px, 1fr)); gap: 14px;}}
  .plot {{margin: 0; background: #fff; border: 1px solid #e5e7eb; border-radius: 6px; padding: 6px;}}
  .plot figcaption {{font-size: 12px; color: #4b5563; margin: 2px 4px 4px;}}
  .plot img {{width: 100%; display: block;}}
  .snaps {{display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 12px;}}
  .snap {{margin: 0; border: 1px solid #e5e7eb; border-radius: 6px; padding: 6px; background: #fff;}}
  .snap img {{width: 100%; display: block;}}
  .snap figcaption {{font-size: 11px; color: #6b7280; margin-top: 4px;}}
  pre {{background: #f6f8fa; border: 1px solid #e5e7eb; border-radius: 6px; padding: 10px; overflow: auto;
        font-size: 12px; max-height: 420px;}}
  details {{margin: 8px 0;}}
  code {{background: #f3f4f6; padding: 1px 4px; border-radius: 3px;}}
  footer {{margin-top: 32px; color: #6b7280; font-size: 12px;}}
</style></head><body>
<h1>{html.escape(title)}</h1>
<p class="muted">run <code>{html.escape(run.name)}</code> · report {now}</p>
<p>이 리포트는 학습 run의 <b>TensorBoard event</b>(마지막으로 기록된 iteration까지) · <b>snapshot</b> · <b>config</b>를
모아 만든 정적 HTML입니다. <b>요약 판정</b>은 first/last 자동 비교라 <b>참고용</b>이고,
최종 판단은 아래 plot의 추세와 실기기 A/B로 하세요.</p>

<div class="cards">
  <div class="card"><div class="k">created</div><div class="v">{html.escape(str(created))}</div></div>
  <div class="card"><div class="k">last iteration</div><div class="v">{last_step if last_step is not None else '—'}</div></div>
  <div class="card"><div class="k">last event time</div><div class="v">{last_time}</div></div>
  <div class="card"><div class="k">elapsed (events)</div><div class="v">{elapsed_s}</div></div>
  <div class="card"><div class="k">scalar tags</div><div class="v">{len(tags)}</div></div>
  <div class="card"><div class="k">snapshots</div><div class="v">{len(snaps)}</div></div>
  {scale_card}
  {combined_card}
</div>

<h2>학습 설정 <span class="muted">(params/env.yaml · agent.yaml — 학습 시작 시 고정)</span></h2>
{setup_html}

<h2>요약 판정 <span class="muted">(자동 first/last 비교 · 참고용)</span></h2>
<ul class="verdict">{verdict_html}</ul>

<h2>지표 설명</h2>
{glossary}

<h2>Scalars</h2>
{table}

{combined_block}

<h2>Plots</h2>
<div class="grid">{plots}</div>

<h2>Snapshots <span class="muted">({len(snaps)})</span></h2>
<div class="snaps">{snaps_html}</div>

<h2>Config</h2>
{config}

<footer>event: {html.escape(event.name)} · run dir: {html.escape(str(run))}</footer>
</body></html>
"""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="학습 run → report.html (plots · table · snapshots · config)")
    ap.add_argument("run", help="run 폴더 | event 파일 | task 이름 (최신 run)")
    ap.add_argument("--out", default=None, help="기본: <run>/report.html")
    ap.add_argument("--max-snaps", type=int, default=24, help="snapshot 최대 개수 (0=전부, 기본 24)")
    args = ap.parse_args(argv)

    run = find_run(args.run)
    events = sorted(run.glob("events.out.tfevents.*"), key=lambda p: p.stat().st_mtime)
    if not events:
        raise SystemExit(f"event 파일 없음: {run}")
    event = events[-1]
    ea = EventAccumulator(str(event), size_guidance={"scalars": 0})
    ea.Reload()
    tags = sort_tags([t for t in ea.Tags().get("scalars", [])
                      if ea.Scalars(t) and not t.endswith("/time")])
    series = {tag: ea.Scalars(tag) for tag in tags}

    snap_dir = run / "snap"
    snaps = sorted(snap_dir.glob("*.png")) if snap_dir.is_dir() else []
    if not snaps and (run / "latest.png").is_file():
        snaps = [run / "latest.png"]
    if args.max_snaps and len(snaps) > args.max_snaps:
        snaps = snaps[-args.max_snaps:]

    run_json = {}
    if (run / "run.json").is_file():
        try:
            run_json = json.loads((run / "run.json").read_text())
        except (OSError, json.JSONDecodeError):
            pass

    env_cfg = {}
    env_path = run / "params/env.yaml"
    if yaml is not None and env_path.is_file():
        try:
            env_cfg = yaml.unsafe_load(env_path.read_text()) or {}
        except Exception:
            env_cfg = {}

    agent_cfg = {}
    agent_path = run / "params/agent.yaml"
    if yaml is not None and agent_path.is_file():
        try:
            agent_cfg = yaml.unsafe_load(agent_path.read_text()) or {}
        except Exception:
            agent_cfg = {}

    cfg = []
    for rel in ("params/env.yaml", "params/agent.yaml", "run.json", "git/IsaacLab-6.1.diff"):
        p = run / rel
        if p.is_file():
            cfg.append((rel, p))

    out = Path(args.out) if args.out else run / "report.html"
    if out.is_dir():
        out = out / "report.html"
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = build_html(run, event, tags, series, snaps, run_json, cfg, env_cfg, agent_cfg)
    out.write_text(doc)
    print(f"[report] {out} ({out.stat().st_size / 1024:.0f} KB · {len(tags)} tags · {len(snaps)} snaps)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
