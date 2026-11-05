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
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator  # noqa: E402

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
    "Train/mean_reward": (
        "에피소드 전체 보상 합",
        "음수일 수 있음. 에피소드 길이가 변하면 총합도 같이 변하므로 "
        "mean_reward ÷ mean_episode_length(스텝당)로 보는 게 안전.",
        "덜 음수"),
    "Metrics/track_err_v": (
        "전진 속도 추종 오차 [m/s]",
        "몸체 v가 명령(정책 보정 후)을 얼마나 못 따라갔는지. 이 task의 실제 목표. "
        "action_scale_lin보다 크면 정책 출력이 포화됐을 수 있음.",
        "작을수록 좋음"),
    "Metrics/track_err_w": (
        "회전 속도 추종 오차 [rad/s]",
        "몸체 w가 명령(정책 보정 후)을 얼마나 못 따라갔는지. "
        "action_scale_ang보다 크면 정책 출력이 포화됐을 수 있음.",
        "작을수록 좋음"),
    "Metrics/combined_err": (
        "종합 추종 오차 [0~1]",
        "0.5 × (|v_err|/max_lin + |w_err|/max_ang). 0=완벽, 1=최대 속도 오차. "
        "v·w를 한 숫자로 합친 이 task의 대표 지표.",
        "작을수록 좋음"),
    "Metrics/cross_track": (
        "경로 이탈 [m]",
        "웨이포인트 경로(앞 점 → 다음 점 직선)에서 벗어난 평균 거리 (rl_maze).",
        "작을수록 좋음"),
    "Metrics/waypoints_done": (
        "도착 웨이포인트 수",
        "에피소드에서 통과한 웨이포인트 수 (rl_maze).",
        "클수록 좋음"),
    "Metrics/collision_rate": (
        "충돌 비율",
        "벽/장애물 충돌로 끝난 비율.",
        "작을수록 좋음"),
    "Metrics/goal_rate": (
        "최종 goal 도달 비율",
        "마지막 웨이포인트까지 간 비율.",
        "클수록 좋음"),
    "Train/mean_episode_length": (
        "평균 에피소드 길이 [step]",
        "최대 길이에 붙어 있는데 track_err가 안 줄면 '버티기'만 배운 것일 수 있음.",
        "적당히 안정"),
    "Loss/value": (
        "critic value loss",
        "가치 함수 학습 오차. 수백~수천이면 reward scale이 크거나 critic이 return을 못 맞추는 중.",
        "작을수록 좋음"),
    "Loss/surrogate": (
        "PPO policy loss",
        "0 근처면 정책 업데이트가 작음 (KL/clip 상태 확인).",
        "0 근처"),
    "Loss/entropy": (
        "정책 엔트로피",
        "탐색량. 급격히 0이 되면 탐색이 붕괴.",
        "서서히 감소"),
    "Loss/learning_rate": (
        "학습률 (adaptive면 자동 조절)",
        "adaptive schedule이 0.01 max에 붙으면 발산 위험. fixed 3e-4 권장.",
        "낮게 유지"),
    "Policy/mean_std": (
        "행동 표준편차",
        "탐색 노이즈. 0으로 붕괴하면 학습이 멈춤.",
        "0.2~0.5"),
    "Perf/total_fps": ("학습 처리량 [step/s]", "학습 속도 (정책 성능 아님).", "참고"),
    "Perf/collection_time": ("수집 시간 [s/iter]", "환경 스텝 수집 시간.", "참고"),
    "Perf/learning_time": ("학습 시간 [s/iter]", "PPO 업데이트 시간.", "참고"),
}


def tag_info(tag: str) -> tuple[str, str, str]:
    if tag in TAG_INFO:
        return TAG_INFO[tag]
    if tag.startswith("Metrics/"):
        return ("task 지표", "task가 기록한 scalar. task README와 태그 이름을 참고.", "task에 따라")
    if tag.startswith("Loss/"):
        return ("loss", "학습 loss.", "작을수록 좋음")
    if tag.startswith("Train/"):
        return ("학습 지표", "학습 중 집계된 값.", "task에 따라")
    if tag.startswith("Policy/"):
        return ("정책 통계", "정책 분포/노이즈 통계.", "안정적")
    if tag.startswith("Perf/"):
        return ("성능", "처리량/시간.", "참고")
    return ("scalar", "설명 없음 — 태그 이름을 참고.", "—")


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
    fig, ax = plt.subplots(figsize=(5.4, 2.7), dpi=110)
    ax.plot([e.step for e in ev], [e.value for e in ev], lw=1.2)
    ax.set_title(tag, fontsize=9)
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
    """v·w 추종 오차를 한 숫자로: 0.5*(|v|/max_lin + |w|/max_ang) — 낮을수록 좋음."""
    ev = series.get("Metrics/combined_err")
    if ev:
        return [(e.step, e.value) for e in ev]
    v = series.get("Metrics/track_err_v")
    w = series.get("Metrics/track_err_w")
    max_lin, max_ang = env_cfg.get("max_lin"), env_cfg.get("max_ang")
    if not v or not w or not max_lin or not max_ang:
        return []
    wm = {e.step: e.value for e in w}
    return [(e.step, 0.5 * (abs(e.value) / float(max_lin) + abs(wm[e.step]) / float(max_ang)))
            for e in v if e.step in wm]


def combined_png(pairs: list[tuple[int, float]]) -> str:
    fig, ax = plt.subplots(figsize=(11.0, 3.1), dpi=115)
    xs = [p[0] for p in pairs]
    ys = [100.0 * p[1] for p in pairs]
    ax.plot(xs, ys, lw=2.0, color="#2563eb")
    ax.fill_between(xs, ys, 0, color="#2563eb", alpha=0.10)
    ax.set_title("Combined tracking error — 0.5·(|v_err|/max_lin + |w_err|/max_ang) × 100 [%]  (lower is better)",
                 fontsize=10)
    ax.set_xlabel("iteration", fontsize=8)
    ax.set_ylabel("%", fontsize=8)
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
        first, last = 100.0 * comb[0][1], 100.0 * comb[-1][1]
        cls = "good" if last < first * 0.95 else "bad" if last > first * 1.05 else "warn"
        word = "좋아짐" if cls == "good" else "나빠짐" if cls == "bad" else "정체"
        add(cls, f"종합 추종 오차: {first:.1f}% → {last:.1f}% — {word} "
                 f"(0=완벽, 100%=최대 속도 오차)")

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
               run_json: dict, cfg_files: list[tuple[str, Path]], env_cfg: dict) -> str:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    created = run_json.get("created_at", "")
    task = run_json.get("task", "")
    first_wall = min((ev[0].wall_time for ev in series.values() if ev), default=None)
    last_wall = max((ev[-1].wall_time for ev in series.values() if ev), default=None)
    elapsed = (last_wall - first_wall) if (first_wall and last_wall) else None
    last_step = max((ev[-1].step for ev in series.values() if ev), default=None)

    combined = combined_series(series, env_cfg)
    combined_card = ""
    combined_block = ""
    if len(combined) >= 2:
        c_first, c_last = 100.0 * combined[0][1], 100.0 * combined[-1][1]
        cls = "good" if c_last < c_first * 0.95 else "bad" if c_last > c_first * 1.05 else "warn"
        word = "좋아짐" if cls == "good" else "나빠짐" if cls == "bad" else "정체"
        combined_card = (
            f'<div class="card"><div class="k">종합 추종 오차 (낮을수록)</div>'
            f'<div class="v">{c_last:.1f}% <span class="muted">← {c_first:.1f}%</span></div>'
            f'<div class="k">{word}</div></div>')
        combined_block = (
            '<h2>종합 지표 — 한 눈에</h2>'
            '<p class="muted">v·w 추종 오차를 한 숫자로 합친 값입니다. '
            '<b>0% = 완벽, 100% = 최대 속도 오차</b>. 낮을수록 좋고, 선이 오른쪽으로 갈수록 내려가면 학습이 좋아지는 중입니다.</p>'
            f'<figure class="plot big">{b64img(combined_png(combined), "combined track error")}'
            f'<figcaption><b>종합 추종 오차</b> — 0.5·(|track_err_v|/max_lin + |track_err_w|/max_ang) × 100 [%]'
            f' · first {c_first:.1f}% → last {c_last:.1f}% ({word})</figcaption></figure>')

    verdict = verdict_items(series, env_cfg)
    verdict_html = "".join(f'<li class="{cls}">{html.escape(text)}</li>' for cls, text in verdict)

    glossary_rows = []
    if combined and "Metrics/combined_err" not in tags:
        label, desc, good = tag_info("Metrics/combined_err")
        glossary_rows.append(
            f"<tr><td><code>Metrics/combined_err</code> <span class='muted'>(계산)</span></td>"
            f"<td>{html.escape(label)}</td><td>{html.escape(desc)}</td><td>{html.escape(good)}</td></tr>")
    for tag in tags:
        label, desc, good = tag_info(tag)
        glossary_rows.append(
            f"<tr><td><code>{html.escape(tag)}</code></td><td>{html.escape(label)}</td>"
            f"<td>{html.escape(desc)}</td><td>{html.escape(good)}</td></tr>")
    glossary = (
        "<table><thead><tr><th>tag</th><th>뜻</th><th>설명</th><th>좋아지는 방향</th></tr></thead>"
        f"<tbody>{''.join(glossary_rows)}</tbody></table>")

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
        label, desc, good = tag_info(tag)
        plot_cards.append(
            f'<figure class="plot"><figcaption><b>{html.escape(tag)}</b> — {html.escape(label)}'
            f'<br><span class="muted">{html.escape(desc)} · {html.escape(good)}</span></figcaption>'
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
    tags = sort_tags([t for t in ea.Tags().get("scalars", []) if ea.Scalars(t)])
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

    cfg = []
    for rel in ("params/env.yaml", "params/agent.yaml", "run.json", "git/IsaacLab-6.1.diff"):
        p = run / rel
        if p.is_file():
            cfg.append((rel, p))

    out = Path(args.out) if args.out else run / "report.html"
    if out.is_dir():
        out = out / "report.html"
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = build_html(run, event, tags, series, snaps, run_json, cfg, env_cfg)
    out.write_text(doc)
    print(f"[report] {out} ({out.stat().st_size / 1024:.0f} KB · {len(tags)} tags · {len(snaps)} snaps)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
