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
    "Metrics/cross_track",
    "Metrics/waypoints_done",
    "Metrics/collision_rate",
    "Metrics/goal_rate",
    "Train/mean_episode_length",
    "Loss/value",
    "Loss/learning_rate",
    "Policy/mean_std",
)


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


def build_html(run: Path, event: Path, tags: list[str], series: dict, snaps: list[Path],
               run_json: dict, cfg_files: list[tuple[str, Path]]) -> str:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    created = run_json.get("created_at", "")
    task = run_json.get("task", "")
    first_wall = min((ev[0].wall_time for ev in series.values() if ev), default=None)
    last_wall = max((ev[-1].wall_time for ev in series.values() if ev), default=None)
    elapsed = (last_wall - first_wall) if (first_wall and last_wall) else None
    last_step = max((ev[-1].step for ev in series.values() if ev), default=None)

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
        plot_cards.append(
            f'<figure class="plot"><figcaption>{html.escape(tag)}</figcaption>'
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

<div class="cards">
  <div class="card"><div class="k">created</div><div class="v">{html.escape(str(created))}</div></div>
  <div class="card"><div class="k">last iteration</div><div class="v">{last_step if last_step is not None else '—'}</div></div>
  <div class="card"><div class="k">last event time</div><div class="v">{last_time}</div></div>
  <div class="card"><div class="k">elapsed (events)</div><div class="v">{elapsed_s}</div></div>
  <div class="card"><div class="k">scalar tags</div><div class="v">{len(tags)}</div></div>
  <div class="card"><div class="k">snapshots</div><div class="v">{len(snaps)}</div></div>
</div>

<h2>Scalars</h2>
{table}

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
    cfg = []
    for rel in ("params/env.yaml", "params/agent.yaml", "run.json", "git/IsaacLab-6.1.diff"):
        p = run / rel
        if p.is_file():
            cfg.append((rel, p))

    out = Path(args.out) if args.out else run / "report.html"
    if out.is_dir():
        out = out / "report.html"
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = build_html(run, event, tags, series, snaps, run_json, cfg)
    out.write_text(doc)
    print(f"[report] {out} ({out.stat().st_size / 1024:.0f} KB · {len(tags)} tags · {len(snaps)} snaps)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
