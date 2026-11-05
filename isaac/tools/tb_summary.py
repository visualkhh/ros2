#!/usr/bin/env python3
"""학습 진행 확인 — TensorBoard event 파일에서 주요 scalar의 first/last/min/max를 표로.

  # 태스크 이름 (artifacts/<task>/training/의 가장 최근 run)
  /home/pank/pa-course/IsaacLab-6.1/.venv/bin/python isaac/tools/tb_summary.py rl_nav_correction

  # run 폴더 / event 파일 직접
  ... tools/tb_summary.py artifacts/rl_nav_correction/training/2026-11-05_12-46-27_v1
  ... tools/tb_summary.py artifacts/.../events.out.tfevents....

  # 10초마다 갱신 (Ctrl+C로 끝)
  ... tools/tb_summary.py rl_nav_correction --watch 10

TensorBoard UI:
  /home/pank/pa-course/IsaacLab-6.1/.venv/bin/tensorboard --logdir isaac/artifacts/rl_nav_correction/training
"""
from __future__ import annotations

import argparse
import glob
import time
from pathlib import Path

try:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
except ImportError:
    raise SystemExit("tensorboard가 없음 — IsaacLab venv python으로 실행: "
                     "/home/pank/pa-course/IsaacLab-6.1/.venv/bin/python")

ISAAC = Path(__file__).resolve().parents[1]
DEFAULT_TAGS = (
    "Train/mean_reward",
    "Metrics/track_err_v",
    "Metrics/track_err_w",
    "Train/mean_episode_length",
    "Loss/value",
    "Loss/learning_rate",
    "Policy/mean_std",
)


def find_event(arg: str) -> Path:
    p = Path(arg).expanduser()
    if p.is_file():
        return p
    if p.is_dir():
        events = sorted(p.glob("events.out.tfevents.*"))
        if events:
            return max(events, key=lambda x: x.stat().st_mtime)
        runs = sorted((d for d in p.iterdir() if d.is_dir()),
                      key=lambda d: d.stat().st_mtime, reverse=True)
        for run in runs:
            events = sorted(run.glob("events.out.tfevents.*"))
            if events:
                return max(events, key=lambda x: x.stat().st_mtime)
        raise SystemExit(f"event 파일 없음: {p}")
    # 태스크 이름
    base = ISAAC / "artifacts" / arg / "training"
    if not base.is_dir():
        raise SystemExit(f"task/run 경로 없음: {arg} (여기도 없음: {base})")
    return find_event(str(base))


def summary(event: Path, tags: tuple[str, ...]) -> str:
    ea = EventAccumulator(str(event), size_guidance={"scalars": 0})
    ea.Reload()
    have = set(ea.Tags().get("scalars", []))
    lines = [f"[tb] {event}", f"[tb] run {event.parent.name}"]
    last_step = None
    for tag in tags:
        if tag not in have:
            lines.append(f"  {tag:30s} (없음)")
            continue
        ev = ea.Scalars(tag)
        if not ev:
            continue
        last_step = ev[-1].step
        lines.append(f"  {tag:30s} n={len(ev):4d} first={ev[0].value:10.4f} "
                     f"last={ev[-1].value:10.4f} min={min(x.value for x in ev):10.4f} "
                     f"max={max(x.value for x in ev):10.4f}")
    if last_step is not None:
        lines.append(f"  {'iteration':30s} last={last_step}")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="TensorBoard event 요약 (first/last/min/max)")
    ap.add_argument("run", help="task 이름 | run 폴더 | event 파일")
    ap.add_argument("--tags", default=",".join(DEFAULT_TAGS),
                    help="쉼표로 구분한 scalar 태그 (기본: 주요 지표)")
    ap.add_argument("--watch", type=float, default=0.0, metavar="SEC",
                    help="SEC초마다 다시 읽기 (0=한 번)")
    args = ap.parse_args(argv)
    tags = tuple(t.strip() for t in args.tags.split(",") if t.strip())
    event = find_event(args.run)
    while True:
        print(summary(event, tags), flush=True)
        if args.watch <= 0:
            return 0
        time.sleep(args.watch)
        print("-" * 80, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
