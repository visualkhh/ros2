#!/bin/bash
# 학습 진행 요약 — TensorBoard event의 주요 scalar(first/last/min/max)를 표로.
#
#   ./run-tb-summary.sh <task|run 폴더|event 파일> [--watch 초] [--tags TAG,TAG,...]
#
#   ./run-tb-summary.sh rl_nav_correction
#   ./run-tb-summary.sh rl_nav_correction --watch 10
#   ./run-tb-summary.sh rl_nav_correction --tags Train/mean_reward,Metrics/track_err_w
#   ./run-tb-summary.sh artifacts/rl_nav_correction/training/2026-11-05_12-46-27_v1
#
# TensorBoard UI:
#   /home/pank/pa-course/IsaacLab-6.1/.venv/bin/tensorboard --logdir artifacts/rl_nav_correction/training
set -euo pipefail

ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"
PY="$ISAACLAB/.venv/bin/python"
[ -x "$PY" ] || PY=python3

if [[ $# -eq 0 || "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi

exec "$PY" "$ISAAC/tools/tb_summary.py" "$@"
