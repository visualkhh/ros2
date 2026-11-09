#!/bin/bash
# TensorBoard 열기 — artifacts/<task>/training의 학습 run들을 그래프로 (브라우저 http://localhost:6006).
#
#   ./run-tensorboard.sh <task|logdir> [--port 6006] [tensorboard 옵션...]
#
#   ./run-tensorboard.sh rl_maze_beebot                 # = --logdir artifacts/rl_maze_beebot/training (run마다 한 줄로 겹쳐 비교)
#   ./run-tensorboard.sh rl_maze_beebot --port 6007     # 다른 포트
#   ./run-tensorboard.sh artifacts/rl_nav_correction/training/2026-11-05_12-46-27_v1   # run 하나만
#
# 숫자만 터미널로 보려면: ./run-tb-summary.sh <task> --watch 10
set -euo pipefail

ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"
TB="$ISAACLAB/.venv/bin/tensorboard"

if [[ $# -eq 0 || "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
[[ -x "$TB" ]] || { echo "tensorboard 없음: $TB (IsaacLab venv에 설치되어 있어야 함)" >&2; exit 2; }

ARG="$1"; shift
if [[ -d "$ARG" ]]; then LOGDIR=$(realpath "$ARG")                      # 폴더를 직접 준 경우
elif [[ -d "$ISAAC/artifacts/$ARG/training" ]]; then LOGDIR="$ISAAC/artifacts/$ARG/training"   # task 이름
else echo "학습 폴더 없음: $ARG (artifacts/$ARG/training도 없음)" >&2; exit 2; fi

PORT=6006; for ((i = 1; i <= $#; i++)); do [[ "${!i}" == --port ]] && j=$((i + 1)) && PORT="${!j}"; [[ "${!i}" == --port=* ]] && PORT="${!i#--port=}"; done
echo "[tensorboard] $LOGDIR → http://localhost:$PORT (Ctrl+C로 끝)"
exec "$TB" --logdir "$LOGDIR" "$@"
