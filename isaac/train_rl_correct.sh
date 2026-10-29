#!/bin/bash
# rl_correct 본학습 (런: isaac/logs/rl_correct/<날짜_시간>/)
#   ./train_rl_correct.sh [max_iterations] [--visualizer kit]
# 보기: TensorBoard -> isaac/logs/rl_correct/<날짜_시간>/
set -e
ITERS=300
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then ITERS=$1; shift; fi   # 첫 인자가 숫자일 때만 (옵션만 줘도 됨)
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/train.py \
  --max-iterations "$ITERS" "$@"
