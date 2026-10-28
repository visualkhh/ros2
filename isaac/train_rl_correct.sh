#!/bin/bash
# rl_correct 본학습 (런: isaac/logs/rl_correct/<날짜_시간>/)
#   ./train_rl_correct.sh [max_iterations] [--visualizer kit]
# 보기: TensorBoard -> isaac/logs/rl_correct/<날짜_시간>/
set -e
ITERS=${1:-300}
shift || true
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/train.py \
  --task rl_correct --max-iterations "$ITERS" "$@"
