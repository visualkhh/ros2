#!/bin/bash
# rl_correct 본학습 (런: isaac/logs/rl_correct/<날짜_시간>/)
#   ./training/rl_correction_wheel_drift/train.sh [max_iterations] [--watch] [--envs 16]
#   --watch = 창 보기 (--visualizer kit). 512 env는 느리니 16~32 권장.
# 보기: TensorBoard -> isaac/logs/rl_correct/<날짜_시간>/
set -e
ITERS=300
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then ITERS=$1; shift; fi   # 첫 인자가 숫자일 때만 (옵션만 줘도 됨)
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/training/rl_correction_wheel_drift/train.py \
  --max-iterations "$ITERS" "$@"
