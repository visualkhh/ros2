#!/bin/bash
# nav2_rl 본학습 (런: isaac/logs/nav2_rl/<날짜_시간>/)
#   ./train_nav2_rl.sh [max_iterations] [--visualizer kit]
set -e
ITERS=${1:-300}
shift || true
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/train.py \
  --task nav2_rl --max-iterations "$ITERS" "$@"
