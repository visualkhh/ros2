#!/bin/bash
# goal 환경 본학습 (목표 변위까지 이동)
#   ./rl/wheel_drift/train_goal.sh [max_iterations] [--watch] [--envs 16]
#   --watch = 창 보기 (--visualizer kit). 512 env는 느리니 16~32 권장.
# 로그: isaac/logs/rl_goal/<날짜_시간>/
set -e
ITERS=300
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then ITERS=$1; shift; fi
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/rl/wheel_drift/train.py \
  --task goal --max-iterations "$ITERS" "$@"
