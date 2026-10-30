#!/bin/bash
# goal + env별 바닥 마찰 randomization 학습
#   ./training/rl_correction_wheel_drift/train_goal_friction.sh [max_iterations] [--watch] [--envs 16]
#   파랑=미끄럼 / 주황=끈적 국소 패치 (기본 4개, --patch-count N)
# 로그: isaac/logs/rl_goal_friction/<날짜_시간>/
set -e
ITERS=300
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then ITERS=$1; shift; fi
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/training/rl_correction_wheel_drift/train.py \
  --task goal --friction --exp-name rl_goal_friction --max-iterations "$ITERS" "$@"
