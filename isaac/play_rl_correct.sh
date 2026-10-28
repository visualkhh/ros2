#!/bin/bash
# 학습된 정책 구경 (뷰포트)
#   ./play_rl_correct.sh [onnx경로] [env수]
# 예: ./play_rl_correct.sh export/rl_correct/rl_correct_v1.onnx 4
set -e
MODEL=${1:-export/rl_correct/rl_correct_v1.onnx}
ENVS=${2:-4}
[[ "$MODEL" != /* ]] && MODEL="/home/pank/workspaces/visualkhh/source/ros2/isaac/$MODEL"
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/play.py \
  --task rl_correct --model "$MODEL" --envs "$ENVS" --visualizer kit
