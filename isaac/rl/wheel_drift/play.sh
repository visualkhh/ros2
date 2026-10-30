#!/bin/bash
# 학습된 정책 구경 (뷰포트)
#   ./rl/wheel_drift/play.sh <onnx경로> [env수]
# 예: ./rl/wheel_drift/play.sh export/rl_correct/rl_correct_v2.onnx 4
set -e
MODEL=${1:?onnx 경로 (v2: 입력 6개)}
ENVS=${2:-4}
[[ "$MODEL" != /* ]] && MODEL="/home/pank/workspaces/visualkhh/source/ros2/isaac/$MODEL"
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/rl/wheel_drift/play.py --model "$MODEL" --envs "$ENVS" --visualizer kit
