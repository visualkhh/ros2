#!/bin/bash
# 체크포인트 -> ONNX (export/rl_correct/<이름>.onnx)
#   ./training/rl_correction_wheel_drift/export.sh <체크포인트> <이름.onnx> [track|goal]
# 예: ./training/rl_correction_wheel_drift/export.sh logs/rl_goal/<날짜_시간>/model_299.pt goal_v1.onnx goal
set -e
CKPT=${1:?checkpoint .pt 경로}
OUT=${2:?출력 파일명}
TASK=${3:-track}
[[ "$CKPT" != /* ]] && CKPT="/home/pank/workspaces/visualkhh/source/ros2/isaac/$CKPT"
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/training/rl_correction_wheel_drift/export.py \
  --checkpoint "$CKPT" --out "$OUT" --task "$TASK"
