#!/bin/bash
# 체크포인트 -> ONNX (export/rl_correct/<이름>.onnx)
#   ./rl/export.sh <체크포인트> <이름.onnx>
# 예: ./rl/export.sh logs/rl_correct/<날짜_시간>/model_299.pt rl_correct_v2.onnx
set -e
CKPT=${1:?checkpoint .pt 경로}
OUT=${2:?출력 파일명}
[[ "$CKPT" != /* ]] && CKPT="/home/pank/workspaces/visualkhh/source/ros2/isaac/$CKPT"
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/rl/export.py --checkpoint "$CKPT" --out "$OUT"
