#!/bin/bash
# 체크포인트 -> ONNX export (export/<task>/<이름>.onnx)
#   ./export_rl.sh rl_correct <체크포인트> <이름.onnx>
# 예: ./export_rl.sh rl_correct logs/rl_correct/20261028_1817/model_29.pt rl_correct_v1.onnx
set -e
TASK=${1:?task: rl_correct|nav2_rl}
CKPT=${2:?checkpoint .pt 경로}
OUT=${3:?출력 파일명}
[[ "$CKPT" != /* ]] && CKPT="/home/pank/workspaces/visualkhh/source/ros2/isaac/$CKPT"
cd /home/pank/pa-course/IsaacLab-6.1
./isaaclab.sh -p /home/pank/workspaces/visualkhh/source/ros2/isaac/scripts/export.py \
  --task "$TASK" --checkpoint "$CKPT" --out "$OUT"
