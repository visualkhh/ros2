#!/bin/bash
# PC에서 RL corrector 노드 실행 (ONNX 경로).
#
#   ./training/real_rl.sh <model.onnx> [goal|track] [task_dir]
#
# 예:
#   ./training/real_rl.sh \
#     logs/rl_goal_friction/<날짜_시간>/rl_goal_friction.onnx goal
#
# Pi는 먼저:
#   ssh pak@pak.local 'cd ~/ros2/tool && ./run-cli route rl'
#
# task_dir 기본: rl_correction_wheel_drift
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ISAAC=$(cd "$HERE/.." && pwd)
MODEL=${1:?ONNX 파일 경로}
MODE=${2:-goal}
TASK=${3:-rl_correction_wheel_drift}
[[ "$TASK" = /* ]] || TASK="$HERE/$TASK"
[[ -f "$TASK/real_launch.sh" ]] || { echo "real_launch.sh 없음: $TASK" >&2; exit 2; }
[[ "$MODEL" = /* ]] || MODEL="$ISAAC/$MODEL"
[[ -f "$MODEL" ]] || { echo "ONNX 없음: $MODEL" >&2; exit 2; }

exec "$TASK/real_launch.sh" "$MODEL" "$MODE"
