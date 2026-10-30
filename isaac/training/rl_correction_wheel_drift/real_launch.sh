#!/bin/bash
# real goal RL 노드 실행 (PC). ONNX 경로를 넘긴다.
#   ./training/rl_correction_wheel_drift/real_launch.sh <model.onnx> [goal|track]
# 예:
#   ./training/rl_correction_wheel_drift/real_launch.sh \
#     logs/rl_goal_friction/20261030_190913/rl_goal_friction.onnx goal
#
# Pi는 별도로:
#   ssh pak@pak.local 'cd ~/ros2/tool && ./run-cli route rl'
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ISAAC=$(cd "$HERE/../.." && pwd)
REPO=$(cd "$ISAAC/.." && pwd)
MODEL=${1:?ONNX 파일 경로}
MODE=${2:-goal}
[[ "$MODEL" != /* ]] && MODEL="$ISAAC/$MODEL"
[ -f "$MODEL" ] || { echo "ONNX 없음: $MODEL" >&2; exit 2; }
[ -f "$REPO/ros2_ws/install/setup.bash" ] || {
  echo "먼저 빌드: cd $REPO/ros2_ws && colcon build --packages-select rl_correction tb3_bringup" >&2
  exit 2
}

source /opt/ros/lyrical/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=25
source "$REPO/tool/peers.sh"

exec ros2 launch tb3_bringup rl-only.launch.py \
  enable:=true mode:="$MODE" goal_mode:=absolute model:="$MODEL"
