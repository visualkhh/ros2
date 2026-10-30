#!/bin/bash
# real goal square 주행 (PC). 먼저 real_launch.sh로 goal node를 띄운다.
#   ./training/rl_correction_wheel_drift/real_square.sh [--side 0.5] [--out ...]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ISAAC=$(cd "$HERE/../.." && pwd)
REPO=$(cd "$ISAAC/.." && pwd)
source /opt/ros/lyrical/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=25
source "$REPO/tool/peers.sh"
exec python3 "$HERE/real_square.py" "$@"
