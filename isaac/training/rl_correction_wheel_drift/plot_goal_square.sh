#!/bin/bash
# real RL square vs no-RL square 비교 그래프
#   ./training/rl_correction_wheel_drift/plot_goal_square.sh [--show]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
exec python3 "$HERE/plot_goal_square.py" "$@"
