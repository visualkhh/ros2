#!/usr/bin/env bash
# KOS sim 실행 (kros 기반 MPPI 시뮬레이터). venv는 kros/.venv를 쓴다.
set -e
here="$(cd "$(dirname "$0")" && pwd)"
kros="/home/pank/workspaces/visualkhh/source/ros2/kros"
cd /home/pank/workspaces/visualkhh/source/ros2
exec "$kros/.venv/bin/python" "$here/main.py" "$@"
EOF
chmod +x /home/pank/workspaces/visualkhh/source/ros2/kos_ws/run.sh