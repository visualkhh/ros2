#!/usr/bin/env bash
# 실기 TurtleBot3 실행 (kros 기반, 시뮬레이터/화면 없음). 라즈베리파이에서:
#   cd ~/ros2 && ./kos_ws/run.sh                    # 0.0.0.0:9000 (PC Viz 접속)
#   KROS_SERVER_PORT=0 ./kos_ws/run.sh               # 네트워크 없이
#   OPENCR_PORT=/dev/opencr LIDAR_PORT=/dev/tb3_lidar ./kos_ws/run.sh
set -e
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
cd "$root"
python="$here/.venv/bin/python"
[ -x "$python" ] || python="$root/kros/.venv/bin/python"
[ -x "$python" ] || python="python3"
exec "$python" -m kos_ws "$@"
