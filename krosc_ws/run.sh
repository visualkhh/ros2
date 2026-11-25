#!/usr/bin/env bash
# 실기 Pi에서 krosc_ws를 최신 빌드한 뒤 실행한다.
#   cd ~/ros2 && ./krosc_ws/run.sh
#   OPENCR_PORT=/dev/opencr LIDAR_PORT=/dev/tb3_lidar KROSC_SERVER_PORT=9000 ./krosc_ws/run.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"

"$here/build.sh" robot
exec "$here/build/krosc_real" "$@"
