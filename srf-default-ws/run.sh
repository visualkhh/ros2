#!/usr/bin/env bash
# 실기 Pi에서 srf_default_ws를 최신 빌드한 뒤 실행한다.
#
#   cd ~/ros2 && ./srf-default-ws/run.sh          # SLAM 없이 시작 (idle) — Viz에서 start SLAM으로 켠다
#   cd ~/ros2 && ./srf-default-ws/run-slam.sh     # 시작하자마자 mapping
#   cd ~/ros2 && ./srf-default-ws/stop.sh         # 정지
#
#   OPENCR_PORT=/dev/opencr LIDAR_PORT=/dev/tb3_lidar SRF_DEFAULT_WS_SERVER_PORT=9000 ./srf-default-ws/run.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"

"$here/build.sh" robot
exec "$here/build/srf_default_ws_real" "$@"
