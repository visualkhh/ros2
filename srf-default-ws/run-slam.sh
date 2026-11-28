#!/usr/bin/env bash
# 실기 Pi에서 srf_default_ws를 최신 빌드한 뒤 SLAM(mapping)으로 실행한다.
#   ./run-slam.sh
#   OPENCR_PORT=/dev/opencr LIDAR_PORT=/dev/tb3_lidar SRF_DEFAULT_WS_SERVER_PORT=9000 ./run-slam.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
exec "$here/run.sh" --slam "$@"
