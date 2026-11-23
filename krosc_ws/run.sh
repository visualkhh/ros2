#!/usr/bin/env bash
# 실기 Pi에서 krosc_ws를 빌드하고 실행한다.
#   cd ~/ros2 && ./krosc_ws/run.sh
#   OPENCR_PORT=/dev/opencr LIDAR_PORT=/dev/tb3_lidar KROSC_SERVER_PORT=9000 ./krosc_ws/run.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
jobs="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
cmake -S "$here" -B "$here/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DKROSC_WS_BUILD_TESTS=OFF
cmake --build "$here/build" -j"$jobs"
exec "$here/build/krosc_real" "$@"
