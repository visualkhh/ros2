#!/usr/bin/env bash
# PC에서 krosc_ws 프로젝트 웹 Viz를 빌드하고 Pi(기본 pak.local:9000)에 붙인다.
#   ./run-viz-web.sh
#   KROSC_VIZ_HOST=10.2.150.138 KROSC_VIZ_WEB_PORT=8080 ./run-viz-web.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
jobs="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
host="${KROSC_VIZ_HOST:-pak.local}"
robot_port="${KROSC_VIZ_PORT:-9000}"
port="${KROSC_VIZ_WEB_PORT:-8080}"
bind="${KROSC_VIZ_WEB_BIND:-0.0.0.0}"

cmake -S "$root/krosc" -B "$root/krosc/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DKROSC_BUILD_VIZ_WEB=ON -DKROSC_BUILD_EXAMPLES=OFF \
      -DKROSC_VIZ_WEB_PROJECT_DIR="$here/viz-web"
cmake --build "$root/krosc/build" --target krosc_ws_viz_web -j"$jobs"

echo "[run-viz-web] http://localhost:$port/  (robot $host:$robot_port)"
exec "$root/krosc/build/viz-web/krosc_ws_viz_web" --robot "$host:$robot_port" \
     --port "$port" --bind "$bind" "$@"
