#!/usr/bin/env bash
# krosc_ws 프로젝트 웹 Viz 게이트웨이를 PC에서 빌드/실행해 Pi(기본 pak.local:9000)에 붙인다.
#   ./run-viz-web.sh
#   KROSC_VIZ_HOST=10.2.150.138 KROSC_VIZ_WEB_PORT=8080 ./run-viz-web.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
host="${KROSC_VIZ_HOST:-pak.local}"
robot_port="${KROSC_VIZ_PORT:-9000}"
port="${KROSC_VIZ_WEB_PORT:-8080}"
bind="${KROSC_VIZ_WEB_BIND:-0.0.0.0}"

"$here/build.sh" viz-web

echo "[run-viz-web] http://localhost:$port/  (robot $host:$robot_port)"
exec "$root/krosc/build/viz-web/krosc_ws_viz_web" --robot "$host:$robot_port" \
     --port "$port" --bind "$bind" "$@"
