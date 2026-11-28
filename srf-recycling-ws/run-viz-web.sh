#!/usr/bin/env bash
# srf_recycling_ws 프로젝트 웹 Viz 게이트웨이를 PC에서 빌드/실행해 Pi(기본 pak.local:9000)에 붙인다.
#   ./run-viz-web.sh
#   SRF_RECYCLING_WS_VIZ_HOST=10.2.150.138 SRF_RECYCLING_WS_VIZ_WEB_PORT=8080 ./run-viz-web.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
# shellcheck source=paths.sh
source "$here/paths.sh"
host="${SRF_RECYCLING_WS_VIZ_HOST:-pak.local}"
robot_port="${SRF_RECYCLING_WS_VIZ_PORT:-9000}"
port="${SRF_RECYCLING_WS_VIZ_WEB_PORT:-8080}"
bind="${SRF_RECYCLING_WS_VIZ_WEB_BIND:-0.0.0.0}"

"$here/build.sh" viz-web

echo "[run-viz-web] http://localhost:$port/  (robot $host:$robot_port)"
exec "$SRF_BUILD_DIR/viz/web/srf_recycling_ws_viz_web" --robot "$host:$robot_port" \
     --port "$port" --bind "$bind" "$@"
