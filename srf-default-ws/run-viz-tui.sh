#!/usr/bin/env bash
# 터미널 Viz를 빌드/실행해 로봇(기본 pak.local:9000)에 붙는다. Pi에서도, SSH 안에서도 된다.
#   ./run-viz-tui.sh
#   SRF_DEFAULT_WS_VIZ_HOST=127.0.0.1 ./run-viz-tui.sh --plugins topics,log
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=paths.sh
source "$here/paths.sh"
host="${SRF_DEFAULT_WS_VIZ_HOST:-pak.local}"
robot_port="${SRF_DEFAULT_WS_VIZ_PORT:-9000}"

"$here/build.sh" viz-tui
exec "$SRF_BUILD_DIR/viz/tui/srf_viz_tui" --robot "$host:$robot_port" "$@"
