#!/usr/bin/env bash
# PC에서 C++ viz를 최신 빌드한 뒤 Pi(기본 pak.local:9000)에 붙인다. 지도 프레임은 map.
#   ./run-viz.sh
#   SRF_DEFAULT_WS_VIZ_HOST=10.2.150.135 SRF_DEFAULT_WS_VIZ_PORT=9000 SRF_DEFAULT_WS_VIZ_FIXED_FRAME=map ./run-viz.sh
#   ./run-viz.sh --screenshot /tmp/viz.png --duration 3
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
# shellcheck source=paths.sh
source "$here/paths.sh"
host="${SRF_DEFAULT_WS_VIZ_HOST:-pak.local}"
port="${SRF_DEFAULT_WS_VIZ_PORT:-9000}"
frame="${SRF_DEFAULT_WS_VIZ_FIXED_FRAME:-map}"
tab="${SRF_DEFAULT_WS_VIZ_TAB:-map}"
plugins="${SRF_DEFAULT_WS_VIZ_PLUGINS:-project_panel,slam,navigation,moveit,rl_nav,opencr,camera}"

"$here/build.sh" viz

echo "[run-viz] $host:$port fixed=$frame tab=$tab plugins=$plugins (teleop/manipulator: --teleop/--manipulator/--manual)"
plugin_args=()
if [ -n "$plugins" ] && [ "$plugins" != "none" ]; then
  plugin_args=(--plugins "$plugins")
else
  plugin_args=(--no-plugins)
fi
exec "$SRF_BUILD_DIR/viz/qt/srf_default_ws_viz_app" --server "$host:$port" --fixed-frame "$frame" --tab "$tab" --manual "${plugin_args[@]}" "$@"
