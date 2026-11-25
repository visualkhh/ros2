#!/usr/bin/env bash
# PC에서 C++ viz를 최신 빌드한 뒤 Pi(기본 pak.local:9000)에 붙인다. 지도 프레임은 map.
#   ./run-viz.sh
#   KROSC_VIZ_HOST=10.2.150.135 KROSC_VIZ_PORT=9000 KROSC_VIZ_FIXED_FRAME=map ./run-viz.sh
#   ./run-viz.sh --screenshot /tmp/viz.png --duration 3
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
host="${KROSC_VIZ_HOST:-pak.local}"
port="${KROSC_VIZ_PORT:-9000}"
frame="${KROSC_VIZ_FIXED_FRAME:-map}"
tab="${KROSC_VIZ_TAB:-map}"
plugins="${KROSC_VIZ_PLUGINS:-project_panel,slam,navigation,moveit,rl_nav}"

"$here/build.sh" viz

echo "[run-viz] $host:$port fixed=$frame tab=$tab plugins=$plugins (teleop/manipulator: --teleop/--manipulator/--manual)"
plugin_args=()
if [ -n "$plugins" ] && [ "$plugins" != "none" ]; then
  plugin_args=(--plugins "$plugins")
else
  plugin_args=(--no-plugins)
fi
exec "$root/krosc/build/viz/krosc_ws_viz_app" --server "$host:$port" --fixed-frame "$frame" --tab "$tab" --manual "${plugin_args[@]}" "$@"
