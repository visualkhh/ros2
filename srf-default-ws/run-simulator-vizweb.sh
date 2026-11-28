#!/usr/bin/env bash
# 시뮬레이터(Qt 진실 뷰)와 웹 Viz 게이트웨이를 함께 실행한다.
#   ./run-simulator-vizweb.sh
#   ./run-simulator-vizweb.sh --pucks 10 --no-noise --port 9000
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
# shellcheck source=paths.sh
source "$here/paths.sh"

robot_port="${SRF_DEFAULT_WS_VIZ_PORT:-9000}"
web_port="${SRF_DEFAULT_WS_VIZ_WEB_PORT:-8080}"
args=("$@")
for ((i = 0; i < ${#args[@]}; ++i)); do
  if [[ "${args[$i]}" == "--port" && $((i + 1)) -lt ${#args[@]} ]]; then
    robot_port="${args[$((i + 1))]}"
  fi
done

"$here/build.sh" simulator
"$here/build.sh" viz-web

sim_pid=""
gateway_pid=""
cleanup() {
  [[ -n "$gateway_pid" ]] && kill "$gateway_pid" 2>/dev/null || true
  [[ -n "$sim_pid" ]] && kill "$sim_pid" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$SRF_DEFAULT_WS_BUILD_DIR/srf_default_ws_simulator" "$@" > /tmp/srf_default_ws_simulator.log 2>&1 &
sim_pid=$!

wait_tcp "$robot_port" 30 || true

echo "[run-simulator-vizweb] http://localhost:$web_port/  (simulator 127.0.0.1:$robot_port)"
"$SRF_BUILD_DIR/viz/web/srf_default_ws_viz_web" --robot "127.0.0.1:$robot_port" \
  --port "$web_port" --bind "${SRF_DEFAULT_WS_VIZ_WEB_BIND:-0.0.0.0}" > /tmp/srf_default_ws_simulator_vizweb.log 2>&1 &
gateway_pid=$!

status=0
wait "$sim_pid" || status=$?
cleanup
exit "$status"
