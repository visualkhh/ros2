#!/usr/bin/env bash
# 시뮬레이터(Qt 진실 뷰)와 Qt Viz(krosc_ws_viz_app)를 함께 실행한다.
#   ./run-simulator-vizqt.sh
#   ./run-simulator-vizqt.sh --pucks 10 --no-noise --port 9000
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"

robot_port="${KROSC_VIZ_PORT:-9000}"
args=("$@")
for ((i = 0; i < ${#args[@]}; ++i)); do
  if [[ "${args[$i]}" == "--port" && $((i + 1)) -lt ${#args[@]} ]]; then
    robot_port="${args[$((i + 1))]}"
  fi
done

"$here/build.sh" simulator
"$here/build.sh" viz

sim_pid=""
viz_pid=""
cleanup() {
  [[ -n "$viz_pid" ]] && kill "$viz_pid" 2>/dev/null || true
  [[ -n "$sim_pid" ]] && kill "$sim_pid" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$root/krosc_ws/build/krosc_ws_simulator" "$@" > /tmp/krosc_ws_simulator.log 2>&1 &
sim_pid=$!

# PeerNetwork가 열릴 때까지 잠깐 기다린다.
for _ in $(seq 1 30); do
  if (exec 3<>"/dev/tcp/127.0.0.1/$robot_port") 2>/dev/null; then
    exec 3>&- 3<&-
    break
  fi
  sleep 0.2
done

echo "[run-simulator-vizqt] simulator=127.0.0.1:$robot_port  (Qt Viz)"
"$root/krosc/build/viz/krosc_ws_viz_app" --server "127.0.0.1:$robot_port" \
  --fixed-frame map > /tmp/krosc_ws_simulator_vizqt.log 2>&1 &
viz_pid=$!

# 시뮬레이터가 끝나면(또는 Ctrl+C) 게이트웨이도 같이 정리한다.
status=0
wait "$sim_pid" || status=$?
cleanup
exit "$status"
