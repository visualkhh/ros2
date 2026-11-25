#!/usr/bin/env bash
# PC에서 krosc 시뮬레이터(진실) + Viz 데모를 최신 빌드한 뒤 실행한다. Qt6가 필요하다.
#   ./run-sim.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"

"$here/build.sh" sim
exec "$root/krosc/build/examples/turtlebot3/krosc_turtlebot3" "$@"
