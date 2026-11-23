#!/usr/bin/env bash
# 로컬 Qt6 GUI로 TurtleBot3 데모를 실행한다 (Docker가 아니라 호스트에서).
#   ./examples/turtlebot3/run.sh
#   ./examples/turtlebot3/run.sh --screenshot docs/screenshot.png --duration 4 --headless
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../.." && pwd)"
build="${KROSC_BUILD_DIR:-$root/build}"
if [ ! -x "$build/examples/turtlebot3/krosc_turtlebot3" ]; then
  echo "빌드가 없다: $build/examples/turtlebot3/krosc_turtlebot3" >&2
  echo "먼저: cmake -S \"$root\" -B \"$build\" -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build \"$build\" -j" >&2
  exit 1
fi
exec "$build/examples/turtlebot3/krosc_turtlebot3" "$@"
