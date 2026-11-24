#!/usr/bin/env bash
# PC에서 krosc 시뮬레이터(진실) + Viz 데모를 빌드하고 실행한다. Qt6가 필요하다.
#   ./run-sim.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
jobs="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"

if ! command -v pkg-config >/dev/null 2>&1 || ! pkg-config --exists Qt6Widgets; then
  echo "Qt6 Widgets가 필요하다 (시뮬레이터 창은 PC에서 실행한다)." >&2
  exit 1
fi

cmake -S "$root/krosc" -B "$root/krosc/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DKROSC_BUILD_TESTS=OFF -DKROSC_BUILD_BENCH=OFF -DKROSC_BUILD_VIZ=ON -DKROSC_BUILD_EXAMPLES=ON
cmake --build "$root/krosc/build" --target krosc_turtlebot3 -j"$jobs"

exec "$root/krosc/build/examples/turtlebot3/krosc_turtlebot3" "$@"
