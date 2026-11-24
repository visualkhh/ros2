#!/usr/bin/env bash
# PC에서 C++ viz를 빌드하고 Pi(기본 pak.local:9000)에 붙인다. 지도 프레임은 map.
#   ./run-viz.sh
#   KROSC_VIZ_HOST=10.2.150.135 KROSC_VIZ_PORT=9000 KROSC_VIZ_FIXED_FRAME=map ./run-viz.sh
#   ./run-viz.sh --screenshot /tmp/viz.png --duration 3
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
jobs="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
host="${KROSC_VIZ_HOST:-pak.local}"
port="${KROSC_VIZ_PORT:-9000}"
frame="${KROSC_VIZ_FIXED_FRAME:-map}"
tab="${KROSC_VIZ_TAB:-map}"

if ! command -v pkg-config >/dev/null 2>&1 || ! pkg-config --exists Qt6Widgets; then
  echo "Qt6 Widgets가 필요하다 (viz는 PC에서 실행한다)." >&2
  exit 1
fi

cmake -S "$root/krosc" -B "$root/krosc/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DKROSC_BUILD_TESTS=OFF -DKROSC_BUILD_BENCH=OFF -DKROSC_BUILD_VIZ=ON -DKROSC_BUILD_EXAMPLES=OFF
cmake --build "$root/krosc/build" --target krosc_viz_app -j"$jobs"

echo "[run-viz] $host:$port fixed=$frame tab=$tab"
exec "$root/krosc/build/viz/krosc_viz_app" --server "$host:$port" --fixed-frame "$frame" --tab "$tab" --manual "$@"
