#!/usr/bin/env bash
# krosc(core/sim)와 krosc_ws 테스트를 빌드하고 실행한다.
#   ./run-tests.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
jobs="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"

echo "[tests] krosc core/sim"
cmake -S "$root/krosc" -B "$root/krosc/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DKROSC_BUILD_VIZ=OFF -DKROSC_BUILD_EXAMPLES=OFF -DKROSC_BUILD_BENCH=OFF
cmake --build "$root/krosc/build" -j"$jobs"
ctest --test-dir "$root/krosc/build" --output-on-failure

echo "[tests] krosc_ws"
cmake -S "$here" -B "$here/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DKROSC_WS_BUILD_TESTS=ON
cmake --build "$here/build" -j"$jobs"
ctest --test-dir "$here/build" --output-on-failure
