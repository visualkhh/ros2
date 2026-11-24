#!/usr/bin/env bash
# krosc + krosc_ws를 한 번에 빌드한다 (실행은 하지 않는다).
#   ./build.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
jobs="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"

echo "[build] krosc (core/sim/viz)"
cmake -S "$root/krosc" -B "$root/krosc/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DKROSC_BUILD_VIZ=ON -DKROSC_BUILD_EXAMPLES=ON
cmake --build "$root/krosc/build" -j"$jobs"

echo "[build] krosc_ws (실기)"
cmake -S "$here" -B "$here/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$here/build" -j"$jobs"

echo "[build] 완료"
