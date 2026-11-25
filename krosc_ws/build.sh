#!/usr/bin/env bash
# krosc + krosc_ws 빌드 단일 진입점.
# 실행 스크립트(run.sh/run-slam.sh/run-sim.sh/run-tests.sh/run-viz.sh/run-viz-web.sh)는
# 항상 이 스크립트를 먼저 호출해서 최신 바이너리를 보장한다.
#
#   ./build.sh                 # 현재 머신에 맞는 전체 (Pi: robot+viz-web, PC: robot+sim+viz+viz-web)
#   ./build.sh robot           # krosc_real
#   ./build.sh sim             # krosc_simulator (PC 전용)
#   ./build.sh tests           # krosc_core_tests + krosc_simulator_tests + krosc_ws_tests
#   ./build.sh viz             # krosc_ws_viz_app (PC 전용, Qt6)
#   ./build.sh viz-web         # krosc_ws_viz_web (Pi/PC)
#
# 환경변수:
#   KROSC_JOBS=2   빌드 병렬 수 (기본: Pi/ARM=2, 그 외 nproc)
#   KROSC_PULL=1   빌드 전에 git pull --ff-only
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
target="${1:-all}"

jobs="${KROSC_JOBS:-}"
if [ -z "$jobs" ]; then
  case "$(uname -m)" in
    aarch64|arm*|armv*) jobs=2 ;;
    *) jobs="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)" ;;
  esac
fi

is_arm() {
  case "$(uname -m)" in
    aarch64|arm*|armv*) return 0 ;;
    *) return 1 ;;
  esac
}

require_qt() {
  if ! command -v pkg-config >/dev/null 2>&1 || ! pkg-config --exists Qt6Widgets; then
    echo "[build] Qt6 Widgets가 필요하다 (viz/sim은 PC에서 빌드한다)." >&2
    exit 1
  fi
}

require_pc() {
  if is_arm; then
    echo "[build] $1 는 PC에서만 빌드한다. Pi에서는 robot/viz-web만 쓴다." >&2
    exit 1
  fi
}

if [ "${KROSC_PULL:-0}" = "1" ]; then
  echo "[build] git pull --ff-only"
  git -C "$root" pull --ff-only
fi

configure_krosc() {
  cmake -S "$root/krosc" -B "$root/krosc/build" \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
}

configure_ws() {
  cmake -S "$here" -B "$here/build" \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo -DKROSC_WS_BUILD_TESTS=ON
}

build_robot() {
  echo "[build] krosc_ws: krosc_real"
  configure_ws
  cmake --build "$here/build" -j"$jobs" --target krosc_real
}

build_sim() {
  require_pc sim
  require_qt
  echo "[build] krosc: krosc_simulator + viz"
  configure_krosc -DKROSC_BUILD_VIZ=ON -DKROSC_BUILD_EXAMPLES=ON
  cmake --build "$root/krosc/build" -j"$jobs" --target krosc_simulator
}

build_tests() {
  echo "[build] krosc: core/sim tests"
  if is_arm; then
    configure_krosc -DKROSC_BUILD_TESTS=ON -DKROSC_BUILD_VIZ=OFF -DKROSC_BUILD_EXAMPLES=OFF
  else
    configure_krosc -DKROSC_BUILD_TESTS=ON
  fi
  cmake --build "$root/krosc/build" -j"$jobs" --target krosc_core_tests krosc_simulator_tests
  echo "[build] krosc_ws: tests"
  configure_ws
  cmake --build "$here/build" -j"$jobs" --target krosc_ws_tests
}

build_viz() {
  require_pc viz
  require_qt
  echo "[build] krosc: krosc_ws_viz_app (Qt6)"
  configure_krosc -DKROSC_BUILD_VIZ=ON -DKROSC_BUILD_EXAMPLES=OFF \
                  -DKROSC_VIZ_PROJECT_DIR="$here/viz"
  cmake --build "$root/krosc/build" -j"$jobs" --target krosc_ws_viz_app
}

build_viz_web() {
  echo "[build] krosc: krosc_ws_viz_web"
  if is_arm; then
    configure_krosc -DKROSC_BUILD_VIZ_WEB=ON -DKROSC_BUILD_VIZ=OFF \
                    -DKROSC_BUILD_EXAMPLES=OFF \
                    -DKROSC_VIZ_WEB_PROJECT_DIR="$here/viz-web"
  else
    configure_krosc -DKROSC_BUILD_VIZ_WEB=ON -DKROSC_BUILD_EXAMPLES=OFF \
                    -DKROSC_VIZ_WEB_PROJECT_DIR="$here/viz-web"
  fi
  cmake --build "$root/krosc/build" -j"$jobs" --target krosc_ws_viz_web
}

case "$target" in
  all)
    if is_arm; then
      build_robot
      build_viz_web
    else
      build_robot
      build_sim
      build_viz
      build_viz_web
    fi
    ;;
  robot) build_robot ;;
  sim) build_sim ;;
  tests) build_tests ;;
  viz) build_viz ;;
  viz-web|viz_web) build_viz_web ;;
  *)
    echo "usage: $0 [all|robot|sim|tests|viz|viz-web]" >&2
    exit 2
    ;;
esac

echo "[build] 완료 ($target, jobs=$jobs)"
