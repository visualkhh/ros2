#!/usr/bin/env bash
# srf + srf_recycling_ws 빌드 단일 진입점.
# 실행 스크립트(run-simulator.sh/run-tests.sh/run-viz-tui.sh)는
# 항상 이 스크립트를 먼저 호출해서 최신 바이너리를 보장한다.
#
#   ./build.sh                 # 현재 머신에 맞는 전체 (Pi: robot+tools, PC: robot+tools+simulator)
#   ./build.sh robot           # srf_recycling_ws_real
#   ./build.sh simulator       # srf_recycling_ws_simulator + srf_recycling_ws_snapshot (PC 전용, Qt)
#   ./build.sh tests           # srf_core_tests + srf_simulator_tests + srf_bag_tests + srf_recycling_ws_tests (+ srf CLI)
#   ./build.sh viz-tui         # srf_viz_tui (터미널 Viz, Pi에서도 된다)
#   ./build.sh tools           # srf 명령 (srf bag / srf topic) → $SRF_BUILD_DIR/tools/srf/srf
#
# 환경변수:
#   SRF_RECYCLING_WS_JOBS=2   빌드 병렬 수 (기본: Pi/ARM=2, 그 외 nproc)
#   SRF_RECYCLING_WS_PULL=1   빌드 전에 git pull --ff-only
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
# shellcheck source=paths.sh
source "$here/paths.sh"
target="${1:-all}"

jobs="${SRF_RECYCLING_WS_JOBS:-}"
if [ -z "$jobs" ]; then
  case "$(uname -m)" in
    aarch64|arm*|armv*) jobs=2 ;;
    *) jobs="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)" ;;
  esac
fi

is_arm() {
  # Apple Silicon(arm64)은 PC다. Pi/Linux ARM만 로봇 전용으로 본다.
  case "$(uname -s):$(uname -m)" in
    Linux:aarch64|Linux:arm*|Linux:armv*) return 0 ;;
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
    echo "[build] $1 는 PC에서만 빌드한다. Pi는 로봇(srf_recycling_ws_real)만 빌드한다." >&2
    exit 1
  fi
}

if [ "${SRF_RECYCLING_WS_PULL:-0}" = "1" ]; then
  echo "[build] git pull --ff-only"
  git -C "$root" pull --ff-only
fi

configure_srf() {
  cmake -S "$root/srf" -B "$SRF_BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
}

configure_ws() {
  local -a ort_args=()
  local -a rs_args=()
  local ort_root="${SRF_RECYCLING_WS_ONNXRUNTIME_ROOT:-${ONNXRUNTIME_ROOT:-$HOME/opt/onnxruntime-gpu/1.26.0}}"
  if [ -d "$ort_root" ]; then
    export ONNXRUNTIME_ROOT="$ort_root"
    ort_args+=(-DSRF_RECYCLING_WS_ONNXRUNTIME_ROOT="$ort_root"
               -UONNXRUNTIME_INCLUDE_DIR -UONNXRUNTIME_LIBRARY)
  fi
  if [ -n "${SRF_RECYCLING_WS_REALSENSE_ROOT:-}" ]; then
    rs_args+=(-DSRF_RECYCLING_WS_REALSENSE_ROOT="$SRF_RECYCLING_WS_REALSENSE_ROOT")
  fi
  cmake -S "$here" -B "$SRF_RECYCLING_WS_BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSRF_RECYCLING_WS_BUILD_TESTS=ON \
        ${ort_args[@]+"${ort_args[@]}"} ${rs_args[@]+"${rs_args[@]}"}
}

build_robot() {
  echo "[build] srf_recycling_ws: srf_recycling_ws_real"
  configure_ws
  cmake --build "$SRF_RECYCLING_WS_BUILD_DIR" -j"$jobs" --target srf_recycling_ws_real
}

build_simulator() {
  require_pc simulator
  require_qt
  echo "[build] srf_recycling_ws: srf_recycling_ws_simulator"
  configure_ws
  cmake --build "$SRF_RECYCLING_WS_BUILD_DIR" -j"$jobs" --target srf_recycling_ws_simulator srf_recycling_ws_snapshot
}

build_tests() {
  echo "[build] srf: core/sim tests"
  if is_arm; then
    configure_srf -DSRF_BUILD_TESTS=ON -DSRF_BUILD_VIZ=OFF -DSRF_BUILD_EXAMPLES=OFF
  else
    configure_srf -DSRF_BUILD_TESTS=ON
  fi
  cmake --build "$SRF_BUILD_DIR" -j"$jobs" --target srf_core_tests srf_simulator_tests srf_bag_tests srf_cli
  echo "[build] srf_recycling_ws: tests"
  configure_ws
  cmake --build "$SRF_RECYCLING_WS_BUILD_DIR" -j"$jobs" --target srf_recycling_ws_tests
}

build_tools() {
  echo "[build] srf: srf 명령 (bag, topic)"
  if is_arm; then
    configure_srf -DSRF_BUILD_VIZ=OFF -DSRF_BUILD_EXAMPLES=OFF
  else
    configure_srf
  fi
  cmake --build "$SRF_BUILD_DIR" -j"$jobs" --target srf_cli
  echo "[build] srf 명령: $SRF_BUILD_DIR/tools/srf/srf  (PATH에 넣으려면: cmake --install $SRF_BUILD_DIR --prefix ~/.local)"
}

build_viz_tui() {
  echo "[build] srf: srf_viz_tui (터미널)"
  if is_arm; then
    configure_srf -DSRF_BUILD_VIZ_TUI=ON -DSRF_BUILD_VIZ=OFF -DSRF_BUILD_EXAMPLES=OFF
  else
    configure_srf -DSRF_BUILD_VIZ_TUI=ON -DSRF_BUILD_EXAMPLES=OFF
  fi
  cmake --build "$SRF_BUILD_DIR" -j"$jobs" --target srf_viz_tui
}

case "$target" in
  all)
    if is_arm; then
      # Pi는 하드웨어만 돌린다. 웹/Qt Viz는 PC에서 빌드해 9000으로 붙는다.
      build_robot
      build_tools
    else
      build_robot
      build_tools
      build_simulator
    fi
    ;;
  robot) build_robot ;;
  simulator) build_simulator ;;
  tests) build_tests ;;
  viz-tui|viz_tui) build_viz_tui ;;
  tools) build_tools ;;
  *)
    echo "usage: $0 [all|robot|simulator|tests|viz-tui|tools]" >&2
    exit 2
    ;;
esac

echo "[build] 완료 ($target, jobs=$jobs)"
