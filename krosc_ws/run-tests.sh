#!/usr/bin/env bash
# krosc(core/sim)와 krosc_ws 테스트를 최신 빌드한 뒤 실행한다.
#   ./run-tests.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
# shellcheck source=paths.sh
source "$here/paths.sh"

"$here/build.sh" tests

status=0
run_test() {
  local binary="$1"
  if [ ! -x "$binary" ]; then
    echo "[tests] 없음: $binary" >&2
    status=1
    return
  fi
  echo "[tests] $(basename "$binary")"
  if ! "$binary"; then
    status=1
  fi
}

run_test "$KROSC_BUILD_DIR/engine/krosc_core_tests"
run_test "$KROSC_BUILD_DIR/simulator/krosc_simulator_tests"
run_test "$KROSC_WS_BUILD_DIR/krosc_ws_tests"

exit "$status"
