#!/usr/bin/env bash
# PC에서 srf_recycling_ws 로봇 스택(ControlManager/SensorManager/ControllerManager/nav2)을
# 재활용 미션 월드(작업 영역·분리수거함·임의 배치 재활용품) 위에서 실행한다. Pi에서는 빌드하지 않는다.
#   ./run-simulator.sh
#   ./run-simulator.sh --seed 7 --scene multi --target can --items 5
#   ./run-simulator.sh --scene ambiguous --camera 640x480 --no-noise
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
# shellcheck source=paths.sh
source "$here/paths.sh"

"$here/build.sh" simulator
exec "$SRF_RECYCLING_WS_BUILD_DIR/srf_recycling_ws_simulator" "$@"
