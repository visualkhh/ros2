#!/usr/bin/env bash
# PC에서 krosc_ws 로봇 스택(ControlManager/SensorManager/ControllerManager/nav2)을
# krosc/simulator 월드 위에서 실행한다. Pi에서는 빌드하지 않는다.
#   ./run-simulator.sh
#   ./run-simulator.sh --port 9000 --pucks 10 --no-noise
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
# shellcheck source=paths.sh
source "$here/paths.sh"

"$here/build.sh" simulator
exec "$KROSC_WS_BUILD_DIR/krosc_ws_simulator" "$@"
