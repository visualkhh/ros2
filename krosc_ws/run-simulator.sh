#!/usr/bin/env bash
# PC에서 krosc_ws 로봇 스택(ControlManager/SensorManager/ControllerManager/nav2)을
# krosc/simulator 월드 위에서 실행한다. Pi에서는 빌드하지 않는다.
#   ./run-simulator.sh
#   ./run-simulator.sh --port 9000 --pucks 10 --no-noise
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"

"$here/build.sh" simulator
exec "$root/krosc_ws/build/krosc_ws_simulator" "$@"
