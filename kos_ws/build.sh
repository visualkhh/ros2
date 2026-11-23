#!/bin/bash
# kos_ws 빌드 — ros2/ros2_ws를 underlay로 쓰는 overlay 워크스페이스.
#
#   ./build.sh            # src/ 아래 패키지를 빌드한다 (underlay source 후)
#   ./build.sh --clean    # build/install/log를 지우고 다시 빌드
set -eo pipefail
WS="$(cd "$(dirname "$0")" && pwd)"
UNDERLAY="${ROS2_UNDERLAY:-$HOME/ros2/ros2_ws}"

if [[ "${1:-}" == "--clean" ]]; then
  rm -rf "$WS/build" "$WS/install" "$WS/log"
fi

# underlay (실제 ROS 패키지) source
if [[ -f "$UNDERLAY/install/setup.bash" ]]; then
  set +u
  source "$UNDERLAY/install/setup.bash"
  set -u
else
  echo "underlay가 없다: $UNDERLAY/install/setup.bash (먼저 ros2_ws를 빌드하세요)" >&2
  exit 2
fi

cd "$WS"
colcon build --symlink-install --merge-install "$@"
