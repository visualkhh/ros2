#!/usr/bin/env bash
# base-only — 로봇(Pi)에서 베이스만 띄운다. = ros2 launch tb3_bringup tb-only.launch.py (포그라운드)
#
#   ./base-only.sh [launch 인자...]
#
#   ./base-only.sh                 state_manager · cmd_manager · OpenCR 드라이버 · robot_state_publisher · 라이다
#   ./base-only.sh lidar:=false    라이다 없이
#   ./base-only.sh port:=/dev/ttyACM0
#
# - Ctrl+C로 끝낸다. 백그라운드로 띄우고 관리하려면 tool/run-cli base start (같은 launch) — 둘을 같이 띄우지 말 것.
# - ROS_DOMAIN_ID=25.
#
#   -h, --help   이 도움말
set -eo pipefail   # set -u 금지 — ROS setup.bash가 미설정 변수를 참조한다
WS="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$WS/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,/^set -eo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
[[ -f "$WS/install/setup.bash" ]] || { echo "먼저 빌드: cd $WS && colcon build" >&2; exit 2; }

source "/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=25
source "$REPO/tool/peers.sh"   # Pi 자신에서는 아무것도 안 함

exec ros2 launch tb3_bringup tb-only.launch.py "$@"
