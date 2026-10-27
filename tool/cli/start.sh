#!/usr/bin/env bash
# tool/cli 시작 — ROS 2와 이 워크스페이스 설치를 source한 뒤 Python CLI로 넘긴다.
# set -u 금지 — ROS setup.bash가 미설정 변수를 참조한다 (start.sh와 같은 이유)
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../ros2_ws" 2>/dev/null && pwd || true)"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
  echo "FAIL: ROS 2 setup 없음: $ROS_SETUP" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [ -n "$WS" ] && [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
else
  echo "WARN: ros2_ws/install/setup.bash 없음 — 먼저 ros2_ws에서 colcon build" >&2
fi

# 이 프로젝트는 ROS_DOMAIN_ID=25로 통일 (로봇 Pi · PC · docker). ~/.bashrc 값과 무관하게 25.
export ROS_DOMAIN_ID=25
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
# PC에서 실행할 때: 로봇 Pi를 유니캐스트로 직접 찾는다. 사내 WiFi는 멀티캐스트(DDS 자동 탐색)를 자주 버려서
# 노드가 보였다 안 보였다 함 (2026-10-27 실측: ping은 되는데 ros2 node list가 빔). 한쪽만 지정해도 양방향 연결됨.
# Pi 자신에서 실행할 때는 필요 없음. 로봇 이름이 다르면 ROBOT_HOST=다른이름.local
if [ "$(hostname)" != "${ROBOT_HOST:-pak.local}" ] && [ "$(hostname).local" != "${ROBOT_HOST:-pak.local}" ]; then
  export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-${ROBOT_HOST:-pak.local}}"
  echo "ROS_STATIC_PEERS=$ROS_STATIC_PEERS"
fi

exec python3 -u "$HERE/robot_tool.py" "$@"
