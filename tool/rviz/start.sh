#!/usr/bin/env bash
# tool/rviz 시작 — PC에서 로봇(Pi)의 /map·/scan·TF를 본다.
#   ./start.sh            # slam.rviz (맵핑 보기: 지도·스캔·odom·포즈 그래프)
#   ./start.sh nav        # nav.rviz  (주행 보기: 지도·코스트맵·경로 · 2D Pose Estimate / Goal)
#   ROBOT_DOMAIN_ID=77 ./start.sh   # 시험용 (기본 25)
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../ros2_ws" 2>/dev/null && pwd || true)"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"

preset="${1:-slam}"
case "$preset" in
  slam|nav) [ $# -gt 0 ] && shift ;;
  *) preset=slam ;;
esac
config="$HERE/$preset.rviz"

if [ ! -f "$ROS_SETUP" ]; then
  echo "FAIL: ROS 2 setup 없음: $ROS_SETUP" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [ -n "$WS" ] && [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi

if ! command -v rviz2 >/dev/null; then
  echo "FAIL: rviz2 없음 — ROS 2 RViz를 설치하세요" >&2
  exit 1
fi

# 이 프로젝트는 ROS_DOMAIN_ID=25로 통일 (로봇 Pi · PC · docker). ~/.bashrc 값과 무관하게 25.
# 시험용으로 다른 값이 필요하면 ROBOT_DOMAIN_ID=77 ./run-cli ...
export ROS_DOMAIN_ID="${ROBOT_DOMAIN_ID:-25}"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

if [ "${ROS_LOCALHOST_ONLY:-0}" = "1" ] || [ "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" = "LOCALHOST" ]; then
  echo "WARN: localhost 전용 설정 — 로봇과 통신 불가 (ROS_LOCALHOST_ONLY=0 / ROS_AUTOMATIC_DISCOVERY_RANGE 해제)" >&2
fi

echo "RViz: $config"
exec rviz2 -d "$config" "$@"
