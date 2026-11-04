#!/usr/bin/env bash
# policy — 실행 중인 rl-only의 보정 정책을 바꾼다 (A/B를 한 번 실행 안에서). = ros2 param set /rl_corrector ...
#
#   ./policy.sh <정책.onnx>   그 ONNX로 바꾸고 켬 (model_path 다시 로드 + enable true)
#   ./policy.sh off           바이패스 — 입력 명령 그대로 통과 (enable false)
#   ./policy.sh on            지금 모델로 다시 켬 (enable true)
#   ./policy.sh status        지금 enable · model_path
#   ./policy.sh -h            이 도움말
#
# 한 번에 A/B (같은 배터리·같은 바닥):
#   PC  ./rl-only.sh label:=ab            # 바이패스로 시작 (A)
#       ./route.sh rl
#       ... 같은 경로 주행 (예: isaac/run-calibration.sh sensor-sync-real --output <다른 파일>) ...
#       ./policy.sh ../isaac/artifacts/real/policy.onnx     # B로 전환
#       ... 같은 경로 주행 ...
#       ./policy.sh off                                     # 다시 A (원하면 여러 번)
#   Ctrl+C → ./report.py   (기록의 rl_policy 칸으로 A/B를 나눠 한 보고서에 겹쳐 그림)
#
# - 없는 경로를 주면 corrector가 로드에 실패하고 자연스럽게 바이패스가 된다 (기록의 rl_policy = bypass).
# - 바뀌는 즉시 적용 — 다음 입력 명령부터. rl-only.sh가 떠 있어야 한다 (노드 /rl_corrector). ROS_DOMAIN_ID=25.
# - 상대경로는 지금 위치 기준 → 절대경로로 넘긴다 (corrector는 다른 폴더에서 돌므로).
#
# 종료 코드: 0 성공 · 1 실패(노드 없음 등) · 2 잘못된 인자
set -eo pipefail   # set -u 금지 — ROS setup.bash가 미설정 변수를 참조한다
WS="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$WS/.." && pwd)"
NODE=/rl_corrector

usage() { sed -n '2,/^set -eo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }
case "${1:-}" in
  -h|--help|help) usage; exit 0 ;;
  "") usage; exit 2 ;;
esac
[[ $# -eq 1 ]] || { echo "인자는 하나: <정책.onnx> | off | on | status (./policy.sh -h)" >&2; exit 2; }

source "/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=25
source "$REPO/tool/peers.sh"

param() { ros2 param set "$NODE" "$1" "$2" || { echo "실패 — rl-only.sh가 떠 있나? (노드 $NODE)" >&2; exit 1; }; }
case "$1" in
  off|on) d=false; [[ "$1" == on ]] && d=true   # = corrector ~/toggle 서비스 (nav2 collision_monitor toggle 방식)
          ros2 service call "$NODE/toggle" std_srvs/srv/SetBool "{data: $d}" | tail -1 || { echo "실패 — rl-only.sh가 떠 있나? (노드 $NODE)" >&2; exit 1; } ;;
  status) ros2 param get "$NODE" enable && ros2 param get "$NODE" model_path || exit 1 ;;
  *)
    model="$1"; [[ "$model" == /* ]] || model="$PWD/$model"
    # 없는 경로를 켜면 corrector가 조용히 바이패스 → A/B가 A/A가 됨. 바이패스는 'off'로만.
    [[ -f "$model" ]] || { echo "[policy] 파일 없음 — 바꾸지 않음: $model (바이패스는 ./policy.sh off)" >&2; exit 1; }
    param model_path "$model"
    param enable true
    echo "[policy] $model"
    ;;
esac
