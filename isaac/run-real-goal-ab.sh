#!/bin/bash
# 실기기 goal 모드 A/B — 같은 /rl_goal을 A(바이패스) · B(정책)로 한 번씩 달리고 비교.
#
#   ./run-real-goal-ab.sh <task> --onnx <policy.onnx> --goal X,Y [--label 이름] [--timeout 초]
#     예: ./run-real-goal-ab.sh rl_correction_waypoint \
#           --onnx artifacts/rl_correction_waypoint/training/<시각>/exported/policy.onnx \
#           --goal 1.0,0.0
#     → artifacts/<task>/ab_goal/<시각>_<label>/{A,B}/raw.csv · compare_goal.png
#
# 준비 (한 번):
#   Pi: base start · control manual · route rl   (이 스크립트가 control manual · route rl을 다시 확인한다)
#   PC: ros2_ws 빌드 · 이 스크립트가 rl-only.sh를 A/B로 직접 띄운다 (모델은 --onnx)
#   /rl_goal은 첫 goal 때 로봇의 현재 자세를 원점(방향 0)으로 잡는다 — A/B는 같은 자리·방향에서.
#   goal (x, y)는 시작 자세 기준 [m] · y=0이면 그 방향 직진.
#
# A/B 사이에 로봇을 A의 출발 자리·방향으로 돌려놓고 Enter를 누른다.
# 주의: 이 스크립트는 goal 모드 정책 전용이다 (track/네모 A/B는 run-real-ab.sh).
set -eo pipefail

ISAAC=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$ISAAC/.." && pwd)
WS="$REPO/ros2_ws"
PY="$ISAAC/.venv/bin/python"
[ -x "$PY" ] || PY=python3

usage() { sed -n '2,/^set -eo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }
if [[ $# -eq 0 || "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
  usage
  exit 0
fi

TASK="$1"; shift
[[ -f "$ISAAC/training/$TASK/report_goal.py" ]] || {
  echo "goal A/B 보고서 없음: training/$TASK/report_goal.py" >&2; exit 2; }

ONNX="" GOAL="" LABEL="ab_goal" TIMEOUT=30
while [[ $# -gt 0 ]]; do
  case "$1" in
    --onnx=*) ONNX="${1#--onnx=}"; shift ;;
    --onnx) ONNX="${2:?--onnx 경로}"; shift 2 ;;
    --goal=*) GOAL="${1#--goal=}"; shift ;;
    --goal) GOAL="${2:?--goal x,y}"; shift 2 ;;
    --label=*) LABEL="${1#--label=}"; shift ;;
    --label) LABEL="${2:?--label 이름}"; shift 2 ;;
    --timeout=*) TIMEOUT="${1#--timeout=}"; shift ;;
    --timeout) TIMEOUT="${2:?--timeout 초}"; shift 2 ;;
    *) echo "모르는 옵션: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$ONNX" ]] || { echo "--onnx <policy.onnx> 필요" >&2; exit 2; }
ONNX=$(realpath "$ONNX")
[[ -f "$ONNX" ]] || { echo "ONNX 없음: $ONNX" >&2; exit 2; }
[[ -f "$ONNX.data" ]] || echo "[goal-ab] 경고: $ONNX.data 없음 — ONNX 로드 실패할 수 있음" >&2

[[ -n "$GOAL" ]] || { echo "--goal X,Y 필요 (예: --goal 1.0,0.0)" >&2; exit 2; }
IFS=, read -r GX GY <<< "$GOAL"
re='^[+-]?[0-9]+([.][0-9]+)?$'
[[ "$GX" =~ $re && "$GY" =~ $re ]] || { echo "--goal은 'x,y' 숫자 (예: 1.0,0.0)" >&2; exit 2; }
[[ "$TIMEOUT" =~ ^[0-9]+$ ]] || { echo "--timeout은 초 단위 정수" >&2; exit 2; }

# ROS / peers — rl-only.sh·run-cli는 내부에서 하지만, 여기서 topic pub도 하므로 미리 source.
source "/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=25
source "$REPO/tool/peers.sh"

RUN="$ISAAC/artifacts/$TASK/ab_goal/$(date +%Y-%m-%d_%H-%M-%S)_$LABEL"
mkdir -p "$RUN/A" "$RUN/B"
echo "[goal-ab] $RUN · goal=($GX,$GY) · B 정책 $ONNX"

log_line() { tail -n 1 "$WS/.run/rl-only.log" 2>/dev/null || true; }
stop_rl() { "$WS/rl-only.sh" stop >/dev/null 2>&1 || true; }

check_no_corrector() {
  # rl-only.sh stop은 자기가 띄운 background만 끈다 — foreground로 띄운 rl-only가 있으면 중복 노드가 된다.
  if ros2 node list 2>/dev/null | grep -qx '/rl_corrector'; then
    echo "[goal-ab] /rl_corrector가 이미 떠 있음 — 기존 rl-only(foreground)를 Ctrl+C로 끄고 다시 실행" >&2
    return 1
  fi
}

start_rl() {   # $1 enable (false/true), $2 label, $3 output
  local en="$1" lab="$2" out="$3" msg
  stop_rl
  sleep 0.5
  check_no_corrector || return 1
  if ! msg=$("$WS/rl-only.sh" start "enable:=$en" "label:=$lab" "output:=$out" "model:=$ONNX" 2>&1); then
    echo "$msg" >&2
    return 1
  fi
  echo "$msg"
  if ! grep -q "ONNX loaded" <<<"$msg" || ! grep -q "mode=goal" <<<"$msg"; then
    echo "[goal-ab] goal 모드 로드 실패 — rl-only 로그: $WS/.run/rl-only.log" >&2
    tail -8 "$WS/.run/rl-only.log" >&2 || true
    stop_rl
    return 1
  fi
}

route_rl() {
  "$REPO/tool/run-cli" control manual >/dev/null || {
    echo "[goal-ab] control manual 실패 — Pi base/state 확인" >&2; return 1; }
  "$REPO/tool/run-cli" route rl >/dev/null || {
    echo "[goal-ab] route rl 실패 — PC rl-only/corrector 확인" >&2; return 1; }
}

send_goal() {
  if ! timeout 8 ros2 topic pub --once /rl_goal geometry_msgs/msg/Point \
      "{x: $GX, y: $GY, z: 0.0}" >/dev/null; then
    echo "[goal-ab] /rl_goal publish 실패 — ROS_DOMAIN_ID=25 · corrector goal 모드 확인" >&2
    return 1
  fi
}

wait_goal() {
  local end=$((SECONDS + TIMEOUT))
  while (( SECONDS < end )); do
    if grep -aq "goal 도착" "$WS/.run/rl-only.log"; then
      grep -a "goal 도착" "$WS/.run/rl-only.log" | tail -1
      return 0
    fi
    if grep -aq "rl_goal 무시" "$WS/.run/rl-only.log"; then
      echo "[goal-ab] corrector가 /rl_goal을 무시함 (goal 모드 아님):" >&2
      grep -a "rl_goal 무시" "$WS/.run/rl-only.log" | tail -1 >&2
      return 1
    fi
    sleep 0.2
  done
  echo "[goal-ab] goal 도착 timeout (${TIMEOUT}s) — rl-only 로그:" >&2
  tail -12 "$WS/.run/rl-only.log" >&2 || true
  return 1
}

finish_run() {
  timeout 5 ros2 topic pub --once /rl_goal_stop std_msgs/msg/Empty "{}" >/dev/null 2>&1 || true
  stop_rl
}

abort_cleanup() {
  finish_run || true
}
trap abort_cleanup EXIT

echo "[goal-ab] A — 바이패스 (nominal controller, 보정 0)"
start_rl false wp_A "$RUN/A/raw.csv"
route_rl
send_goal
wait_goal
finish_run

read -r -p "[goal-ab] 로봇을 A와 같은 출발 자리·방향으로 돌려놓고 Enter (취소 Ctrl+C) " _

echo "[goal-ab] B — 정책 보정"
start_rl true wp_B "$RUN/B/raw.csv"
route_rl
send_goal
wait_goal
finish_run

trap - EXIT
"$PY" "$ISAAC/training/$TASK/report_goal.py" \
  "$RUN/A/raw.csv" "$RUN/B/raw.csv" \
  --goal "$GX,$GY" --out "$RUN/compare_goal.png"
echo "[goal-ab] → $RUN"
