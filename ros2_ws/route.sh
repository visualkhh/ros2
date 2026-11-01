#!/usr/bin/env bash
# route — 바퀴 명령 경로 바꾸기 (direct ↔ rl). = tool/run-cli route <direct|rl>
#
#   ./route.sh rl        PC RL 보정을 거치게
#   ./route.sh direct    바로 드라이버로 (RL 안 거침)
#   ./route.sh status    지금 상태 보기 (경로 · 제어 모드 · 비상정지 · 노드)
#   ./route.sh -h        이 도움말
#
# 경로:
#   direct   (Pi) cmd_manager ──────────────────────────────────────→ opencr_driver → 바퀴
#   rl       (Pi) cmd_manager → /cmd_vel_rl_in → [PC rl_corrector] → /cmd_vel_rl_out → cmd_manager → opencr_driver
#
# 순서 (RL 실험):
#   1) Pi   ./base-only.sh                (또는 tool/run-cli base start)
#   2) PC   ./rl-only.sh label:=before   (After: enable:=true label:=after)         # 먼저 띄운다
#   3) 어디서든 ./route.sh rl              ← rl_corrector가 안 보이면 거부한다 (켜면 바퀴가 바로 멈추므로)
#   4) 주행 (tool/run-cli teleop · drive ...) → 끝나면 rl-only Ctrl+C 전에 ./route.sh direct
#
# 동작 · 안전:
#   - /cmd_manual_rl(Bool)을 보내고 로봇이 알리는 /state_rl(latched)로 결과를 확인해 출력한다.
#   - rl 경로에서 PC 응답(/cmd_vel_rl_out)이 0.3초 넘게 끊기면 cmd_manager가 0을 보낸다 → 로봇 정지 (의도된 동작).
#   - 실행 중 전환 가능. bringup을 다시 띄울 필요 없음. 시작부터 rl로: ./base-only.sh rl:=true
#   - ROS 토픽이 보이는 곳 어디서나 (Pi · PC). ROS_DOMAIN_ID=25, PC면 tool/peers.sh로 Pi를 찾는다.
#
# 종료 코드: 0 성공 · 1 거부/실패(rl_corrector 없음, 응답 없음) · 2 잘못된 인자
set -eo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"

case "${1:-}" in
  rl|direct)
    [[ $# -eq 1 ]] || { echo "인자는 하나: rl | direct (./route.sh -h)" >&2; exit 2; }
    exec "$REPO/tool/run-cli" route "$1"
    ;;
  status)
    exec "$REPO/tool/run-cli" status
    ;;
  -h|--help|help)
    sed -n '2,/^set -eo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
    ;;
  "")
    sed -n '2,/^set -eo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
    exit 2
    ;;
  *)
    echo "모르는 경로: $1 — rl | direct | status (./route.sh -h)" >&2
    exit 2
    ;;
esac
