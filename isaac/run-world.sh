#!/bin/bash
# task 하나를 창으로 띄운다 (envs/drive.py).
#
#   ./run-world.sh <task> [--calibration] [--command NAME] [--viz [NAME]] [--speed 0.15] [--turn 1.0]
#
#   <task>          training/ 아래 task 폴더 이름 — 로봇도 그 폴더 (robot_cfg.py · usda/robot.usda)
#                   기본: task env.py + command.py — 노란 점(랜덤 웨이포인트)을 기본 제어기가 따라감 (정책 보정 0) · R 새 에피소드
#   --calibration   task의 calibration/ — CalibrationEnv + CalibrationCommand (네모 54 s, 노란 점 = 목표 경로)
#   --command NAME  명령만 바꿈 — 학습 배경 위에서: keyboard (commands/keyboard.py 직접 운전) · square · MOD:CLASS
#                   task env가 direct_drive면(rl_maze) 그 env 자체에서 [v,w]로 몬다 (미로 배경 그대로).
#                   (--calibration과 같이 쓰면 측정용 평면)
#   --speed --turn  기준 전진 [m/s] · 회전 [rad/s] (keyboard: 키 하나 · 네모: 기준 속도)
#   --onnx PATH     기본 모드(웨이포인트)에서 학습한 정책으로 보정 (exported/policy.onnx + 옆에 .onnx.data) · P = 켬/끔
#   --viz           창 켜기 (기본 끔 = headless · --viz만 쓰면 kit · --viz X로 다른 visualizer). keyboard는 창이 있어야 함
#
# 키: W/A/S/D(화살표) 주행(keyboard) · Shift 느리게 · X 정지 · `[`/`]` 속도 -/+ · R 처음부터
#     F 카메라를 로봇 쪽으로 · L 라이다 점군 (rl_maze) · 창 닫으면 끝
# 카메라: 마우스 휠 줌 · 오른쪽 드래그 회전 · 가운데 드래그 이동 (고정 안 함)
# 화면: 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향
#       바퀴 위 두 줄 = 아래 유효 마찰(파랑=미끄럼·회색=보통·빨강=끈적) · 위 바퀴 속도(초록=전진·마젠타=후진)
#       터미널: 0.5 s마다 명령 / 엔코더 / 자이로 / 바퀴 마찰·속도 (직접 운전)
#
#   ./run-world.sh rl_correction_waypoint --viz
#   ./run-world.sh rl_correction_waypoint --calibration --viz
#   ./run-world.sh rl_correction_waypoint --command keyboard --viz
#   ./run-world.sh rl_correction_waypoint --viz --onnx artifacts/rl_correction_waypoint/training/<시각>/exported/policy.onnx
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"

# 창은 기본 끔 (headless) — --viz 뒤에 visualizer 이름(kit · newton_gl · newton_rtx · none · rerun · viser, CSV 가능)이 없으면 --viz kit
viz_args() {
  local names='^(kit|newton_gl|newton_rtx|none|rerun|viser)(,(kit|newton_gl|newton_rtx|none|rerun|viser))*$'
  VIZ_ARGS=()
  while [[ $# -gt 0 ]]; do
    if [[ "$1" == "--viz" || "$1" == "--visualizer" ]] && ! [[ "${2:-}" =~ $names ]]; then VIZ_ARGS+=("$1" kit)
    else VIZ_ARGS+=("$1"); fi
    shift
  done
}

if [[ $# -eq 0 || "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  echo "task: $(cd "$ISAAC/training" && ls -d */ | tr -d / | grep -v __ | tr "\n" " ")"
  exit 0
fi
cd "$ISAAC"   # 상대경로 기준 = isaac/
viz_args "$@"
VIRTUAL_ENV="$ISAACLAB/.venv" exec "$ISAACLAB/isaaclab.sh" -p -m envs.drive "${VIZ_ARGS[@]}"
