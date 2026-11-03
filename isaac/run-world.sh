#!/bin/bash
# 로봇을 직접 몰아보는 창 — 로봇(USD)과 환경(env · env cfg)을 골라서 띄운다 (envs/drive.py).
#
#   ./run-world.sh [--usd-dir DIR|FILE] [--env MOD:CLASS] [--env-cfg MOD:CLASS] [--speed 0.15] [--turn 1.0]
#
#   --usd-dir   로봇: urdf/<robot> 폴더(urdf.xacro → USD 자동 변환) · USD가 든 폴더 · .usd/.usda 파일
#               (기본: env cfg의 로봇 = assets/tb3_waffle_pi TB3_CFG = urdf/beebot). 바퀴 관절 이름은 wheel_left/right_joint여야 함
#   --env       환경 클래스   (기본 envs.default.default_env:DefaultEnv — 아무것도 없는 평평한 바닥)
#   --env-cfg   환경 설정 클래스 (기본 envs.default.default_env_cfg:DefaultEnvCfg)
#               행동이 [v, w] 명령인 env면 된다 (BaseEnv 계열: DefaultEnv · calibration CalibEnv)
#   --speed --turn   키 하나의 전진 [m/s] · 회전 [rad/s]
#
# 키: W/A/S/D(화살표) 주행 · Shift 느리게 · X 정지 · R 원점으로 · F 카메라를 로봇 쪽으로 · 창 닫으면 끝
# 카메라: 마우스 휠 줌 · 오른쪽 드래그 회전 · 가운데 드래그 이동 (고정 안 함)
# 화면: 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 (cfg.debug_vis) · 터미널: 0.5 s마다 명령 / 엔코더 / 자이로
#
#   ./run-world.sh
#   ./run-world.sh --usd-dir urdf/beebot
#   ./run-world.sh --env calibration.calib_env:CalibEnv --env-cfg calibration.calib_env_cfg:RoomsEnvCfg   # 방 + 장애물
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
cd "$ISAAC"   # 상대경로 기준 = isaac/
exec "$ISAACLAB/isaaclab.sh" -p -m envs.drive "$@"
