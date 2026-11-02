#!/bin/bash
# training 실행기 — python -m training 을 IsaacLab으로 돌린다.
#
#   ./run-training.sh [task] [옵션...]      (task 기본: rl_correction_wheel_drift, 경로는 isaac/ 기준)
#   ./run-training.sh rl_correction_wheel_drift --help      task 옵션 전체 (python argparse)
#
# 무엇을 하나: Isaac Lab에 로봇 512대를 띄워 PPO로 바퀴 드리프트 보정 정책을 학습 → policy.onnx.
#   명령은 0.5~2초마다 랜덤(키보드식 주행 · 부드러운 곡선 · 정지), 정책은 [Δv, Δw]를 더해 실제 몸체 속도를 명령에 맞춘다.
#   모터 지연·좌우 gain·데드밴드·통신 지연·바닥 마찰/기울기를 에피소드마다 랜덤 → --fit으로 그 중심을 real 값에 맞춘다.
#
# 결과 폴더: artifacts/training/<시각>-<exp-name>/
#   policy.onnx (입력 obs[1,6] → 출력 [1,2] 물리 보정량) · eval.json (학습 끝 sim 정책 끔/켬 비교)
#   · base-fit.json (넣은 fit · 지연 · 랜덤 범위) · model_<iter>.pt · TensorBoard · screen_shot_*.png · snaps/ · latest.png
#
# ─── 옵션 ──────────────────────────────────────────────────────────────────────
#   --iterations N     학습 반복 (기본 300). 1회 = envs × 64스텝 수집 + PPO 업데이트 (512대 ≈ 5.5 s/회, 300회 ≈ 30분)
#   --envs N           동시에 학습하는 로봇 수 (기본 512 · 창 볼 땐 16~32)
#   --fit R,S          real/fit.json,sim/fit.json → 차이를 바퀴 τ·gain·지연(·데드밴드) 랜덤 범위의 중심으로 (폭 유지).
#                      S는 --fit 없이 잰 raw sim, 학습과 같은 물리(robot_spec.PHYSICS_HZ · ActuatorParam)로 잰 것 — 자동 확인은 없다.
#                      1개면 real 값 그대로 (학습 env를 이상적 모터로 보는 것 — 보통은 2개)
#   --latency CSV      rl-only Before 기록(경로 rl, enable:=false) → RL 경로 왕복 지연을 명령/관측 지연 범위에. PC·Pi 시계가 안 맞으면 거부
#   --friction / --no-friction   env별 바닥 마찰 랜덤 + 파랑(미끄럼)/주황(끈적) 패치 (기본 켜짐) · --patch-count 4
#   --slope DEG        env마다 바닥을 랜덤 방향 · 0~DEG° 기울임 (기본 3, 0 = 평평)
#   --uneven           낮은 턱(1~3 cm) 켜기 (기본 끔) · --uneven-count 4
#   --gyro-bias RAD_S  에피소드마다 자이로 영점 오차 ±RAD_S (기본 0.05, 0 = 끔)
#   --exp-name NAME    결과 폴더 이름 (기본 rl_correct)
#   --watch            Isaac 창으로 보기 (기본은 headless) + TRAINING FIT 창(보정값·지연·랜덤 범위·액추에이터·마찰, 읽기만)
#                      창을 클릭하고 P 키 → <결과>/screen_shot_<시각>.png
#   --snap SEC         창 없이도 SEC초마다 <결과>/snaps/<시각>.png + <결과>/latest.png (eog로 열어 두면 갱신, ~20% 느려짐)
#   화면(--watch · --snap)에서 보이는 것:
#     초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 (길이 ∝ 속도)
#     노란 점   = 목표 경로 — 랜덤 명령이 바뀐 순간의 자세에서 그 명령(v, w)대로 유지 시간 동안 갈 길
#     로봇이 노란 점에서 벗어나는 만큼이 드리프트 (정책이 학습될수록 줄어야 함)
#
# ─── 예 ────────────────────────────────────────────────────────────────────────
#   FIT=artifacts/calibration/<real>/fit.json,artifacts/calibration/<sim>/fit.json
#   ./run-training.sh --watch --envs 16 --iterations 5 --fit $FIT        # 짧게 확인 (이 정책은 실기기에 올리지 말 것)
#   ./run-training.sh --iterations 300 --fit $FIT --snap 60              # 본 학습
#   ./run-training.sh --iterations 300 --fit $FIT --latency ../ros2_ws/artifacts/real/before.csv   # RL 경로 지연까지
#   tensorboard --logdir artifacts/training                              # 진행 그래프
#   학습 뒤 sim에서 play: ./run-calibration.sh sensor-sync-sim --watch --fit $FIT --policy <결과>/policy.onnx
#
# 새 task: training/<task>/train.py 에서 training.Train.Train 을 상속 (add_args · before_launch · run) — training/README.md
#
# common:
#   -h, --help, help   이 도움말
#   ISAACLAB_DIR 환경변수로 IsaacLab 경로 변경 (기본 /home/pank/pa-course/IsaacLab-6.1)
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
if [[ $# -eq 0 || "$1" == -* ]]; then
  set -- rl_correction_wheel_drift "$@"
fi

cd "$ISAAC"   # python -m training + 상대경로 기준
exec "$ISAACLAB/isaaclab.sh" -p -m training "$@"
