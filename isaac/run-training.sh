#!/bin/bash
# training 실행기 — python -m training 을 IsaacLab으로 돌린다.
#
#   ./run-training.sh [task] [옵션...]      (task 기본: rl_correction_wheel_drift, 경로는 isaac/ 기준)
#
#   ./run-training.sh --iterations 300 --fit artifacts/calibration/<real>/fit.json,artifacts/calibration/<sim>/fit.json
#   ./run-training.sh --watch --envs 16 --fit artifacts/calibration/<real>/fit.json,artifacts/calibration/<sim>/fit.json
#   ./run-training.sh --no-friction
#   ./run-training.sh --fit <real>,<sim> --latency ../ros2_ws/artifacts/real/before.csv   # RL 경로 지연까지
#   ./run-training.sh rl_correction_wheel_drift --help      # task 옵션 전체
#
# - 기본 headless. --watch면 Isaac 창 + TRAINING FIT 패널(보정값·랜덤 범위·마찰, 읽기만)
#   창에서 P 키 → 결과 폴더/screen_shot_<시각>.png
# - --snap 60: 창 없이도 60초마다 결과 폴더/snaps/<시각>.png + latest.png (eog로 열어 두면 갱신됨, ~20% 느려짐)
# - --slope 3: env 바닥을 랜덤 방향으로 0~3° 기울임 · --uneven: 낮은 턱 (둘 다 기본 끔)
# - 마찰 패치(파랑=미끄럼/주황=끈적) 기본 켜짐, --no-friction으로 끔
# - 결과: artifacts/training/<시각>-<exp-name>/ (checkpoint · policy.onnx · eval.json(sim 끔/켬) · TensorBoard · base-fit.json · 스크린샷)
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
