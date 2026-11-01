#!/bin/bash
# training 실행기 — task별 래퍼 없이 여기서 다 돌린다.
#
#   ./run-training.sh <command> [options]
#
# commands:
#   train [task_dir] [...]   학습 실행 (task 디렉토리 지정, 기본 rl_correction_wheel_drift)
#     ./run-training.sh train rl_correction_wheel_drift 300 --task goal --friction
#     ./run-training.sh train rl_correction_wheel_drift 300 --watch --envs 16
#   train-track [iters] [...]  track 본학습 (기본 300)
#   train-goal [iters] [...]   goal 본학습 (기본 300)
#   train-goal-friction [iters] [...]  goal + 바닥 마찰 randomization
#   export <ckpt.pt> <이름.onnx> [track|goal]  체크포인트 → ONNX
#   play <model.onnx> [envs]     학습된 정책 구경 (뷰포트, 기본 4)
#   real-launch <model.onnx> [goal|track]  PC RL 노드 실행 (Pi는 route rl)
#   real-square [...]            같은 사각형+삼각형을 RL goal로 주행 + CSV
#   plot-goal-square [--show]    RL 전/후 궤적·보정 비교 PNG
#   real-rl <model.onnx> [goal|track] [task_dir]  corrector 실행 + 보고서
#   sim-eval <model.onnx> [track|goal] [...]  sim Before/After (real 가기 전 수치 확인)
#     ./run-training.sh sim-eval export/rl_correct/foo.onnx track --episodes 5
#
# common:
#   -h, --help, help  이 도움말
#   ISAACLAB_DIR 환경변수로 IsaacLab 경로 변경 (기본 /home/pank/pa-course/IsaacLab-6.1)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ISAAC="$HERE"
TRAINING="$HERE/training"
TASK_DEFAULT="$TRAINING/rl_correction_wheel_drift"
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"

usage() {
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
}

if [[ $# -eq 0 ]] || [[ "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
  usage
  exit 0
fi

CMD="$1"; shift
case "$CMD" in
  train)
    TASK_DIR=""
    if [[ $# -gt 0 && "$1" != -* && ! "$1" =~ ^[0-9]+$ ]]; then
      TASK_DIR=$1; shift
    fi
    TASK_DIR=${TASK_DIR:-rl_correction_wheel_drift}
    [[ "$TASK_DIR" = /* ]] || {
      if [[ -d "$TRAINING/$TASK_DIR" ]]; then TASK_DIR="$TRAINING/$TASK_DIR"
      elif [[ -d "$PWD/$TASK_DIR" ]]; then TASK_DIR="$PWD/$TASK_DIR"
      else echo "task 디렉토리 없음: $TASK_DIR" >&2; exit 2; fi
    }
    [[ -f "$TASK_DIR/train.py" ]] || { echo "train.py 없음: $TASK_DIR" >&2; exit 2; }
    TRAIN_ARGS=()
    if [[ $# -gt 0 && "$1" =~ ^[0-9]+$ ]]; then
      TRAIN_ARGS+=(--max-iterations "$1"); shift
    fi
    NAME=$(basename "$TASK_DIR")
    STAMP=$(date +%Y%m%d_%H%M%S)
    RUN="$TRAINING/artifacts/train_${NAME}_${STAMP}"
    mkdir -p "$RUN"
    {
      echo "date: $(date -Is)"
      echo "task_dir: $TASK_DIR"
      echo "command: train.py ${TRAIN_ARGS[*]} $*"
    } > "$RUN/command.txt"
    echo "[train] task: $TASK_DIR"
    echo "[train] report: $RUN"
    cd "$ISAACLAB"
    set -o pipefail
    ./isaaclab.sh -p "$TASK_DIR/train.py" "${TRAIN_ARGS[@]}" "$@" 2>&1 | tee "$RUN/train.log"
    ONNX=$(grep -oE '/[^ ]+\.onnx' "$RUN/train.log" | tail -1 || true)
    if [[ -n "${ONNX:-}" && -f "$ONNX" ]]; then
      cp "$ONNX" "$RUN/"
      echo "[train] onnx → $RUN/$(basename "$ONNX")"
    fi
    ;;
  train-track)
    ITERS=300
    if [[ "${1:-}" =~ ^[0-9]+$ ]]; then ITERS=$1; shift; fi
    cd "$ISAACLAB"
    ./isaaclab.sh -p "$TASK_DEFAULT/train.py" --max-iterations "$ITERS" "$@"
    ;;
  train-goal)
    ITERS=300
    if [[ "${1:-}" =~ ^[0-9]+$ ]]; then ITERS=$1; shift; fi
    cd "$ISAACLAB"
    ./isaaclab.sh -p "$TASK_DEFAULT/train.py" --task goal --max-iterations "$ITERS" "$@"
    ;;
  train-goal-friction)
    ITERS=300
    if [[ "${1:-}" =~ ^[0-9]+$ ]]; then ITERS=$1; shift; fi
    cd "$ISAACLAB"
    ./isaaclab.sh -p "$TASK_DEFAULT/train.py" \
      --task goal --friction --exp-name rl_goal_friction --max-iterations "$ITERS" "$@"
    ;;
  export)
    CKPT=${1:?checkpoint .pt 경로}
    OUT=${2:?출력 파일명}
    TASK=${3:-track}
    [[ "$CKPT" != /* ]] && CKPT="$ISAAC/$CKPT"
    cd "$ISAACLAB"
    ./isaaclab.sh -p "$TASK_DEFAULT/export.py" \
      --checkpoint "$CKPT" --out "$OUT" --task "$TASK"
    ;;
  play)
    MODEL=${1:?onnx 경로}
    ENVS=${2:-4}
    [[ "$MODEL" != /* ]] && MODEL="$ISAAC/$MODEL"
    cd "$ISAACLAB"
    ./isaaclab.sh -p "$TASK_DEFAULT/play.py" --model "$MODEL" --envs "$ENVS" --visualizer kit
    ;;
  real-launch)
    REPO=$(cd "$ISAAC/.." && pwd)
    MODEL=${1:?ONNX 파일 경로}
    MODE=${2:-goal}
    [[ "$MODEL" != /* ]] && MODEL="$ISAAC/$MODEL"
    [ -f "$MODEL" ] || { echo "ONNX 없음: $MODEL" >&2; exit 2; }
    [ -f "$REPO/ros2_ws/install/setup.bash" ] || {
      echo "먼저 빌드: cd $REPO/ros2_ws && colcon build --packages-select rl_correction tb3_bringup" >&2
      exit 2
    }
    source /opt/ros/lyrical/setup.bash
    source "$REPO/ros2_ws/install/setup.bash"
    export ROS_DOMAIN_ID=25
    source "$REPO/tool/peers.sh"
    exec ros2 launch tb3_bringup rl-only.launch.py \
      enable:=true mode:="$MODE" goal_mode:=absolute model:="$MODEL"
    ;;
  real-square)
    REPO=$(cd "$ISAAC/.." && pwd)
    source /opt/ros/lyrical/setup.bash
    source "$REPO/ros2_ws/install/setup.bash"
    export ROS_DOMAIN_ID=25
    source "$REPO/tool/peers.sh"
    STAMP=$(date +%Y%m%d_%H%M%S)
    RUN="$TRAINING/artifacts/real_square_${STAMP}"
    mkdir -p "$RUN"
    set -o pipefail
    PY="$ISAAC/.venv/bin/python"
    [ -x "$PY" ] || PY=python3
    "$PY" "$TASK_DEFAULT/real_square.py" "$@" 2>&1 | tee "$RUN/run.log"
    cp "$TRAINING/artifacts/real_goal_square.csv" "$RUN/" 2>/dev/null || true
    ;;
  plot-goal-square)
    PY="$ISAAC/.venv/bin/python"
    [ -x "$PY" ] || PY=python3
    exec "$PY" "$TASK_DEFAULT/plot_goal_square.py" "$@"
    ;;
  real-rl)
    MODEL=${1:?ONNX 파일 경로}
    MODE=${2:-goal}
    TASK=${3:-rl_correction_wheel_drift}
    [[ "$TASK" = /* ]] || TASK="$TRAINING/$TASK"
    [[ -d "$TASK" ]] || { echo "task 없음: $TASK" >&2; exit 2; }
    [[ "$MODEL" = /* ]] || MODEL="$ISAAC/$MODEL"
    [[ -f "$MODEL" ]] || { echo "ONNX 없음: $MODEL" >&2; exit 2; }
    STAMP=$(date +%Y%m%d_%H%M%S)
    RUN="$TRAINING/artifacts/real_rl_${STAMP}"
    mkdir -p "$RUN"
    {
      echo "date: $(date -Is)"
      echo "model: $MODEL"
      echo "mode: $MODE"
      echo "task: $TASK"
    } > "$RUN/command.txt"
    echo "[real_rl] model: $MODEL"
    echo "[real_rl] report: $RUN"
    echo "[real_rl] Pi에서 먼저: ssh pak@pak.local 'cd ~/ros2/tool && ./run-cli route rl'"
    set -o pipefail
    "$0" real-launch "$MODEL" "$MODE" 2>&1 | tee "$RUN/launch.log"
    ;;
  sim-eval)
    MODEL=${1:?사용법: sim-eval <model.onnx> [track|goal] [calibration.py 옵션...]}
    TASK=track
    if [[ "${2:-}" == "track" || "${2:-}" == "goal" ]]; then TASK=$2; shift 2; else shift; fi
    [[ "$MODEL" != /* ]] && MODEL="$ISAAC/$MODEL"
    [[ -f "$MODEL" ]] || { echo "ONNX 없음: $MODEL" >&2; exit 2; }
    cd "$ISAACLAB"
    ./isaaclab.sh -p "$TASK_DEFAULT/calibration.py" --model "$MODEL" --task "$TASK" "$@"
    ;;
  *)
    echo "모르는 명령: $CMD (./run-training.sh --help)" >&2
    exit 2
    ;;
esac
