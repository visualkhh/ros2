#!/bin/bash
# 학습 실행 — task 디렉토리를 첫 인자로 받는다.
#
#   ./training/train.sh [task_dir] [train.py 옵션...]
#
# 예:
#   ./training/train.sh rl_correction_wheel_drift 300 --task goal --friction
#   ./training/train.sh rl_correction_wheel_drift 300 --watch --envs 16
#   ./training/train.sh rl_correction_wheel_drift --help
#
# task_dir 기본: rl_correction_wheel_drift
# 학습 로그는 isaac/logs/<exp>/ 에, 실행 보고서는 training/artifacts/train_*/ 에 쌓인다.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ISAAC=$(cd "$HERE/.." && pwd)
TASK_DIR=""
if [[ $# -gt 0 && "$1" != -* && ! "$1" =~ ^[0-9]+$ ]]; then
  TASK_DIR=$1; shift
fi
TASK_DIR=${TASK_DIR:-rl_correction_wheel_drift}
[[ "$TASK_DIR" = /* ]] || {
  if [[ -d "$HERE/$TASK_DIR" ]]; then TASK_DIR="$HERE/$TASK_DIR"
  elif [[ -d "$PWD/$TASK_DIR" ]]; then TASK_DIR="$PWD/$TASK_DIR"
  else echo "task 디렉토리 없음: $TASK_DIR" >&2; exit 2; fi
}
[[ -f "$TASK_DIR/train.py" ]] || { echo "train.py 없음: $TASK_DIR" >&2; exit 2; }

# 두 번째 위치 인자가 숫자면 --max-iterations로 변환 (기존 train_goal.sh와 같은 사용법)
TRAIN_ARGS=()
if [[ $# -gt 0 && "$1" =~ ^[0-9]+$ ]]; then
  TRAIN_ARGS+=(--max-iterations "$1"); shift
fi

NAME=$(basename "$TASK_DIR")
STAMP=$(date +%Y%m%d_%H%M%S)
RUN="$HERE/artifacts/train_${NAME}_${STAMP}"
mkdir -p "$RUN"
{
  echo "date: $(date -Is)"
  echo "task_dir: $TASK_DIR"
  echo "command: train.py ${TRAIN_ARGS[*]} $*"
} > "$RUN/command.txt"

echo "[train] task: $TASK_DIR"
echo "[train] report: $RUN"
cd /home/pank/pa-course/IsaacLab-6.1
set -o pipefail
./isaaclab.sh -p "$TASK_DIR/train.py" "${TRAIN_ARGS[@]}" "$@" 2>&1 | tee "$RUN/train.log"

ONNX=$(grep -oE '/[^ ]+\.onnx' "$RUN/train.log" | tail -1 || true)
if [[ -n "${ONNX:-}" && -f "$ONNX" ]]; then
  cp "$ONNX" "$RUN/"
  echo "[train] onnx → $RUN/$(basename "$ONNX")"
fi
