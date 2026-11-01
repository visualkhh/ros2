#!/usr/bin/env bash
# rl-only — PC에서 RL 노드(corrector + eval_recorder)만 띄운다. = ros2 launch tb3_bringup rl-only.launch.py
#
#   ./rl-only.sh [enable:=true] [label:=이름] [model:=<정책.onnx>] [output:=<저장.csv>] [launch 인자...]
#
#   ./rl-only.sh label:=before                    # 보정 없이 통과 (Before)
#   ./rl-only.sh enable:=true label:=after        # 정책 보정 (After) — 기본 모델 isaac/artifacts/real/policy.onnx
#   ./rl-only.sh enable:=true model:=<isaac>/artifacts/training/<시각>-rl_correct/policy.onnx
#
# 기본 경로:
#   model:=   isaac/artifacts/real/policy.onnx    쓸 정책을 여기에 복사해 두면 된다 (입력 6차원 → track 자동)
#   output:=  ros2_ws/artifacts/real/raw.csv       고정 파일 — 실행마다 덮어쓴다 (남기려면 output:=로 다른 이름)
#   label은 CSV의 label 칸 (기본 rl)
#
# - output CSV (eval_recorder, 입력 1개 = 1줄, 줄마다 바로 디스크로): 입력(lin/ang_raw) · 그 입력의 보정 출력(lin/ang_out)
#   · 정책 원출력(model_dv/dw) · 드라이버 명령(cmd_base) · odom · 엔코더 · 바퀴 속도/위치/전류 · IMU(자이로·가속도·yaw)
#   · 모터 온도 · 배터리 · 상태(control/rl/estop) · 진단 · 메시지 시각(지연). 칸 설명: src/rl_eval/rl_eval/eval_recorder.py
# - 그림: ./report.py (→ CSV 옆 report.png)
# - 실행 중 정책 교체(한 번에 A/B): ./policy.sh <정책.onnx> | off — 기록의 rl_policy 칸으로 구간이 나뉜다
# - enable:=true인데 모델 파일이 없으면 시작하지 않는다. enable 없이(통과)는 모델 없어도 된다.
# - 상대경로는 지금 위치 기준 → 절대경로로 바꿔 넘긴다. 저장 폴더는 없으면 만든다.
# - 로봇(Pi)은 RL 경로여야 한다: ./run-cli route rl (끝나면 route direct). ROS_DOMAIN_ID=25.
#
#   -h, --help   이 도움말
set -eo pipefail   # set -u 금지 — ROS setup.bash가 미설정 변수를 참조한다
WS="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$WS/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,/^set -eo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi

MODEL_DEFAULT="$REPO/isaac/artifacts/real/policy.onnx"
OUTPUT_DEFAULT="$WS/artifacts/real/raw.csv"
abs() { [[ "$1" == /* ]] && echo "$1" || echo "$PWD/$1"; }
args=() output="" model="" enable=false label=rl
for a in "$@"; do
  case "$a" in
    output:=*) output="$(abs "${a#output:=}")"; continue ;;
    model:=*)  model="$(abs "${a#model:=}")";   continue ;;
    label:=*)  label="${a#label:=}" ;;
    enable:=true) enable=true ;;
  esac
  args+=("$a")
done
output="${output:-$OUTPUT_DEFAULT}"
if [[ -z "$model" && -f "$MODEL_DEFAULT" ]]; then
  model="$MODEL_DEFAULT"
fi
if [[ "$enable" == true ]]; then
  [[ -n "$model" ]] || { echo "enable:=true인데 모델 없음 — $MODEL_DEFAULT 에 두거나 model:=<정책.onnx>" >&2; exit 2; }
fi
[[ -z "$model" || -f "$model" ]] || { echo "ONNX 없음: $model" >&2; exit 2; }
args+=("output:=$output")
[[ -n "$model" ]] && args+=("model:=$model")
[[ -f "$WS/install/setup.bash" ]] || { echo "먼저 빌드: cd $WS && colcon build --packages-select rl_correction rl_eval tb3_bringup" >&2; exit 2; }
mkdir -p "$(dirname "$output")"

source "/opt/ros/${ROS_DISTRO_NAME:-lyrical}/setup.bash"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=25
source "$REPO/tool/peers.sh"   # PC → 로봇 Pi 유니캐스트 탐색

echo "[rl-only] enable=$enable model=${model:-(없음)} → $output"
exec ros2 launch tb3_bringup rl-only.launch.py "${args[@]}"
