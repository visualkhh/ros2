#!/usr/bin/env bash
# rl-only — PC에서 RL 노드(corrector + eval_recorder)만 띄운다. = ros2 launch tb3_bringup rl-only.launch.py
#
#   ./rl-only.sh [enable:=false] [label:=이름] [model:=<정책.onnx>] [output:=<저장.csv>] [launch 인자...]
#
#   ./rl-only.sh enable:=false label:=before      # 보정 없이 통과 (Before · calibration 측정)
#   ./rl-only.sh label:=after                     # 정책 보정 (After, 기본 켬) — 기본 모델 isaac/artifacts/real/policy.onnx (없으면 통과)
#   ./rl-only.sh model:=<isaac>/artifacts/rl_correction_waypoint/training/<시각>/exported/policy.onnx   (play로 내보낸 것 — .onnx.data 같이)
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
# - 기본 enable 켬(ONNX 적용). 모델이 없으면 그냥 통과. enable:=true를 직접 줬는데 모델이 없으면 시작하지 않는다.
# - 실행 중 켬/끔: ros2 service call /rl_corrector/toggle std_srvs/srv/SetBool "{data: false}" (또는 ./policy.sh off|on)
# - 상대경로는 지금 위치 기준 → 절대경로로 바꿔 넘긴다. 저장 폴더는 없으면 만든다.
# - 로봇(Pi)은 RL 경로여야 한다: ./run-cli route rl (끝나면 route direct). ROS_DOMAIN_ID=25.
#
# 백그라운드 실행 (ONNX 바꾸고 다시 켤 때 · 터미널 하나로):
#   ./rl-only.sh start [인자...]     백그라운드로 켬 → corrector 로드 결과(ONNX loaded/passthrough/실패)를 보여 줌
#   ./rl-only.sh stop               끔 (Ctrl+C와 같음 — CSV가 깨끗이 닫힘)
#   ./rl-only.sh restart [인자...]   끄고 다시 켬. 인자 없으면 지난 start 인자 그대로 (output은 덮어씀 → 남기려면 새 output:=)
#   ./rl-only.sh status             켜져 있나 · 인자 · 모델 로드 결과
#   ./rl-only.sh log                로그 따라 보기 (Ctrl+C는 보기만 끝냄)
#   pid·로그·인자: ros2_ws/.run/rl-only.{pid,log,args}. 서브명령 없이 쓰면 지금처럼 앞에서(foreground) 돈다.
#
#   -h, --help   이 도움말
set -eo pipefail   # set -u 금지 — ROS setup.bash가 미설정 변수를 참조한다
WS="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$WS/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,/^set -eo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi

RUN="$WS/.run"; PIDF="$RUN/rl-only.pid"; LOGF="$RUN/rl-only.log"; ARGSF="$RUN/rl-only.args"
mkdir -p "$RUN"
running() { [[ -f "$PIDF" ]] && kill -0 "$(cat "$PIDF")" 2>/dev/null; }
load_line() { grep -aE "ONNX loaded|passthrough|ONNX load failed" "$LOGF" 2>/dev/null | tail -1 | sed 's/^.*\] //'; }
stop_bg() {
  running || { rm -f "$PIDF"; echo "[rl-only] 꺼져 있음"; return 0; }
  local pid; pid=$(cat "$PIDF")
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid"   # setsid로 띄움 → 프로세스 그룹째 (launch + 노드)
  for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
  kill -0 "$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null
  rm -f "$PIDF"; echo "[rl-only] 끔 (pid $pid)"
}
MODE=fg
case "${1:-}" in
  stop)    stop_bg; exit 0 ;;
  status)  if running; then echo "[rl-only] 켜짐 (pid $(cat "$PIDF")) · 인자: $(tr '\n' ' ' < "$ARGSF")"; echo "  $(load_line)"
             echo "  (모델이 로드돼도 enable이 꺼져 있으면 바이패스 — 지금 켜짐/꺼짐: ./policy.sh status)"
           else echo "[rl-only] 꺼짐"; fi; exit 0 ;;
  log)     exec tail -n 50 -f "$LOGF" ;;
  start)   shift; MODE=bg ;;
  restart) shift; stop_bg
           if [[ $# -eq 0 && -f "$ARGSF" ]]; then mapfile -t saved < "$ARGSF"; set -- "${saved[@]}"; fi
           MODE=bg ;;
esac
if [[ "$MODE" == bg ]]; then
  running && { echo "[rl-only] 이미 켜짐 (pid $(cat "$PIDF")) — ./rl-only.sh restart [인자...]" >&2; exit 1; }
  printf '%s\n' "$@" > "$ARGSF"
fi

MODEL_DEFAULT="$REPO/isaac/artifacts/real/policy.onnx"
OUTPUT_DEFAULT="$WS/artifacts/real/raw.csv"
abs() { [[ "$1" == /* ]] && echo "$1" || echo "$PWD/$1"; }
args=() output="" model="" enable=true explicit=false label=rl
for a in "$@"; do
  case "$a" in
    output:=*) output="$(abs "${a#output:=}")"; continue ;;
    model:=*)  model="$(abs "${a#model:=}")";   continue ;;
    label:=*)  label="${a#label:=}" ;;
    enable:=true) enable=true explicit=true ;;
    enable:=false) enable=false ;;
  esac
  args+=("$a")
done
output="${output:-$OUTPUT_DEFAULT}"
if [[ -z "$model" && -f "$MODEL_DEFAULT" ]]; then
  model="$MODEL_DEFAULT"
fi
if [[ "$explicit" == true ]]; then
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
[[ "$MODE" == fg ]] && exec ros2 launch tb3_bringup rl-only.launch.py "${args[@]}"

setsid nohup ros2 launch tb3_bringup rl-only.launch.py "${args[@]}" > "$LOGF" 2>&1 < /dev/null &
echo $! > "$PIDF"
for _ in $(seq 1 30); do [[ -n "$(load_line)" ]] && break; running || break; sleep 0.5; done
running || { echo "[rl-only] 시작 실패 — 로그: $LOGF" >&2; tail -5 "$LOGF" >&2; rm -f "$PIDF"; exit 1; }
echo "[rl-only] 켜짐 (pid $(cat "$PIDF")) · 로그 ./rl-only.sh log · 끄기 ./rl-only.sh stop"
echo "  $(load_line)"
