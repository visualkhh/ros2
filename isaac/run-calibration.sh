#!/bin/bash
# calibration 실행기 — python -m calibration (calibration/__main__.py) 을 --robot에 맞는 런타임(IsaacLab / ROS .venv / .venv)으로 돌린다.
#
#   ./run-calibration.sh <task> --robot sim|real|dummy|plan|both [옵션...]   (경로는 isaac/ 기준 상대경로 또는 절대경로)
#   ./run-calibration.sh <task> --robot sim --help                       그 로봇의 옵션 전체 (python argparse)
#
# 학습 전에 먼저: training/<task>/calibration/ 명령(CalibrationCommand = 네모: 사각형 → 180° 턴 → 역주행, 54 s) · env로
#   sim과 real에 같은 명령을 넣고 응답을 재서 fit.json(바퀴 gain 좌/우 · 지연 · τ · 데드밴드)으로 만든다.
#   차이는 training/<task>/robot_cfg.py RESPONSE_*에 숫자로 → 학습 env가 그 값 중심으로 랜덤. 자세히: README.md 3장.
#   로봇 = 그 task 폴더: sim → sim_robot.py(+ calibration/env.py) · real → real_robot.py · dummy → dummy_robot.py
#
# 결과 폴더: artifacts/<task>/calibration/<시각>_<label>/ → raw.csv + (측정 뒤 자동) report.py → fit.json · report.png
# 터미널 출력: artifacts/<task>/logs/<시각>_<label>_calibration.log (화면에도 그대로)
#   raw.csv  = 20 Hz 기록 (명령 · 보정 출력 · odom · 엔코더 · 자이로 · 바퀴 rad/s · 메시지 시각)
#   fit.json = 응답 fit (gain 좌/우 · 지연 · τ · 데드밴드 · 구간별 speed_fit)
#
# ─── --robot ───────────────────────────────────────────────────────────────────
#
#   공통   --label NAME (폴더 <시각>-NAME · CSV label) · --out PATH (정확한 raw.csv, label보다 우선)
#          --side 0.5 --speed 0.15 --turn 1.0 (사각형 한 변 [m] · 기준 전진 [m/s] · 회전 [rad/s]) · --cmd square|MOD:CLASS (명령 바꾸기)
#
#   sim    순수 sim (IsaacLab — 보정 없음) · 창은 기본 끔 · --viz (= kit) 또는 --viz X로 켬 · --watch (기록 후 창 닫을 때까지 반복)
#          창: 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 노란 점 = 목표 경로 — 벗어나는 만큼이 드리프트
#   real   실기기 (PC · .venv · ROS 환경/peers 자동, ROS_DOMAIN_ID=25) · --check-only (안 움직이고 상태만) · --rate 20
#          준비: 로봇 베이스 켜기 · MANUAL · 주변 1 m 비우기 (순서: training/<task>/README.md calibration 절)
#   dummy  가짜 로봇 (Isaac/ROS 없이 흐름 확인) · --gain/--gain-l/--gain-r --tau --delay --deadband --slip --noise --seed
#          → fit.json에 넣은 값이 나오면 파이프라인 OK
#   plan   로봇 없이 명령만 계획 JSON {dt, label, steps[[v, w]...]}으로 (--out PATH · --rate 20) — 로봇 쪽 재생기(Lv2_bee_Assignment run-cli calibrate wheel)가 읽는다
#   both   sim + real을 같은 경로로 한 번에 → 비교 그림 · --label real_floor --side --speed --turn --show
#          → artifacts/<task>/calibration/<시각>_calibrate/{sim,real}/ + report.py (fit.json · report.png · compare.png · RESPONSE 값)
#
#   ./run-calibration.sh rl_correction_waypoint --robot sim            # 창 없음
#   ./run-calibration.sh rl_correction_waypoint --robot sim --viz      # 창으로 보면서
#   ./run-calibration.sh rl_correction_waypoint --robot real --check-only
#   ./run-calibration.sh rl_correction_waypoint --robot real --label real_floor
#   ./run-calibration.sh rl_correction_waypoint --robot dummy --gain-l 0.96 --gain-r 1.02
#   ./run-calibration.sh rl_correction_waypoint --robot both
#
# 분석 (측정 뒤 직접): python3 training/<task>/calibration/report.py <sim 폴더> <real 폴더>
#   → 폴더마다 fit.json · report.png, real 폴더에 compare.png, robot_cfg.py RESPONSE_* 값 출력
#
# common:
#   -h, --help, help   이 도움말 · ISAACLAB_DIR 환경변수로 IsaacLab 경로 변경 (기본 /home/pank/pa-course/IsaacLab-6.1)
#   어느 폴더에서 실행해도 isaac/로 이동해서 돈다 (경로 인자는 isaac/ 기준)
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$ISAAC/.." && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"
PY="$ISAAC/.venv/bin/python"
[ -x "$PY" ] || PY=python3

usage() {
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
}

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

sim() {
  viz_args "$@"
  VIRTUAL_ENV="$ISAACLAB/.venv" "$ISAACLAB/isaaclab.sh" -p "${VIZ_ARGS[@]}"
}

ros_env() {
  set +u   # ROS setup.bash는 unset 변수를 참조한다
  source /opt/ros/lyrical/setup.bash
  export ROS_DOMAIN_ID=25
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
  source "$REPO/tool/peers.sh"
  set -u
}

# real_robot.py가 bee에 SSH/scp로 붙어 로봇 안에서 재생하는 task(rl_maze_beebot 등)는
# PC의 ROS/peers가 필요 없다. ros_env()는 pak.local(DDS 피어) · ROS_DOMAIN_ID=25를 쓰는
# Pi 직접 연결용이라 beebot에서는 엉뚱한 pak를 찾고 25로 표시된다 → 건너뛴다.
real_uses_ssh() {
  grep -q "BEE_HOST" "$ISAAC/training/$TASK/real_robot.py" 2>/dev/null
}

if [[ $# -eq 0 ]] || [[ "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
  usage
  exit 0
fi

cd "$ISAAC"   # python -m calibration + 상대경로 기준
TASK="$1"; shift
ROBOT=""; REST=()   # --robot=X 또는 --robot X
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot=*) ROBOT="${1#--robot=}"; shift ;;
    --robot) ROBOT="${2:?--robot sim|real|dummy|both}"; shift 2 ;;
    *) REST+=("$1"); shift ;;
  esac
done
set -- ${REST[@]+"${REST[@]}"}
LABEL_ARG=$(printf '%s\n' "$@" | sed -n '/^--label=/{s/^--label=//p;q}; /^--label$/{n;p;q}')
OUT_ARG=$(printf '%s\n' "$@" | sed -n '/^--\(out\|output\)=/{s/^--[a-z]*=//p;q}; /^--\(out\|output\)$/{n;p;q}')
STAMP=$(date +%Y-%m-%d_%H-%M-%S)
declare -A TAG=([sim]=02-sim [real]=02-real [dummy]=dummy)
if [[ -n "${TAG[$ROBOT]:-}" && -z "$OUT_ARG" ]]; then   # 결과 폴더를 여기서 정해 넘긴다 — 측정 뒤 report를 그 폴더로
  OUT_ARG="artifacts/$TASK/calibration/${STAMP}_${LABEL_ARG:-${TAG[$ROBOT]}}/raw.csv"
  set -- "$@" --out "$OUT_ARG"
fi
if [[ -n "$ROBOT" && -d "$ISAAC/training/$TASK" && " $* " != *" --help "* && " $* " != *" -h "* ]]; then
  mkdir -p "$ISAAC/artifacts/$TASK/logs"
  LOG="$ISAAC/artifacts/$TASK/logs/${STAMP}_${LABEL_ARG:-$ROBOT}_calibration.log"
  exec > >(tee -a "$LOG") 2>&1   # 화면 + 파일
  echo "[log] $LOG"
fi
report() {   # 측정이 끝나면 그 폴더로 task의 report.py (fit.json · report.png) — --robot both 안쪽 실행은 끝에 한 번에
  local rc=$1
  if [[ $rc -eq 0 && -f "$OUT_ARG" && -z "${CALIB_NO_REPORT:-}" ]]; then
    "$PY" "$ISAAC/training/$TASK/calibration/report.py" "$(dirname "$OUT_ARG")" 2>&1 | grep -v "UserWarning\|warnings.warn" || true
  fi
  exit "$rc"
}
case "$ROBOT" in
  sim)
    set +e; sim -m calibration "$TASK" --robot sim "$@"; rc=$?; set -e
    report $rc
    ;;
  real)
    # beebot처럼 real_robot.py가 SSH로 로봇 안에서 재생하는 task는 PC ROS/peers를 안 쓴다.
    if ! real_uses_ssh; then
      ros_env
    fi
    set +e; "$PY" -m calibration "$TASK" --robot real "$@"; rc=$?; set -e
    report $rc
    ;;
  dummy)
    set +e; "$PY" -m calibration "$TASK" --robot dummy "$@"; rc=$?; set -e
    report $rc
    ;;
  plan)
    exec "$PY" -m calibration "$TASK" --robot plan "$@"
    ;;
  both)
    LABEL=real_floor SIDE=0.5 SPEED=0.15 TURN=1.0
    SIM_EXTRA=() SHOW=() COMMON_EXTRA=()
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --label) LABEL=${2:?--label 이름}; shift 2 ;;
        --side) SIDE=${2:?--side 값}; shift 2 ;;
        --speed) SPEED=${2:?--speed 값}; shift 2 ;;
        --turn) TURN=${2:?--turn 값}; shift 2 ;;
        --show) SHOW=(--show); shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "모르는 옵션: $1" >&2; exit 2 ;;
      esac
    done
    RUN="artifacts/$TASK/calibration/$(date +%Y-%m-%d_%H-%M-%S)_calibrate"
    mkdir -p "$RUN"
    COMMON=(--side "$SIDE" --speed "$SPEED" --turn "$TURN" ${COMMON_EXTRA[@]+"${COMMON_EXTRA[@]}"})
    echo "[calibrate] sim + real 시작 → $RUN"
    CALIB_NO_REPORT=1 "$0" "$TASK" --robot sim "${COMMON[@]}" ${SIM_EXTRA[@]+"${SIM_EXTRA[@]}"} \
      --out "$RUN/sim/raw.csv" > "$RUN/sim.log" 2>&1 &
    SIM_PID=$!
    set +e
    CALIB_NO_REPORT=1 "$0" "$TASK" --robot real "${COMMON[@]}" --label "$LABEL" \
      --out "$RUN/real/raw.csv" > "$RUN/real.log" 2>&1
    REAL_RC=$?
    wait "$SIM_PID"; SIM_RC=$?
    set -e
    if [[ $REAL_RC -ne 0 || $SIM_RC -ne 0 ]]; then
      echo "[calibrate] 실패 sim=$SIM_RC real=$REAL_RC" >&2
      tail -30 "$RUN/sim.log" >&2 || true
      tail -30 "$RUN/real.log" >&2 || true
      exit 1
    fi
    "$PY" "$ISAAC/training/$TASK/calibration/report.py" "$RUN/sim" "$RUN/real" | tee "$RUN/compare.log"
    echo "[calibrate] → $RUN  (sim/fit.json · real/fit.json · compare.png)"
    ;;
  *)
    echo "--robot sim|real|dummy|plan|both 필요 (지금: '${ROBOT}') — ./run-calibration.sh -h" >&2
    exit 2
    ;;
esac
