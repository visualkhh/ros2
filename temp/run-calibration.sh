#!/bin/bash
# calibration 실행기 — 래퍼 없이 여기서 다 돌린다.
#
#   ./run-calibration.sh <command> [options]
#
# commands (sim은 IsaacLab venv, real/plot은 isaac/.venv):
#   random-world     랜덤 방 + 우리 로봇 주행 (W/A/S/D, N 다음 방)
#     ./run-calibration.sh random-world [--seed 7] [--no-calib]
#   sensor-sync-sim  sim 센서 기록 (IMU/odom/raw wheel CSV)
#     ./run-calibration.sh sensor-sync-sim --viz none [--stand]           # 바닥 기록
#     ./run-calibration.sh sensor-sync-sim --viz none --stand             # 공중(지지 박스) 기록
#     ./run-calibration.sh sensor-sync-sim --viz none --stand --watch     # 창 보고 반복
#     ./run-calibration.sh sensor-sync-sim --viz none --no-calib          # raw 기준
#   sensor-sync-real real 센서 기록 (PC, Pi base+MANUAL 필요)
#     ./run-calibration.sh sensor-sync-real --check-only
#     ./run-calibration.sh sensor-sync-real --label real_floor
#     ./run-calibration.sh sensor-sync-real --label real_stand
#   plot-compare     sim/real CSV 파형·궤적 비교 + gain/delay/tau 피팅
#     ./run-calibration.sh plot-compare sim.csv real.csv [--show]
#     ./run-calibration.sh plot-compare sim.csv real.csv --out compare.png --calib-out sim_calib.json
#   calibrate [...]    sim + real 동시 실행 → 비교 → training/artifacts/<run>/ 보고서
#     ./run-calibration.sh calibrate --side 0.5 --show
#     ./run-calibration.sh calibrate --stand --label stand1
#   verify-track [...] track RL 검증 — 같은 (v,w) 재생 + before/after 비교를 한 번에
#     ./run-calibration.sh verify-track --show
#
# common:
#   -h, --help, help  이 도움말 (뒤에 붙이면 각 .py로 그대로 전달)
#   sim 2종은 --viz/--visualizer 없으면 --viz kit 자동 (창 표시)
#   ISAACLAB_DIR 환경변수로 IsaacLab 경로 변경 (기본 /home/pank/pa-course/IsaacLab-6.1)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ISAAC="$HERE"
CALIB="$HERE/calibration"
REPO=$(cd "$HERE/.." && pwd)

usage() {
  sed -n '2,/^set -e$/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
}

if [[ $# -eq 0 ]] || [[ "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
  usage
  exit 0
fi

CMD="$1"; shift
case "$CMD" in
  random-world|random_world_sim|random_world_sim.py)
    PYFILE="random_world_sim.py"
    ARGS=("$@")
    if [[ " ${ARGS[*]} " != *" --viz "* && " ${ARGS[*]} " != *" --visualizer "* ]]; then
      ARGS+=(--viz kit)
    fi
    exec "${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}/isaaclab.sh" -p "$CALIB/$PYFILE" "${ARGS[@]}"
    ;;
  sensor-sync-sim|DefaultSensorSim|DefaultSensorSim.py)
    PYFILE="sensor/DefaultSensorSim.py"
    ARGS=("$@")
    if [[ " ${ARGS[*]} " != *" --viz "* && " ${ARGS[*]} " != *" --visualizer "* ]]; then
      ARGS+=(--viz kit)
    fi
    exec "${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}/isaaclab.sh" -p "$CALIB/$PYFILE" "${ARGS[@]}"
    ;;
  sensor-sync-real|DefaultSensorReal|DefaultSensorReal.py)
    source /opt/ros/lyrical/setup.bash
    export ROS_DOMAIN_ID=25
    export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
    source "$REPO/tool/peers.sh"
    PY="$ISAAC/.venv/bin/python"
    [ -x "$PY" ] || PY=python3
    exec "$PY" "$CALIB/sensor/DefaultSensorReal.py" "$@"
    ;;
  plot-compare|plot_compare|plot_compare.py)
    PY="$ISAAC/.venv/bin/python"
    [ -x "$PY" ] || PY=python3
    exec "$PY" "$CALIB/compare/plot_compare.py" "$@"
    ;;
  calibrate)
    # (구 training/calibrate.sh) sim + real 동시 실행 → 비교 → 보고서
    PROFILE="$CALIB/excitation/CalibExcitation.py"
    LABEL=""
    STAND=0
    NOCAL=0
    SHOW=0
    SIDE=0.5
    SPEED=0.15
    TURN=1.0
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --profile) PROFILE=${2:?--profile 경로}; shift 2 ;;
        --label) LABEL=${2:?--label 이름}; shift 2 ;;
        --stand) STAND=1; shift ;;
        --no-calib) NOCAL=1; shift ;;
        --show) SHOW=1; shift ;;
        --side) SIDE=${2:?--side 값}; shift 2 ;;
        --speed) SPEED=${2:?--speed 값}; shift 2 ;;
        --turn) TURN=${2:?--turn 값}; shift 2 ;;
        -h|--help) echo "사용법: $0 calibrate [--profile PATH] [--label N] [--stand] [--no-calib] [--side M] [--speed M/S] [--turn R/S] [--show]"; exit 0 ;;
        *) echo "모르는 옵션: $1" >&2; exit 2 ;;
      esac
    done
    [[ "$PROFILE" != /* ]] && PROFILE="$PWD/$PROFILE"
    if [[ -d "$PROFILE" ]]; then PROFILE="$PROFILE/CalibExcitation.py"; fi
    [[ -f "$PROFILE" ]] || { echo "profile 없음: $PROFILE" >&2; exit 2; }
    PROFILE=$(realpath "$PROFILE")
    TRAINING="$ISAAC/training"
    if [[ $STAND -eq 1 ]]; then
      TAG="stand"
      LABEL=${LABEL:-stand}
      REAL_ARGS=(--label "real_$LABEL")
      SIM_ARGS=(--stand)
    else
      TAG="floor"
      LABEL=${LABEL:-floor}
      REAL_ARGS=(--label "real_$LABEL")
      SIM_ARGS=()
    fi
    [[ $NOCAL -eq 1 ]] && SIM_ARGS+=(--no-calib)
    STAMP=$(date +%Y%m%d_%H%M%S)
    RUN="$TRAINING/artifacts/calibrate_${STAMP}_${LABEL}"
    mkdir -p "$RUN"
    {
      echo "date: $(date -Is)"
      echo "profile: $PROFILE"
      echo "label: $LABEL"
      echo "stand: $STAND no_calib: $NOCAL side: $SIDE speed: $SPEED turn: $TURN"
    } > "$RUN/command.txt"
    echo "[calibrate] profile: $PROFILE"
    echo "[calibrate] sim + real 동시 시작 → $RUN"
    "$0" sensor-sync-sim --viz none --profile "$PROFILE" \
      --side "$SIDE" --speed "$SPEED" --turn "$TURN" "${SIM_ARGS[@]}" \
      > "$RUN/sim.log" 2>&1 &
    SIM_PID=$!
    set +e
    "$0" sensor-sync-real --profile "$PROFILE" \
      --side "$SIDE" --speed "$SPEED" --turn "$TURN" "${REAL_ARGS[@]}" \
      > "$RUN/real.log" 2>&1
    REAL_RC=$?
    wait "$SIM_PID"; SIM_RC=$?
    set -e
    if [[ $REAL_RC -ne 0 || $SIM_RC -ne 0 ]]; then
      echo "[calibrate] 실패 sim=$SIM_RC real=$REAL_RC" >&2
      tail -30 "$RUN/sim.log" >&2 || true
      tail -30 "$RUN/real.log" >&2 || true
      exit 1
    fi
    SIM_CSV="$TRAINING/artifacts/sim_sensor_${TAG}.csv"
    REAL_CSV="$TRAINING/artifacts/real_${LABEL}.csv"
    [[ -f "$SIM_CSV" && -f "$REAL_CSV" ]] || { echo "CSV 없음" >&2; exit 1; }
    PLOT_ARGS=()
    [[ $SHOW -eq 1 ]] && PLOT_ARGS+=(--show)
    "$0" plot-compare "$SIM_CSV" "$REAL_CSV" "${PLOT_ARGS[@]}" | tee "$RUN/plot.log"
    cp "$SIM_CSV" "$RUN/"
    cp "$REAL_CSV" "$RUN/"
    cp "$TRAINING/artifacts/compare_${TAG}.png" "$RUN/" 2>/dev/null || true
    cp "$TRAINING/artifacts/sim_calib_${TAG}.json" "$RUN/" 2>/dev/null || true
    echo "[calibrate] report → $RUN"
    ;;
  verify-track)
    # (구 training/verify_track.sh) 같은 (v,w) 재생 + before/after 비교
    TRAINING="$ISAAC/training"
    PROFILE="$TRAINING/rl_correction_wheel_drift/CalibExcitation.py"
    LABEL=real_track_rl
    BEFORE="$TRAINING/artifacts/real_floor.csv"
    AFTER=""
    OUT="$TRAINING/artifacts/compare_track_rl.png"
    SHOW=0
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --profile) PROFILE=${2:?--profile 경로}; shift 2 ;;
        --label) LABEL=${2:?--label 이름}; shift 2 ;;
        --before) BEFORE=${2:?--before 경로}; shift 2 ;;
        --after) AFTER=${2:?--after 경로}; shift 2 ;;
        --out) OUT=${2:?--out 경로}; shift 2 ;;
        --show) SHOW=1; shift ;;
        -h|--help) echo "사용법: $0 verify-track [--profile P] [--label N] [--before CSV] [--after CSV] [--out PNG] [--show]"; exit 0 ;;
        *) echo "모르는 옵션: $1" >&2; exit 2 ;;
      esac
    done
    [[ "$PROFILE" != /* ]] && PROFILE="$PWD/$PROFILE"
    [[ -d "$PROFILE" ]] && PROFILE="$PROFILE/CalibExcitation.py"
    [[ -f "$PROFILE" ]] || { echo "profile 없음: $PROFILE" >&2; exit 2; }
    PROFILE=$(realpath "$PROFILE")
    [[ -f "$BEFORE" ]] || { echo "before CSV 없음: $BEFORE (먼저 calibrate)" >&2; exit 2; }
    [[ -n "$AFTER" ]] || AFTER="$TRAINING/artifacts/${LABEL}.csv"
    STAMP=$(date +%Y%m%d_%H%M%S)
    RUN="$TRAINING/artifacts/verify_track_${STAMP}"
    mkdir -p "$RUN"
    {
      echo "date: $(date -Is)"
      echo "profile: $PROFILE"
      echo "label: $LABEL"
      echo "before: $BEFORE"
      echo "after: $AFTER"
      echo "out: $OUT"
    } > "$RUN/command.txt"
    echo "[verify] Pi는 route rl / MANUAL, PC는 real_rl track 이어야 함"
    echo "[verify] replay: $PROFILE"
    "$0" sensor-sync-real --label "$LABEL" --profile "$PROFILE" \
      2>&1 | tee "$RUN/replay.log"
    [[ -f "$AFTER" ]] || { echo "after CSV 없음: $AFTER" >&2; exit 2; }
    PLOT_ARGS=()
    [[ $SHOW -eq 1 ]] && PLOT_ARGS+=(--show)
    "$0" plot-compare "$BEFORE" "$AFTER" \
      --out "$OUT" --calib-out "$RUN/sim_calib_track_rl.json" "${PLOT_ARGS[@]}" \
      2>&1 | tee "$RUN/compare.log"
    cp "$AFTER" "$RUN/"
    cp "$OUT" "$RUN/" 2>/dev/null || true
    echo "[verify] report → $RUN"
    ;;
  *)
    echo "모르는 명령: $CMD (./run-calibration.sh --help)" >&2
    exit 2
    ;;
esac
