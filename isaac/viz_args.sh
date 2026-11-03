# source 해서 쓴다 (run-world.sh · run-calibration.sh · run-training.sh 공통) — 창은 기본 끔 (headless).
#   --viz           → --viz kit (기본 창)
#   --viz X | --viz=X   → 그대로 (kit · newton · rerun · viser, CSV 여러 개 가능)
# 결과: VIZ_ARGS 배열 (원래 인자, 맨 --viz만 kit을 붙임)
VIZ_DEFAULT=kit
viz_args() {
  VIZ_ARGS=()
  while [[ $# -gt 0 ]]; do
    if [[ "$1" == "--viz" || "$1" == "--visualizer" ]] && [[ $# -eq 1 || "$2" == -* ]]; then
      VIZ_ARGS+=("$1" "$VIZ_DEFAULT"); shift
    else
      VIZ_ARGS+=("$1"); shift
    fi
  done
}
