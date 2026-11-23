#!/usr/bin/env bash
# TurtleBot3 3D 시뮬레이터 실행. venv/의존성이 없으면 이 기기에 맞게 준비한 뒤 python -m kros.examples 를 실행한다.
# 사용: ./run.sh   (GPU 선택은 examples/__main__.py가 처리)
set -e
here="$(cd "$(dirname "$0")" && pwd)"
kros="$here/../.."
cd "$kros/.."  # ros2/ 에서 돌려야 kros를 패키지로 import한다
if [ ! -x "$kros/.venv/bin/python" ] || ! "$kros/.venv/bin/python" -c "import panda3d, numpy, onnxruntime" 2>/dev/null; then
  profile="$("$kros/setting-venv.sh" --detect)"
  "$kros/setting-venv.sh" "${profile/core/sim}"  # 시뮬레이터는 최소 sim 프로필이 필요하다
fi
exec "$kros/.venv/bin/python" -m kros.examples "$@"
