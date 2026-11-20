#!/usr/bin/env bash
# kros용 .venv 생성 + 이 기기에 맞는 의존성 설치.
# 사용: ./setting-venv.sh [core|sim|gpu|auto]   (기본 auto)   ./setting-venv.sh --detect  (자동 감지 결과만 출력)
#   core: kros 코어만 (의존성 없음). 실제 하드웨어(라즈베리파이 등)
#   sim : + 시뮬레이터 (Panda3D, numpy), 개발 도구(mypy)
#   gpu : + GPU 계산 (cupy). NVIDIA GPU + CUDA 13 드라이버가 있을 때
#   auto: 라즈베리파이면 core, 아니면 sim, NVIDIA GPU+CUDA 13이면 gpu. 환경변수 KROS_PROFILE로도 고정 가능
set -e
here="$(cd "$(dirname "$0")" && pwd)"
req="$here/examples/turtlebot3"

cuda_major() {  # nvidia-smi가 보여주는 드라이버의 CUDA 버전(주 버전). GPU 없으면 빈 값
  command -v nvidia-smi >/dev/null 2>&1 || return 0
  nvidia-smi 2>/dev/null | sed -n 's/.*CUDA Version: \([0-9]*\)\..*/\1/p' | head -1
}

detect() {
  if grep -qi "raspberry" /proc/device-tree/model 2>/dev/null; then echo core; return; fi
  if [ "$(cuda_major)" = "13" ]; then echo gpu; return; fi
  echo sim
}

profile="${1:-${KROS_PROFILE:-auto}}"
if [ "$profile" = "--detect" ]; then detect; exit 0; fi
[ "$profile" = "auto" ] && profile="$(detect)"
case "$profile" in core | sim | gpu) ;; *) echo "알 수 없는 프로필: $profile (core|sim|gpu|auto)" >&2; exit 1 ;; esac

cd "$here"
[ -x .venv/bin/python ] || python3 -m venv .venv
echo "프로필: $profile"
# pip는 ros2/ 에서 돌린다 (kros를 패키지로 import하는 위치)
if [ "$profile" != "core" ]; then
  (cd "$here/.." && "$here/.venv/bin/python" -m pip install -q -r "$req/requirements.txt")
fi
if [ "$profile" != "core" ]; then  # 개발용(타입 검사). 라즈베리파이용 core에는 설치하지 않는다
  (cd "$here/.." && "$here/.venv/bin/python" -m pip install -q -r "$here/requirements-dev.txt")
fi
if [ "$profile" = "gpu" ]; then
  (cd "$here/.." && "$here/.venv/bin/python" -m pip install -q -r "$req/requirements-gpu.txt")
  if ! (cd "$here/.." && "$here/.venv/bin/python" -c "import cupy; cupy.zeros(1).sum().item()") 2>/dev/null; then
    echo "경고: cupy가 GPU를 쓰지 못한다. CPU(numpy)로 동작한다." >&2
  fi
fi
echo "done. activate: source $here/.venv/bin/activate"
