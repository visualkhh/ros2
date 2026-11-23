#!/usr/bin/env bash
# 컨테이너 안에서 GCC + CMake로 빌드하고 테스트한다.
#   ./docker/compile.sh                       # RelWithDebInfo (기본)
#   ./docker/compile.sh Debug                 # Debug
#   ./docker/compile.sh RelWithDebInfo --clean
#   ./docker/compile.sh --clean Debug
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"

build_type="RelWithDebInfo"
clean=0
for argument in "$@"; do
  case "$argument" in
    --clean) clean=1 ;;
    Debug|Release|RelWithDebInfo|MinSizeRel) build_type="$argument" ;;
    -h|--help)
      echo "usage: $0 [Debug|Release|RelWithDebInfo|MinSizeRel] [--clean]"
      exit 0
      ;;
    *)
      echo "unknown argument: $argument" >&2
      exit 2
      ;;
  esac
done

command="cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=$build_type -DCMAKE_CXX_COMPILER=g++ && cmake --build build -j\"\$(nproc)\" && ctest --test-dir build --output-on-failure"
if [ "$clean" -eq 1 ]; then
  command="rm -rf build && $command"
fi
docker compose -f "$here/docker-compose.yml" run --rm krosc bash -lc "$command"
