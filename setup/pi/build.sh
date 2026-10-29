#!/bin/bash
# Pi: ros2_ws 빌드 한 번에 — 처음 세팅이든 git pull 뒤든 같은 명령.
#
#   ~/ros2/setup/pi/build.sh                                  # 전부
#   ~/ros2/setup/pi/build.sh --packages-select tb3_bringup    # 뒤 인자는 colcon build에 그대로
#
# 1) 패치된 외부 소스를 원본으로 되돌림 (그래야 2·3이 몇 번 돌려도 같은 결과 — 두 번 패치해 .rej 꼬이는 일 없음)
# 2) vcs import: ros2_ws/deps.repos의 외부 소스를 고정 커밋으로 받거나 맞춤 (src/는 gitignore)
# 3) ros2_ws/patches/<저장소>-*.patch 를 src/<저장소>에 적용
# 4) colcon build (Pi 4 메모리: 2개씩 · CMake 4.x용 정책 옵션)
# 먼저: apt 패키지 (documents/ros/lyrical/lyrical-setup-ubuntu26.04.md 2장)
set -eo pipefail

WS="$(cd "$(dirname "$0")/../../ros2_ws" && pwd)"
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
cd "$WS"

patched_repo() { local name; name=$(basename "$1" .patch); echo "src/${name%%-*}"; }

echo "=== 1) 패치 되돌리기"
for p in patches/*.patch; do
  repo=$(patched_repo "$p")
  [ -d "$repo/.git" ] && git -C "$repo" checkout -q -- . && git -C "$repo" clean -qfd
done

echo "=== 2) 외부 소스 (deps.repos)"
vcs import src < deps.repos

echo "=== 3) 패치"
for p in patches/*.patch; do
  repo=$(patched_repo "$p")
  git -C "$repo" apply "$WS/$p"
  echo "  $p -> $repo"
done

echo "=== 4) colcon build"
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "$@"
echo "OK — source $WS/install/setup.bash"
