#!/bin/bash
# OpenCR 툴 설치 (최초 1회 또는 환경이 바뀌었을 때)
#
#   ./setup.sh
#
# 설치:
#   - arduino-cli, OpenCR 보드 코어 (DynamixelSDK 포함)
#   - ARM: 시스템 arm-none-eabi-gcc + ROBOTIS opencr_ld_shell
#   - x86: OpenCR 코어의 opencr_gcc를 시스템 gcc 링크로 교체
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

echo "=== apt 패키지 ($ARCH)"
pkgs=(curl ca-certificates bzip2)
if [ "$ARCH" = arm ]; then
  pkgs+=(gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib)
  if [ "$(uname -m)" = aarch64 ]; then
    # opencr_ld_shell_arm은 32비트(armhf) 바이너리 → libc6:armhf 필요.
    # Ubuntu 26.04: arm64는 archive.ubuntu.com, armhf는 ports.ubuntu.com에만 있음 →
    # 기존 저장소는 arm64로 묶고 armhf는 ports에서 받는다 (안 하면 apt update가 armhf 404로 실패).
    $SUDO dpkg --add-architecture armhf
    for f in /etc/apt/sources.list.d/*.sources; do
      grep -q '^Architectures:' "$f" || $SUDO sed -i 's/^Types: .*/&\nArchitectures: arm64/' "$f"
    done
    armhf_src=/etc/apt/sources.list.d/ubuntu-ports-armhf.sources
    [ -f "$armhf_src" ] || $SUDO tee "$armhf_src" >/dev/null <<EOF2
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: $(lsb_release -cs) $(lsb_release -cs)-updates $(lsb_release -cs)-security
Components: main universe
Architectures: armhf
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF2
  fi
  pkgs+=(libc6:armhf)
fi
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends "${pkgs[@]}"

echo "=== arduino-cli"
if ! command -v arduino-cli >/dev/null 2>&1; then
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | $SUDO env BINDIR=/usr/local/bin sh
fi
# 사용자 ~/bin에 이미 설치한 경우도 인식
[ -x "$HOME/bin/arduino-cli" ] && export PATH="$HOME/bin:$PATH"

arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls "$BOARD_URL"
arduino-cli core update-index

echo "=== OpenCR 보드 코어"
data=$(arduino-cli config get directories.data)
if [ "$ARCH" = x86 ]; then
  arduino-cli core install OpenCR:OpenCR
  # ROBOTIS 32비트 툴체인은 64비트 Ubuntu에서 실행 불가 → 시스템 gcc 링크
  rm -rf "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"
  mkdir -p "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"
  for f in /usr/bin/arm-none-eabi-*; do
    ln -sf "$f" "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/"
  done
elif [ ! -d "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION" ]; then
  # ARM: 보드 패키지만 수동 설치 + 시스템 gcc 링크
  # NOTE: 릴리스 파일명이 .tar.bz2지만 내용은 gzip → xzf
  tmp=$(mktemp -d)
  curl -fsSL "https://github.com/ROBOTIS-GIT/OpenCR/releases/download/$OPENCR_VERSION/opencr.tar.bz2" \
    -o "$tmp/opencr.tar.bz2"
  echo "$OPENCR_SHA256  $tmp/opencr.tar.bz2" | sha256sum -c -
  tar xzf "$tmp/opencr.tar.bz2" -C "$tmp"
  mkdir -p "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION" \
    "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin"
  cp -a "$tmp/opencr/." "$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION/"
  ln -sf /usr/bin/arm-none-eabi-* "$data/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/"
  rm -rf "$tmp"
fi

echo "=== OpenCR 코어 패치: min/max 매크로 (gcc 14 newlib C++ 헤더와 충돌)"
# 코어 wiring_constants.h의 min(a,b)/max(a,b) 매크로가 <map> 등 std::min(a,b,comp)을 깨뜨려
# DynamixelSDK 컴파일이 실패한다 (Ubuntu 26.04 gcc-arm-none-eabi 14). C++에서는 템플릿 함수로 바꾼다
# (ArduinoCore-API와 같은 방식). C 파일은 기존 매크로 그대로.
wc_h="$data/packages/OpenCR/hardware/OpenCR/$OPENCR_VERSION/cores/arduino/wiring_constants.h"
if [ -f "$wc_h" ] && ! grep -q 'tool-patch: minmax2' "$wc_h"; then
  python3 - "$wc_h" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p).read()
orig = """#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max
"""
new = """// tool-patch: minmax2 — C++에서는 매크로 대신 템플릿 (std 헤더 <map> 등과 충돌 방지).
// 이 헤더는 바깥 extern "C" 블록 안에서 포함되기도 하므로 extern "C++"로 감싼다.
#ifdef __cplusplus
extern "C++" {
template<class T, class L>
auto min(const T& a, const L& b) -> decltype((b < a) ? b : a) { return (b < a) ? b : a; }
template<class T, class L>
auto max(const T& a, const L& b) -> decltype((b < a) ? b : a) { return (a < b) ? b : a; }
}
#else
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max
#endif
"""
if orig in s:
    s = s.replace(orig, new)
else:  # 이전 판 패치(minmax)를 덮어씀
    s, n = re.subn(r"// tool-patch: minmax .*?#endif // max\n#endif\n", lambda m: new, s, flags=re.S)
    assert n == 1, "wiring_constants.h 형식이 예상과 다름"
open(p, "w").write(s)
PY
  echo "패치 적용: $wc_h"
fi

echo "=== opencr_ld_shell ($ARCH)"
# 이름만 .tar.bz2이고 내용은 gzip → 파일로 받아 tar가 형식을 판별
mkdir -p "$TOOLS_DIR"
curl -fsSL "$OPENCR_UPDATE_URL" -o "$TOOLS_DIR/opencr_update.tar"
tar xf "$TOOLS_DIR/opencr_update.tar" -C "$TOOLS_DIR" --strip-components=1 \
  opencr_update/opencr_ld_shell_arm opencr_update/opencr_ld_shell_x86
rm -f "$TOOLS_DIR/opencr_update.tar"
chmod +x "$TOOLS_DIR"/opencr_ld_shell_*

arduino-cli board details --fqbn "$FQBN" >/dev/null
echo "OK: setup 완료 ($ARCH, $LD_SHELL)"
echo "다음: ./install-udev.sh  (/dev/opencr 고정 이름 + 권한)"
echo "시리얼 권한이 없으면: sudo usermod -aG dialout \$USER  (재로그인 필요)"
