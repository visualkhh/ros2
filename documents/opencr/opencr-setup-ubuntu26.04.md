# OpenCR 세팅 - Ubuntu 26.04 기준 (Arduino CLI)

> 원본: `/home/pank/workspaces/pa/source/Lv2_bee_Assignment/docker/Dockerfile` 127-169행
> 24.04 Jetson Orin Nano 기준 내용 중 26.04에서도 그대로 되는 부분만 정리.
> ROS Lyrical 문서: [../ros/lyrical/lyrical-setup-ubuntu26.04.md](../ros/lyrical/lyrical-setup-ubuntu26.04.md)

- 보드: OpenCR 1.0 (TurtleBot3 Burger / Waffle)
- 코어 버전: 1.5.3 (고정, 재현성)
- 툴체인: Ubuntu 시스템 `gcc-arm-none-eabi` 사용 (인덱스 내장 `opencr_gcc` 대체)
- 26.04 amd64/arm64 공통 적용

## 1. 설치

```bash
# Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
  | BINDIR=/usr/local/bin sh

# OpenCR 보드 인덱스
arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls \
  https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
arduino-cli core update-index

# 시스템 ARM 툴체인 (26.04 동일 패키지명)
sudo apt update && sudo apt install -y \
  gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib

# 보드 코어 설치 (아키텍처별 분기 - 원본 Dockerfile 로직)
# x86_64:
arduino-cli core install OpenCR:OpenCR
# 인덱스의 opencr_gcc는 32비트(i386)라 64비트 이미지에서 실행 불가 → 시스템 gcc로 대체
rm -rf /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin
mkdir -p /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin
for f in /usr/bin/arm-none-eabi-*; do ln -s "$f" /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/; done

# arm64 (라즈베리파이 등):
# curl -fsSL "https://github.com/ROBOTIS-GIT/OpenCR/releases/download/1.5.3/opencr.tar.bz2" -o /tmp/opencr.tar.bz2
# echo "418656e5e6d99d45d187ffdb28dece0f450c6707da3f6db56769f3ecafdc413c  /tmp/opencr.tar.bz2" | sha256sum -c -
# mkdir -p /root/.arduino15/packages/OpenCR/hardware/OpenCR/1.5.3 \
#   /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin
# tar xzf /tmp/opencr.tar.bz2 -C /tmp
# cp -a /tmp/opencr/. /root/.arduino15/packages/OpenCR/hardware/OpenCR/1.5.3/
# for f in /usr/bin/arm-none-eabi-*; do ln -s "$f" /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/; done

# 라이브러리
arduino-cli lib install Dynamixel2Arduino
```

환경변수 (알림 끄기):

```bash
export ARDUINO_UPDATER_ENABLE_NOTIFICATION=false
```

## 2. 26.04 주의사항 (24.04 대비)

- `OPENCR_VERSION=1.5.3`, `OPENCR_SHA256=418656e5e...` 고정값 그대로 사용.
- 릴리스 파일명이 `.tar.bz2`지만 내용은 gzip → `tar xzf` 사용 (원본 주석 실측).
- `opencr_tools` 업로더는 x86_64만 제공 → arm64에서는 컴파일만 되고 업로드는 x86_64 실기 PC에서.
- FQBN: `OpenCR:OpenCR:OpenCR`

## 3. 검증

```bash
arduino-cli core list
arduino-cli lib list | grep -i dynamixel
ls -l /dev/opencr  # 실기 연결 시 (udev 고정 이름)
```

관련:
- Lyrical 본체: [ROS 2 Lyrical Ubuntu 26.04 세팅](../ros/lyrical/lyrical-setup-ubuntu26.04.md)
- 허브: [Ubuntu 26.04 정리](../ubuntu26_04/lyrical-setup-ubuntu26.04.md)
