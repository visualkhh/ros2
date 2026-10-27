# OpenCR 세팅 — Raspberry Pi 4 (Ubuntu 26.04, arm64) 기준

OpenCR은 로봇 Pi(`pak`)의 USB에 꽂혀 있고, 펌웨어 빌드·업로드도 **Pi에서** 합니다.
설치·빌드·업로드는 저장소의 스크립트가 다 하므로 이 문서는 순서와 주의점만 적습니다.

- 보드: OpenCR 1.0 · 바퀴 DYNAMIXEL XM430-W350 ×2 (ID 1·2, 1 Mbps, Protocol 2.0)
- 펌웨어: [`firmware/opencr/tb3_wheels`](../../firmware/opencr/README.md) (프로토콜 `C`/`T`/`R` → `S`/`E`)
- 보드 코어 1.5.3 고정 (sha256 확인) · 툴체인은 Ubuntu `gcc-arm-none-eabi` · 라이브러리는 코어에 든 DynamixelSDK
- ROS 설치: [Lyrical (Pi 기준)](../ros/lyrical/lyrical-setup-ubuntu26.04.md) — 펌웨어 작업만이면 ROS 없어도 됨

## 1. 순서 (Pi에서)

```bash
ssh pak
cd ~/ros2/firmware/opencr
./setup.sh            # 최초 1회: apt(gcc-arm-none-eabi 등) · arduino-cli · OpenCR 코어 · opencr_ld_shell_arm
./install-udev.sh     # 최초 1회: /dev/opencr (OpenCR) · /dev/tb3_lidar (LDS-03) 고정 이름 + 권한 0666
./compile.sh          # → build/tb3_wheels/tb3_wheels.ino.bin
./upload.sh           # /dev/opencr (없으면 /dev/ttyACM0)
./test-firmware       # S 줄 5개 읽기 (아무것도 안 보냄)
```

`./setup_compile_upload.sh` = setup + compile + upload 한 번에.

## 2. arm64(Pi)에서 달라지는 점 — `setup.sh`가 처리

| 항목 | x86_64 PC | arm64 Pi |
|---|---|---|
| 보드 코어 | `arduino-cli core install OpenCR:OpenCR` | 인덱스에 arm64 툴이 없어 릴리스 `opencr.tar.bz2`(1.5.3)를 받아 수동 설치. 이름은 `.tar.bz2`지만 내용은 gzip → `tar xzf` |
| 컴파일러 | 인덱스 `opencr_gcc`(32비트)를 시스템 `arm-none-eabi-*` 링크로 교체 | 같은 자리에 시스템 `arm-none-eabi-*` 링크 |
| 업로더 | `opencr_ld_shell_x86` | `opencr_ld_shell_arm` (32비트 ARM 바이너리) → `dpkg --add-architecture armhf` + `libc6:armhf` 필요 |
| 업로더 위치 | `~/.local/share/opencr_update/` | 같음 |

이전 판의 "arm64에서는 업로드 불가, x86 PC에서" 는 틀렸습니다 — ROBOTIS가 `opencr_ld_shell_arm`을 제공하고
`setup.sh`가 받습니다. `Dynamixel2Arduino` 설치도 필요 없습니다 (tb3_wheels는 코어 내장 DynamixelSDK 사용).

## 3. 포트·권한

```bash
lsusb | grep -E '0483:5740|10c4:ea60'   # OpenCR (STM Virtual COM) · LDS-03 USB2LDS (CP2102)
ls -l /dev/opencr /dev/tb3_lidar        # udev 설치 후 → ttyACM0 · ttyUSB0
```

| 장치 | USB ID | udev 이름 | 규칙 파일 | 쓰는 곳 |
|---|---|---|---|---|
| OpenCR | `0483:5740` | `/dev/opencr` | `firmware/opencr/99-opencr.rules` | 펌웨어 업로드 · `opencr_driver` |
| LDS-03 (USB2LDS) | `10c4:ea60` | `/dev/tb3_lidar` | `firmware/opencr/99-tb3-lidar.rules` | `coin_d4_driver` (`ros2_ws/src/tb3_bringup/config/lidar.yaml`) |

- 둘 다 `MODE=0666` · ModemManager 무시. `./install-udev.sh`가 두 규칙을 같이 설치한다. 규칙 없이 쓰려면 `sudo usermod -aG dialout $USER` 후 재로그인.
- `/dev/tb3_lidar`는 TurtleBot3 공식 udev 규칙과 같은 이름 (coin_d4_driver 기본 포트). CP2102는 흔한 칩이라 같은 칩의 다른 USB-시리얼을 꽂으면 그것도 이 이름을 가져갈 수 있음.
- 포트를 다른 프로그램(베이스 드라이버·시리얼 모니터)이 잡고 있으면 `upload.sh`·`test-firmware`가 거부 → `tool/run-cli stop-all`.

## 4. 문제 확인

- 업로드 응답 없음 → OpenCR **SW2를 누른 채 RESET** (부트로더 모드, LED 깜빡임) → `./upload.sh` 다시.
- `test-firmware`가 `응답 없음` → 펌웨어 미업로드 · 바퀴 전원(12V) · 모터 ID/보드레이트 확인. 모터가 없어도 `S`는 나옴 (값 0).
- `E,..` 줄 → 비트 해석·`R` 재부팅은 [firmware README](../../firmware/opencr/README.md).

```bash
arduino-cli core list            # OpenCR:OpenCR 1.5.3
ls ~/.local/share/opencr_update/ # opencr_ld_shell_arm · _x86
```

관련: [Lyrical (Pi)](../ros/lyrical/lyrical-setup-ubuntu26.04.md) · [허브](../ubuntu26_04/lyrical-setup-ubuntu26.04.md)
