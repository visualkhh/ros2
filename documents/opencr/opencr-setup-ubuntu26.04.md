# OpenCR 세팅 — Raspberry Pi 4 (Ubuntu 26.04, arm64) 기준

OpenCR은 로봇 Pi(`pak`)의 USB에 꽂혀 있고, 펌웨어 빌드·업로드도 **Pi에서** 합니다.
설치·빌드·업로드는 저장소의 스크립트가 다 하므로 이 문서는 순서와 주의점만 적습니다.

- 보드: OpenCR 1.0 · 바퀴 DYNAMIXEL XM430-W210 ×2 (ID 2 왼쪽 · ID 1 오른쪽, 1 Mbps, Protocol 2.0)
- 펌웨어: [`firmware/opencr/tb3_wheels`](../../firmware/opencr/README.md) (프로토콜 `C`/`T`/`R`/`B` → `S`/`I`/`E`)
- 보드 코어 1.5.3 고정 (sha256 확인) · 툴체인은 Ubuntu `gcc-arm-none-eabi` · 라이브러리는 코어에 든 DynamixelSDK
- ROS 설치: [Lyrical (Pi 기준)](../ros/lyrical/lyrical-setup-ubuntu26.04.md) — 펌웨어 작업만이면 ROS 없어도 됨

## 1. 순서 (Pi에서)

```bash
ssh pak
cd ~/ros2/firmware/opencr
./setup.sh            # 최초 1회: apt(gcc-arm-none-eabi 등) · arduino-cli · OpenCR 코어 · opencr_ld_shell_arm
~/ros2/documents/raspberry/udev/install-udev.sh   # 최초 1회: /dev/opencr · /dev/tb3_lidar → udev 문서
./compile.sh          # → build/tb3_wheels/tb3_wheels.ino.bin
./upload.sh           # /dev/opencr (없으면 /dev/ttyACM0)
./test-firmware       # S 줄 5개 읽기 (아무것도 안 보냄)
```

> 2026-10-30: `I` 줄 포함 펌웨어 업로드 완료. `/imu` 20 Hz 확인 (정지 시 `gz≈0`, `az≈9.98`).

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

`/dev/opencr` 고정 이름과 권한은 udev 규칙으로 만듭니다 → **[udev 세팅](../raspberry/udev-setup.md)** (라이다 `/dev/tb3_lidar`도 같이).

- 포트를 다른 프로그램(베이스 드라이버·시리얼 모니터)이 잡고 있으면 `upload.sh`·`test-firmware`가 거부 → `tool/run-cli stop-all`.

## 4. 바퀴 ID · 방향 점검 (새 로봇 · 모터 교체 · 배선 변경 후 1회)

ROS 없이 펌웨어만으로 **바퀴 하나씩** 돌려서 두 가지를 따로 확인합니다. 로봇을 들어 **바퀴를 띄우고**, 베이스는 끈 상태에서.

```bash
~/ros2/tool/run-cli base stop            # 포트 비우기 (opencr_driver가 /dev/opencr를 잡고 있음)
cd ~/ros2/firmware/opencr
./test-firmware --wheel 0.5 0 --yes      # C의 첫 값 = 왼쪽만 돌아야 함 (2초)
./test-firmware --wheel 0 0.5 --yes      # 둘째 값 = 오른쪽만
~/ros2/tool/run-cli base start
```

| 결과 | 원인 | 고칠 곳 |
|---|---|---|
| 엉뚱한 바퀴가 돈다 | **펌웨어 ID 매핑** | `firmware/opencr/tb3_wheels/tb3_wheels.ino` `WHEEL_LEFT_ID`·`WHEEL_RIGHT_ID` → compile·upload |
| 맞는 바퀴인데 +0.5에 뒤로 구른다 | 모터 장착 방향 | `ros2_ws/src/tb3_bringup/config/robot.yaml` `left_dir`·`right_dir` = -1 (펌웨어 그대로) |
| 맞는 바퀴가 앞으로 | 정상 | 그 바퀴 dir = +1 |

순서가 중요: **ID를 먼저** 맞추고 방향을 본다. ID가 틀린 채 방향 부호만 바꾸면 직진은 맞아도 회전이 반대가 되고 odom 좌우가 뒤집힌다.

마지막으로 ROS로 확인: `./run-cli teleop` — `w` 직진 · `a` 제자리 좌회전(위에서 반시계) · `d` 우회전(시계).

이 로봇(2026-10-27): **ID 2 = 왼쪽 · ID 1 = 오른쪽**, 두 모터 모두 +속도 = 뒤로 → `left_dir: -1` · `right_dir: -1`.

## 5. 문제 확인

- 업로드 응답 없음 → OpenCR **SW2를 누른 채 RESET** (부트로더 모드, LED 깜빡임) → `./upload.sh` 다시.
- `test-firmware`가 `응답 없음` → 펌웨어 미업로드 · 바퀴 전원(12V) · 모터 ID/보드레이트 확인. 모터가 없어도 `S`는 나옴 (값 0).
- `E,..` 줄 → 비트 해석·`R` 재부팅은 [firmware README](../../firmware/opencr/README.md).

```bash
arduino-cli core list            # OpenCR:OpenCR 1.5.3
ls ~/.local/share/opencr_update/ # opencr_ld_shell_arm · _x86
```

관련: [Lyrical (Pi)](../ros/lyrical/lyrical-setup-ubuntu26.04.md) · [Pi 세팅 순서](../raspberry/README.md)
