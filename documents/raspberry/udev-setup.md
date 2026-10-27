# udev — USB 장치 고정 이름 (`/dev/opencr` · `/dev/tb3_lidar`)

USB 시리얼은 꽂는 순서에 따라 `ttyACM0`/`ttyUSB0`… 번호가 바뀝니다. udev 규칙으로 **장치마다 고정 이름**을 붙이고
**권한(0666)** 도 열어서, 드라이버·펌웨어 스크립트가 항상 같은 경로를 쓰게 합니다. 로봇 Pi에 최초 1회 (PC에서 OpenCR을 꽂아 쓸 때도 같음).

파일: [`udev/`](udev/) — `99-opencr.rules` · `99-tb3-lidar.rules` · `install-udev.sh`

## 1. 설치

```bash
ssh pak
~/ros2/documents/raspberry/udev/install-udev.sh            # sudo 필요
# OK: 설치 (99-opencr.rules 99-tb3-lidar.rules)
# /dev/opencr -> ttyACM0
# /dev/tb3_lidar -> ttyUSB0

~/ros2/documents/raspberry/udev/install-udev.sh --remove   # 제거
```

하는 일: 규칙을 `/etc/udev/rules.d/`에 복사 → `udevadm control --reload` → `udevadm trigger` (다시 꽂지 않아도 바로 적용).

## 2. 규칙

| 장치 | USB ID (`lsusb`) | 고정 이름 | 규칙 파일 | 쓰는 곳 |
|---|---|---|---|---|
| OpenCR | `0483:5740` STMicroelectronics Virtual COM Port | `/dev/opencr` | `udev/99-opencr.rules` | 펌웨어 업로드·시험 ([OpenCR](../opencr/opencr-setup-ubuntu26.04.md)) · `opencr_driver` (`tb3_bringup/config/robot.yaml`) |
| LDS-03 (USB2LDS) | `10c4:ea60` Silicon Labs CP2102 | `/dev/tb3_lidar` | `udev/99-tb3-lidar.rules` | `coin_d4_driver` (`tb3_bringup/config/lidar.yaml`) |

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="opencr",    MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="tb3_lidar", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
```

- `MODE="0666"`: `dialout` 그룹 없이 읽기/쓰기. (규칙 대신 `sudo usermod -aG dialout $USER` + 재로그인도 가능하지만 이름은 고정 안 됨)
- `ID_MM_DEVICE_IGNORE`: ModemManager가 시리얼 포트를 모뎀으로 착각해 몇 초간 잡는 것을 막음 (TurtleBot3 규칙과 같음).
- `/dev/tb3_lidar`는 TurtleBot3 공식 규칙과 같은 이름 = `coin_d4_driver` 기본 포트.

## 3. 확인 · 문제

```bash
lsusb | grep -E '0483:5740|10c4:ea60'
ls -l /dev/opencr /dev/tb3_lidar /dev/serial/by-id/
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID'   # 규칙과 ID가 맞는지
```

- 이름이 안 생김 → USB ID가 다름 (`lsusb`로 확인 후 규칙 수정) · 다시 `install-udev.sh`.
- **CP2102는 흔한 USB-시리얼 칩**이라 같은 칩의 다른 장치를 꽂으면 그것도 `/dev/tb3_lidar`가 될 수 있음. 그럴 땐 규칙에
  `ATTRS{serial}=="…"`(`udevadm info -a -n /dev/ttyUSB0 | grep serial`)을 추가해 이 라이다만 집어낸다.
- 포트를 다른 프로그램이 잡고 있으면 펌웨어 업로드·`test-firmware`가 거부 → `tool/run-cli stop-all`.

관련: [OpenCR 세팅](../opencr/opencr-setup-ubuntu26.04.md) · [Lyrical (Pi)](../ros/lyrical/lyrical-setup-ubuntu26.04.md) · [노드 계통도](../../ros2_ws/ARCH.md)
