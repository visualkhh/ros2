# raspberry — 로봇 Pi (`pak`)

로봇 SBC 세팅 문서 모음. **1장 = 지금 설치·설정된 내역(스펙 요약)**, **2장 = 처음부터 세팅하는 순서**.

## 1. 설치·설정 요약 (2026-10-28 실측)

### 하드웨어

| 항목 | 값 |
|---|---|
| 로봇 | TurtleBot3 **Waffle Pi** 개조 — 맨 위 판 제거, LDS-03 라이다를 1층 맨 앞(두 바퀴 사이 가운데) |
| SBC | Raspberry Pi 4 Model B Rev 1.5 · RAM 4GB(3.7Gi) · 4코어 · arm64 |
| 저장 | SD 32GB (`/` 29G, 여유 12G) · swap 2GB (`/swapfile`) |
| 바퀴 제어 | OpenCR 1.0 + DYNAMIXEL XM430-W350 ×2 (ID 2 왼쪽 · ID 1 오른쪽, 1 Mbps, Protocol 2.0) |
| 라이다 | LDS-03 (USB2LDS · CP2102) — 360°, 0.05~12 m, ~10 Hz |
| 바퀴 치수 | 반지름 0.033 m · 간격 0.287 m (Waffle Pi) |

### OS · 계정 · 네트워크

| 항목 | 값 | 문서 |
|---|---|---|
| OS | Ubuntu 26.04.1 LTS Server (resolute) · 커널 7.0.0-1020-raspi · Python 3.14 | [SD카드 설치](raspberry-pi-ubuntu-sdcard-os-install.md) |
| hostname / 사용자 | `pak` / `pak` (sudo 그룹) | |
| 접속 | `ssh pak` = `pak@pak.local` (avahi mDNS) · SSH 키 인증 | [SSH](ssh-pak-local-troubleshooting.md) |
| sudo | 비번 없음 (`/etc/sudoers.d/90-pak`, NOPASSWD) — AI가 직접 작업용 | [AI 접근](../ai/ai-passwordless-robot-access.md) |
| WiFi | `KANT_Teamsparta` · DHCP (IP는 바뀜, 예: 10.2.12.142 → `pak.local`로 접속) | |
| 시간 | Asia/Seoul · NTP 동기화 (RTC 없음 — 인터넷 없이 부팅하면 시계 틀림) | |
| apt | arm64 = archive.ubuntu.com · armhf = ports.ubuntu.com (`ubuntu-ports-armhf.sources`, OpenCR 업로더용) | [OpenCR](../opencr/opencr-setup-ubuntu26.04.md) |

### ROS 2

| 항목 | 값 | 문서 |
|---|---|---|
| 배포판 | Lyrical (ros-base 0.13.0) · 공식 apt | [Lyrical (Pi)](../ros/lyrical/lyrical-setup-ubuntu26.04.md) |
| apt 패키지 | nav2_bringup 1.5 · nav2 amcl/map_server · laser_filters 2.3 · robot_state_publisher · xacro · teleop_twist_keyboard · tf2_tools · turtlebot3_msgs · dynamixel_sdk · ros-dev-tools (+ libceres-dev 등 cartographer 빌드 의존) | |
| 소스 의존 | `coin_d4_driver` (LDS-03) · `explore_lite` (자동 맵핑) · `cartographer`+`cartographer_ros` (SLAM) — `ros2_ws/deps.repos`로 받아 `patches/*.patch` 적용 후 빌드 · `/scan` 10 Hz | [ros2_ws README](../../ros2_ws/README.md#depsrepos--apt에-없는-외부-소스) |
| 워크스페이스 | `~/ros2` (GitHub clone, pull 전용) · `~/ros2/ros2_ws` 빌드: tb3_bringup · coin_d4_driver · cartographer(·_ros·_msgs) · rl_correction · rl_eval · nav2_rl_controller | [ARCH](../../ros2_ws/ARCH.md) |
| 상시 노드 (`run-cli base start` = tb-only) | state_manager · cmd_manager · opencr_driver · robot_state_publisher · coin_d4_driver · scan_filter · scan_resample | [ARCH 2장](../../ros2_ws/ARCH.md) |
| 필요할 때 (`run-cli`, 어디서나) | cartographer (`slam start`: `cartographer.launch.py` + `config/cartographer_2d.lua`) · Nav2 (`nav start` / `explore start`) — 지도 `~/ros2/ros2_ws/map/` | [tool](../../tool/README.md) |
| `~/.bashrc` | `source /opt/ros/lyrical/setup.bash` · ros2_ws `install/setup.bash` · `ROS_DOMAIN_ID=25` · `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` | |

### 장치 · 펌웨어

| 항목 | 값 | 문서 |
|---|---|---|
| udev | `/dev/opencr` → ttyACM0 · `/dev/tb3_lidar` → ttyUSB0 (`99-opencr.rules` · `99-tb3-lidar.rules`) | [udev](udev-setup.md) |
| 펌웨어 툴 | arduino-cli 1.5.1 · OpenCR 코어 1.5.3 (min/max 패치) · gcc-arm-none-eabi 14.2 · `opencr_ld_shell_arm` | [OpenCR](../opencr/opencr-setup-ubuntu26.04.md) |
| OpenCR 펌웨어 | `tb3_wheels` 업로드됨 (**ID 2 = 왼쪽 · ID 1 = 오른쪽**) — `S` 20 Hz · 텔레오퍼 주행 확인 | [firmware](../../firmware/opencr/README.md) |
| 바퀴 방향 | `robot.yaml` `left_dir: -1` · `right_dir: -1` (두 모터 모두 +속도 = 뒤로) | [OpenCR 4장](../opencr/opencr-setup-ubuntu26.04.md) |

### 아직 안 한 것

- [x] `coin_d4_driver` 받아서 빌드 → `/scan` 10 Hz (뒤 135°는 몸통에 가려 안 보임 — 정상)
- [x] 바퀴 ID·방향 확인 (펌웨어 ID 2=왼쪽·1=오른쪽, dir -1/-1)
- [ ] 실기 SLAM → 지도 저장 → Nav 목표 주행 ([MAP_NAV](../../tool/MAP_NAV.md))
- [ ] 몸통 필터 범위 확인 (앞 최근접 0.17~0.2 m 점이 몸통인지)
- [ ] URDF 실측 값 (`lidar_x` · `lidar_z` · `lidar_yaw` · `plate_z2`) — `ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro`

## 2. 처음부터 세팅하는 순서

| # | 단계 | 문서 | 끝나면 |
|---|---|---|---|
| 1 | SD카드에 Ubuntu 26.04 Server 굽기 (WiFi · SSH 비번 접속) | [raspberry-pi-ubuntu-sdcard-os-install.md](raspberry-pi-ubuntu-sdcard-os-install.md) | 첫 부팅 |
| 2 | IP 찾아 접속 · `pak.local`(avahi) · SSH 키 | [ssh-pak-local-troubleshooting.md](ssh-pak-local-troubleshooting.md) | `ssh pak` |
| 3 | (AI가 직접 작업하게) sudo 비번 없이 | [../ai/ai-passwordless-robot-access.md](../ai/ai-passwordless-robot-access.md) | `ssh pak 'sudo -n true'` |
| 4 | USB 장치 고정 이름 | [udev-setup.md](udev-setup.md) | `/dev/opencr` · `/dev/tb3_lidar` |
| 5 | ROS 2 Lyrical + 프로젝트 패키지 · `ros2_ws` 빌드 | [../ros/lyrical/lyrical-setup-ubuntu26.04.md](../ros/lyrical/lyrical-setup-ubuntu26.04.md) | `./run-cli status` |
| 6 | OpenCR 펌웨어 (툴체인 · 컴파일 · 업로드) | [../opencr/opencr-setup-ubuntu26.04.md](../opencr/opencr-setup-ubuntu26.04.md) | `./test-firmware`에 `S` 줄 |
| 7 | 운영 (베이스 · SLAM · Nav · 텔레오퍼) | [../../tool/README.md](../../tool/README.md) | 지도 만들기 |

- 시스템 구성(노드·TF·토픽): [../../ros2_ws/ARCH.md](../../ros2_ws/ARCH.md)
- 코드 갱신: PC에서 push → `ssh pak 'cd ~/ros2 && git pull'` (ros2_ws를 바꿨으면 빌드 후 `./run-cli base restart`)
- 운영은 토픽이 보이는 곳 어디서나 `tool/run-cli` (켜기/끄기·지도는 tool이 SSH로 Pi에 넘김)
- `ROS_DOMAIN_ID=25` (PC·Pi·docker 공통). PC 시험대(Docker): [../docker/README.md](../docker/README.md)

## 3. 이 폴더

```text
raspberry/
├── README.md                                 # 이 문서
├── raspberry-pi-ubuntu-sdcard-os-install.md  # SD카드 OS 설치
├── ssh-pak-local-troubleshooting.md          # 접속·IP 찾기·SSH 키·pak.local
├── udev-setup.md                             # /dev/opencr · /dev/tb3_lidar
├── udev/                                     # 규칙 2개 + install-udev.sh
└── image/                                    # 설치 화면 캡처
```
