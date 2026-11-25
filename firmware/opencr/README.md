# OpenCR 2륜 펌웨어 (tb3_wheels)

TurtleBot3 Waffle Pi형 2륜 + OpenCR 최소 펌웨어. 바퀴 2개(XM430-W210, **ID 2 = 왼쪽 · ID 1 = 오른쪽**) 속도제어 + 상태 송신.
바퀴 + (선택) ROBOTIS OpenMANIPULATOR-X 4축·그리퍼 + 내장 IMU(`I` 줄). 팔이 없어도(뽑음·12V 꺼짐) 바퀴는 그대로 동작하고, 5초마다 팔을 다시 찾는다.
팔 쪽은 Lv2 `beebot_opencr`를 참고해 **기존 줄은 그대로 두고 줄을 추가**했다 (Pi 드라이버는 모르는 줄을 무시 — 드라이버 수정 없이 올릴 수 있음).
기준 문서: <https://docs.robotis.com/docs/systems/turtlebot3/manipulation/>

- 소스: `tb3_wheels/tb3_wheels.ino` (Arduino 스케치, 폴더명=파일명 유지 필수) · 팔 설정(ID·부호·그리퍼 표) `tb3_wheels/arm_config.h`
- 의존: OpenCR 보드 패키지 + DynamixelSDK (보드 설치는 [OpenCR 문서](../../documents/opencr/opencr-setup-ubuntu26.04.md))
- **현재(2026-10-30)**: `pak`에 업로드 완료. `/imu` 20 Hz 확인 (정지 `gz≈0`, `az≈9.98`). `S` 직후 `I` 줄이 오는 구조 그대로.

## 프로토콜 (USB CDC, `/dev/opencr`, 115200)

| 방향 | 형식 | 주기 |
|---|---|---|
| SBC → OpenCR | `C,<좌 rad/s>,<우 rad/s>` (빈칸=유지) | 제어 주기 |
| SBC → OpenCR | `T,0` / `T,1` (토크 off/on) | 필요시 |
| SBC → OpenCR | `R` (양쪽) / `R,<좌 0·1>,<우 0·1>` (다이나믹셀 재부팅, 빈칸·0=그대로) | 에러 시 |
| SBC → OpenCR | `B` (2000Hz 150ms) / `B,<Hz>,<ms>` (부저 삑, 100~8000Hz · 10~2000ms, 안 기다림) | 필요시 |
| OpenCR → SBC | `S,<좌pos rad>,<좌vel>,<우pos>,<우vel>,<uptime ms>,<좌전류 A>,<우전류 A>,<좌온도 ℃>,<우온도 ℃>,<배터리 V>` | 20Hz |
| OpenCR → SBC | `I,<gx>,<gy>,<gz rad/s>,<ax>,<ay>,<az m/s²>,<qw>,<qx>,<qy>,<qz>` (OpenCR 내장 IMU, 보드 좌표 = `imu_link`). 부팅 때 자이로 바이어스 보정 → **켜고 ~2초 가만히** | `S` 바로 뒤 (20Hz) |
| OpenCR → SBC | `E,<좌err>,<우err>` (Hardware Error Status 비트마스크, 0=정상, -1=무응답) | 변화 시 + 에러 중 1Hz |

### 팔 줄 (추가)

모터: joint1~4 = ID 11~14, 그리퍼 = ID 15 (DYNAMIXEL Wizard로 미리 설정, 1 Mbps, Protocol 2.0). 0 rad = 서보 중앙(2048). 그리퍼는 손가락 하나 m (0 = 닫힘).

| 방향 | 형식 | 비고 |
|---|---|---|
| SBC → OpenCR | `A,<j1>,<j2>,<j3>,<j4 rad>,<그리퍼 m>[,<그리퍼 전류 A>]` | 빈칸 = 유지. 한 패킷으로 동시 반영. 그리퍼 전류 = 쥐는 힘 한계 |
| SBC → OpenCR | `AT,0` · `AT,1` · `AT,<5칸 0/1>` | 팔 토크. 켤 때 지금 자세가 목표 |
| SBC → OpenCR | `AR,1` · `AR,<5칸 0/1>` | 팔 모터 재부팅 (Hardware Error 해제) |
| OpenCR → SBC | `M,<5 × (pos, vel, 전류 A)>,<5 온도 ℃>` | 20Hz, `I` 뒤. 못 읽은 값 `nan` · 온도 −1 |
| OpenCR → SBC | `F,<6 에러>` | `E`와 같음 (변화 시 + 에러 중 1Hz) |

안전: 500ms 동안 `A`가 없으면 현재 자세 유지. 12V를 껐다 켜 토크가 풀린 모터는 0.5초 안에 다시 켜고 느린 속도로 복귀. 관절 부호(`ARM_DIR`)와 그리퍼 쥐는 전류는 아직 **실측 전** (Lv2 값 그대로).

자세 유지: 위치 PID `P=500, I=30, D=0` (`arm_config.h`). P만 쓰면 중력 토크 때문에 정상상태 오차가 남아 팔이 천천히 처지므로 I로 0을 만든다. SBC는 300ms마다 같은 `A`를 다시 보내 이 500ms 타임아웃이 "처진 위치"를 새 목표로 덮어쓰지 않게 한다 (드라이버 `tick()`).

에러(`E`) 비트: `1`=입력전압 `4`=과열 `8`=엔코더 `16`=전기충격 `32`=과부하 (여러 개면 합, 예 `36`=과열+과부하). 에러가 나면 다이나믹셀이 스스로 토크를 끔 → `R`로 재부팅해야 해제. 정상 복귀 시 `E,0,0` 한 번 송신.

재부팅(`R`): 과열·과부하 등 하드웨어 에러 해제용. 재부팅된 바퀴는 속도모드·목표 0·현재 `T` 상태로 복구. 다회전 위치가 1바퀴 이내로 초기화되므로 `S` pos가 튐 → SBC 오도메트리에서 처리 필요. 바퀴당 최대 ~1초 블로킹.

안전: 500ms(`CMD_TIMEOUT_MS`) 동안 `C` 없으면 자동 정지. 속도 ±8.0 rad/s clip (Waffle Pi 사양 0.26 m/s ÷ 바퀴 반지름 0.033 m ≈ 7.9).

## 스크립트 (로봇 Pi·PC 공통, 이 폴더에서 실행)

```text
firmware/opencr/
├── tb3_wheels/              # 스케치 (폴더명=파일명)
├── common.sh                # 공통 변수 (source 전용)
├── setup.sh                 # arduino-cli + OpenCR 보드 코어(DynamixelSDK 포함) + 업로더 — 최초 1회
├── compile.sh               # → build/tb3_wheels/tb3_wheels.ino.bin
├── upload.sh                # .bin → .opencr 변환 후 업로드
├── setup_compile_upload.sh  # setup + compile + upload 한 번에
└── test-firmware            # ROS 없이 시리얼로 S/E 읽기 · C/T/R 보내기
```

```bash
./setup.sh                 # sudo 필요 (apt · arduino-cli)
~/ros2/documents/raspberry/udev/install-udev.sh   # 최초 1회: /dev/opencr — documents/raspberry/udev-setup.md
./compile.sh
./upload.sh                # 기본 포트 /dev/opencr → 없으면 /dev/ttyACM0
./test-firmware            # S 5줄 (읽기만 — 아무것도 안 보냄)
./test-firmware --wheel 0.5 --yes          # 바퀴 띄우고! 2초 회전 후 정지
./test-firmware --reboot --yes             # 에러(E) 났을 때 재부팅
```

- 업로드가 응답 없으면: OpenCR **SW2를 누른 채 RESET** → 부트로더 모드에서 `./upload.sh` 다시.
- 포트를 다른 프로그램(ROS 드라이버·tool의 베이스·시리얼 모니터)이 잡고 있으면 `upload.sh`·`test-firmware`가 거부함 → 먼저 종료 (`tools/run-cli stop-all`).
- `/dev/opencr` 고정 이름·권한: [udev 세팅](../../documents/raspberry/udev-setup.md). 없이 쓰려면 `sudo usermod -aG dialout $USER` 후 재로그인, 포트는 `/dev/ttyACM0`.

## SBC 연결 계획

- `test-firmware` 방식 그대로 사용 가능 (`/dev/opencr` 가상/실물 동일 이름)
- 지금은 `tb3_bringup/opencr_driver`가 `C` 송신·`S`·`I`·`E` 파싱. ros2_control 하드웨어 플러그인으로 옮길 때도 프로토콜 그대로
- LiDAR: LDS-03 → `coin_d4_driver`, SLAM: cartographer ([ros2_ws/README.md](../../ros2_ws/README.md))
