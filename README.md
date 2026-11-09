# ros2 — TurtleBot3 Waffle Pi 개조 로봇 `pak`

Raspberry Pi 4 · Ubuntu 26.04 · ROS 2 Lyrical 로봇의 bringup · 운영 도구 · OpenCR 펌웨어 · Isaac RL 보정.

| 폴더 | 내용 | 시작 문서 |
|---|---|---|
| [documents/](documents/) | 문서 목차 (Pi 세팅 · ROS 설치 · OpenCR · Isaac · 시행착오) | [documents/README.md](documents/README.md) |
| [ros2_ws/](ros2_ws/) | ROS 2 워크스페이스 (bringup · RL 보정 · 외부 소스 `deps.repos`) | [ros2_ws/README.md](ros2_ws/README.md) · [ARCH.md](ros2_ws/ARCH.md) |
| [tools/](tools/) | 운영 CLI `run-cli` · RViz `run-rviz` (베이스·SLAM·Nav·텔레오퍼·지도) | [tools/README.md](tools/README.md) · [MAP_NAV.md](tools/MAP_NAV.md) |
| [isaac/](isaac/) | Isaac Lab RL 이동 보정 — sim 보정 · 학습 · ONNX | [isaac/README.md](isaac/README.md) |
| [firmware/](firmware/) | OpenCR 펌웨어 `tb3_wheels` (프로토콜 · 업로드) | [firmware/opencr/README.md](firmware/opencr/README.md) |
| [setup/](setup/) | Pi 빌드 스크립트 (`build.sh`) | [setup/pi/README.md](setup/pi/README.md) |
| [todo/](todo/) | 날짜별 할 일 · 테스트 체크리스트 | [todo/README.md](todo/README.md) |

## 빠른 시작

```bash
ssh pak 'cd ~/ros2 && git pull && ./setup/pi/build.sh'   # Pi: 최신화 + 빌드
ssh pak 'cd ~/ros2 && ./tools/run-cli base start'         # Pi: 베이스 (바퀴·odom·라이다)
./tools/run-cli                                           # PC: 메뉴 (다른 터미널에서 ./tools/run-rviz slam = 지도 화면)
```

공통 규칙: `ROS_DOMAIN_ID=25` (PC·Pi·docker). 코드 갱신은 PC에서 push → Pi에서 `git pull` (Pi는 pull 전용).
