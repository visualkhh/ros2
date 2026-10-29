# documents

TurtleBot3 Waffle Pi 개조 로봇 (Raspberry Pi 4 · Ubuntu 26.04 · ROS 2 Lyrical) 문서 목차.

| 폴더 | 내용 | 시작 문서 |
|---|---|---|
| [raspberry/](raspberry/) | **로봇 Pi(`pak`)** — 설치·설정 내역 요약, 처음부터 세팅 순서, SD카드·SSH·udev | [raspberry/README.md](raspberry/README.md) |
| [ros/lyrical/](ros/lyrical/) | ROS 2 Lyrical 설치 (Pi 기준, PC 추가분) · 프로젝트 패키지 · `ros2_ws` 빌드 | [lyrical-setup-ubuntu26.04.md](ros/lyrical/lyrical-setup-ubuntu26.04.md) |
| [opencr/](opencr/) | OpenCR 펌웨어 툴체인·컴파일·업로드 (arm64 Pi에서) | [opencr-setup-ubuntu26.04.md](opencr/opencr-setup-ubuntu26.04.md) |
| [ai/](ai/) | AI(Claude Code)가 Pi를 비밀번호 없이 다루게 하기 (SSH 키 · sudo) | [ai-passwordless-robot-access.md](ai/ai-passwordless-robot-access.md) |
| [docker/](docker/) | PC 시험대 (Lyrical 컨테이너 · 테스트 스크립트) | [docker/README.md](docker/README.md) |
| [isaac/](isaac/) | Isaac Sim/Lab RL 정책 만들기 (lab PC 학습 → Pi ONNX 추론) | [isaac/README.md](isaac/README.md) |
| [issue-tracking/](issue-tracking/) | 날짜별 작업·**시행착오** (증상 → 원인 → 해결 → 교훈) | [issue-tracking/README.md](issue-tracking/README.md) |
| [turtle-bot/](turtle-bot/) | 로봇 실물 사진 (맨 위 판 제거 후, 위에서 — 위쪽이 앞) | [images/robot-top.jpg](turtle-bot/images/robot-top.jpg) |

저장소의 다른 곳:

| 위치 | 내용 |
|---|---|
| [../ros2_ws/STATE_CONTROL.md](../ros2_ws/STATE_CONTROL.md) | **상태 바꾸기** — 제어 상태 · 경로(직결/RL) · 비상정지: `/cmd_manual_*` 요청 → `/state_*` 확인, tool 명령·메뉴 |
| [../ros2_ws/ARCH.md](../ros2_ws/ARCH.md) | **시스템 노드 계통도** — state_manager · cmd_manager(mux) 명령 경로 · TF · 토픽 · 장치 |
| [../ros2_ws/README.md](../ros2_ws/README.md) | 워크스페이스 패키지 · `deps.repos` · 빌드 · bringup · `/cmd_manual_*` · RL Before/After |
| [../tool/README.md](../tool/README.md) · [MAP_NAV.md](../tool/MAP_NAV.md) | 운영 CLI (`run-cli`, 토픽 보이는 곳 어디서나: 베이스·SLAM·Nav·텔레오퍼·지도·모드/RL/비상정지) · RViz (`run-rviz`) · 지도 만들기→주행 순서 |
| [../firmware/opencr/README.md](../firmware/opencr/README.md) | OpenCR 펌웨어 `tb3_wheels` 프로토콜 (`C`·`T`·`R` → `S`·`E`) · 스크립트 |

공통 규칙: `ROS_DOMAIN_ID=25` (PC·Pi·docker) · 코드 갱신은 PC에서 push → `ssh pak 'cd ~/ros2 && git pull'`.
