# documents

TurtleBot3 Waffle Pi 개조 로봇 (Raspberry Pi 4 · Ubuntu 26.04 · ROS 2 Lyrical) 문서 목차.

| 폴더 | 내용 | 시작 문서 |
|---|---|---|
| [raspberry/](raspberry/) | **로봇 Pi(`pak`)** — 설치·설정 내역 요약, 처음부터 세팅 순서, SD카드·SSH·udev | [raspberry/README.md](raspberry/README.md) |
| [ros/lyrical/](ros/lyrical/) | ROS 2 Lyrical 설치 (Pi 기준, PC 추가분) · 프로젝트 패키지 · `ros2_ws` 빌드 | [lyrical-setup-ubuntu26.04.md](ros/lyrical/lyrical-setup-ubuntu26.04.md) |
| [opencr/](opencr/) | OpenCR 펌웨어 툴체인·컴파일·업로드 (arm64 Pi에서) | [opencr-setup-ubuntu26.04.md](opencr/opencr-setup-ubuntu26.04.md) |
| [ai/](ai/) | AI(Claude Code)가 Pi를 비밀번호 없이 다루게 하기 (SSH 키 · sudo) | [ai-passwordless-robot-access.md](ai/ai-passwordless-robot-access.md) |
| [docker/](docker/) | PC 시험대 (Lyrical 컨테이너 · 테스트 스크립트) | [docker/README.md](docker/README.md) |
| [작업이력/](작업이력/) | 날짜별 작업·**시행착오** (증상 → 원인 → 해결 → 교훈) | [작업이력/README.md](작업이력/README.md) |
| [turtle-bot/](turtle-bot/) | 로봇 실물 사진 (맨 위 판 제거 후) | [images/img.png](turtle-bot/images/img.png) |

저장소의 다른 곳:

| 위치 | 내용 |
|---|---|
| [../ros2_ws/ARCH.md](../ros2_ws/ARCH.md) | **시스템 노드 계통도** — 노드·TF·토픽·장치, tb-only / tb-for-rl / rl-only |
| [../ros2_ws/README.md](../ros2_ws/README.md) | 워크스페이스 패키지 · bringup 사용법 · RL Before/After |
| [../tool/README.md](../tool/README.md) | 운영 CLI (`run-cli`: 베이스·SLAM·Nav·텔레오퍼·지도 저장) · RViz (`run-rviz`) |
| [../firmware/opencr/README.md](../firmware/opencr/README.md) | OpenCR 펌웨어 `tb3_wheels` 프로토콜 (`C`·`T`·`R` → `S`·`E`) · 스크립트 |

공통 규칙: `ROS_DOMAIN_ID=25` (PC·Pi·docker) · 코드 갱신은 PC에서 push → `ssh pak 'cd ~/ros2 && git pull'`.
