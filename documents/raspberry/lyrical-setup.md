# 로봇 Pi 세팅 순서 (Raspberry Pi 4 · Ubuntu 26.04 · ROS 2 Lyrical)

로봇 SBC `pak`을 처음부터 쓸 수 있게 만드는 순서. 각 단계는 해당 문서를 따라갑니다 (여기는 목차).

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
- 코드 갱신: PC에서 push → `ssh pak 'cd ~/ros2 && git pull'`
- `ROS_DOMAIN_ID=25` (PC·Pi·docker 공통). PC 시험대(Docker): [../docker/README.md](../docker/README.md)

기준: Raspberry Pi 4 Model B 4GB · Ubuntu 26.04.1 LTS · TurtleBot3 Waffle Pi 개조(맨 위 판 제거, LDS-03 1층 앞) · 2026-10-27
