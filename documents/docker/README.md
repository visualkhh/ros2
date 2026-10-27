# Docker (26.04 Lyrical) — 문서 + 설정파일 세트

원본: `/home/pank/workspaces/pa/source/Lv2_bee_Assignment/docker/` 13종 + `fastdds.xml`, `fastdds_local.xml`
이 폴더는 26.04 Lyrical용으로 검증·정리한 복사본. `Dockerfile`, `docker-compose.yml`은 26.04로 변환, `.sh`는 원본 그대로(26.04 호환 확인됨).

관련: [Lyrical 세팅](../ros/lyrical/lyrical-setup-ubuntu26.04.md) · [OpenCR 세팅](../opencr/opencr-setup-ubuntu26.04.md) · [Pi 세팅 순서](../raspberry/lyrical-setup.md)

## 파일 목록

- `Dockerfile` — 26.04 변환됨 (FROM 26.04, Isaac 삭제, resolute apt, python3.13)
- `docker-compose.yml` — 템플릿화 (Lv2 경로 → `./example_ws` 예시, shm/domain/discovery 유지)
- `entrypoint.sh` — 가상 시리얼 브릿지 + /debug 초기화 (그대로)
- `fastdds.xml` / `fastdds_local.xml` — TypeLookup OFF + SHM 32MB (그대로)
- `build-ws.sh` — colcon 빌드 공용 (그대로)
- `test-serial.sh`, `test-firmware.sh` — 시리얼/OpenCR 검증 (그대로, 26.04 호환)
- `test-packages.sh`, `test-hardware.sh`, `test-bringup.sh`, `test-faker.sh`, `test-all.sh` — Lv2 전용 검증 (그대로, 아래 표 참조)
- `ros-kill.sh`, `ros-restart.sh`, `ros-faker-restart.sh`, `reset-testbed.sh` — 실행/정리 (그대로)

## 검증 결과 (26.04 기준)

| 파일 | 24.04 전용 문자열? | 판정 | 비고 |
|---|---|---|---|
| `Dockerfile` | Isaac 2줄, noble deb, python3.12 | 수정됨 | 26.04 변환 완료 |
| `docker-compose.yml` | `../lv2_module6/...` 경로 | 템플릿화 | shm 4gb, host, DOMAIN_ID, SUBNET, FASTDDS 유지 |
| `entrypoint.sh` | 없음 (socat+PTY, ROS source) | 그대로 사용 | lyrical 경로만 사용, noble 의존 없음 |
| `build-ws.sh` | 없음 (colcon) | 그대로 사용 | 컨테이너 `/ws` 우선이라 호환 |
| `test-serial.sh` | 없음 (stty+socat) | 그대로 사용 | 26.04 호환 |
| `test-firmware.sh` | 없음 (arduino-cli FQBN) | 그대로 사용 | OpenCR 1.5.3 고정, 26.04 문서와 일치 |
| `test-packages.sh` | 없음 (colcon test) | 그대로 사용 | Lv2 DEVELOP.md 언급만 주석, 동작 무관 |
| `test-hardware.sh` | beebot/firmware 경로 주석 | Lv2 전용, 그대로 보관 | ros_system+fake_opencr 없으면 FAIL — Lv2 레포에서만 실행 |
| `test-bringup.sh` | bringup/노드명 하드코딩 | Lv2 전용, 그대로 보관 | NODES 목록이 Lv2 bringup 기준 |
| `test-faker.sh` | faker launch 하드코딩 | Lv2 전용, 그대로 보관 | faker 패키지 없으면 실행 불가 |
| `test-all.sh` | 위 4종 순서 실행 | Lv2 전용, 그대로 보관 | push 전 게이트용 |
| `ros-kill.sh` | Gazebo/rviz 패턴 포함 | 그대로 사용 (일반+Lv2) | ROS 정리+shm 삭제는 공용, gz 패턴은 없어도 무해 |
| `ros-restart.sh` | bringup.launch.py 하드코딩 | Lv2 전용, 그대로 보관 | 자신의 launch명으로 바꿔 사용 |
| `ros-faker-restart.sh` | faker/state_manager 하드코딩 | Lv2 전용, 그대로 보관 | faker 없으면 ros-kill+test-faker 경로라 Lv2 전용 |
| `reset-testbed.sh` | test-faker 호출 | Lv2 전용, 그대로 보관 | `compose down→build→up→test-faker` 패턴은 공용으로 참고 가능 |
| `fastdds.xml` | 없음 | 그대로 사용 | 32MB SHM + type_propagation off |
| `fastdds_local.xml` | Jetson 격리용 주석 | 그대로 보관 | 한 기기 SHM-only, 주석만 Jetson 언급 |

검증 방법: `grep -rn "noble\|24\.04\|isaac" --include="*.sh" .` → `.sh`에는 없음 확인. `Dockerfile`/`compose`만 26.04로 손봄. `bash -n *.sh` 문법 통과.

## 사용법

```bash
cd documents/docker
docker compose build
docker compose up -d && docker compose exec lyrical bash
# 컨테이너 안
build-ws
test-serial
test-firmware
test-all   # Lv2 레포 src가 있을 때만 PASS
```

호스트에서 쓸 때 `WS` 경로 주의: 스크립트는 `/ws/src` 우선, 없으면 `../lv2_module6/ros2_ws` 폴백. 이 레포에 ros2_ws가 없으면 컨테이너 안에서만 실행하거나 `WS=/path/to/ros2_ws build-ws`처럼 환경변수로 지정.

## 주의 (Lv2 전용 포함 이유)

전부 의미 있어서 보관. 단 `test-hardware/brringup/faker`, `ros-restart/faker-restart`, `reset-testbed`, `test-all`은 Lv2 패키지(`bringup`, `faker`, `ros_system`, `beebot`)가 있어야 PASS. 일반 26.04 Lyrical 프로젝트에서는 `entrypoint/build-ws/test-serial/test-firmware/ros-kill/fastdds`만 가져가면 됨.
