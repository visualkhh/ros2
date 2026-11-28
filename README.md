# ros2 — TurtleBot3 Waffle Pi 개조 로봇 `pak`

Raspberry Pi 4 · Ubuntu 26.04 · ROS 2 Lyrical 로봇의 bringup · 운영 도구 · OpenCR 펌웨어 · Isaac RL 보정.

| 폴더 | 내용 | 시작 문서 |
|---|---|---|
| [srf/](srf/) | **SRF — Simple Robot Framework** (git submodule, [dooboostore-develop/srf](https://github.com/dooboostore-develop/srf)) — C++20 로봇 프레임워크 + 시뮬레이터 + Qt/웹 뷰어 | [srf/README.md](srf/README.md) · [아래 submodule 사용법](#srf-git-submodule) |
| [srf-default-ws/](srf-default-ws/) | SRF로 만든 실기 TurtleBot3 Pi 워크스페이스 — OpenCR/LDS-03 C++ 드라이버 + EKF + 원격 viz (`../srf`를 링크) | [srf-default-ws/README.md](srf-default-ws/README.md) |
| [docker/](docker/) | SRF용 Ubuntu 26.04 + GCC + CMake + Qt6 + noVNC 컨테이너 | [docker/README.md](docker/README.md) |
| [documents/](documents/) | 문서 목차 (Pi 세팅 · ROS 설치 · OpenCR · Isaac · 시행착오) | [documents/README.md](documents/README.md) |
| [ros2_ws/](ros2_ws/) | ROS 2 워크스페이스 (bringup · RL 보정 · 외부 소스 `deps.repos`) | [ros2_ws/README.md](ros2_ws/README.md) · [ARCH.md](ros2_ws/ARCH.md) |
| [tools/](tools/) | 운영 CLI `run-cli` · RViz `run-rviz` (베이스·SLAM·Nav·텔레오퍼·지도) | [tools/README.md](tools/README.md) · [MAP_NAV.md](tools/MAP_NAV.md) |
| [isaac/](isaac/) | Isaac Lab RL 이동 보정 — sim 보정 · 학습 · ONNX | [isaac/README.md](isaac/README.md) |
| [firmware/](firmware/) | OpenCR 펌웨어 `tb3_wheels` (프로토콜 · 업로드) | [firmware/opencr/README.md](firmware/opencr/README.md) |
| [setup/](setup/) | Pi 빌드 스크립트 (`build.sh`) | [setup/pi/README.md](setup/pi/README.md) |
| [todo/](todo/) | 날짜별 할 일 · 테스트 체크리스트 | [todo/README.md](todo/README.md) |

## 빠른 시작

```bash
ssh pak 'cd ~/ros2 && git pull && git submodule update --init srf && ./setup/pi/build.sh'   # Pi: 최신화 + 빌드
ssh pak 'cd ~/ros2 && ./tools/run-cli base start'         # Pi: 베이스 (바퀴·odom·라이다)
./tools/run-cli                                           # PC: 메뉴 (다른 터미널에서 ./tools/run-rviz slam = 지도 화면)
```

공통 규칙: `ROS_DOMAIN_ID=25` (PC·Pi·docker). 코드 갱신은 PC에서 push → Pi에서 `git pull` (Pi는 pull 전용).

## SRF (git submodule)

`srf/`는 별도 저장소 `git@github.com:dooboostore-develop/srf.git`의 `main` 브랜치를 가리키는 git submodule이다.
`srf-default-ws/`(그리고 앞으로 추가할 `srf-*-ws/`)는 `../srf`를 소스로 링크해 빌드한다.
ros2 저장소에는 srf의 **커밋 하나(포인터)** 만 기록된다.

### 처음 받을 때

```bash
git clone --recurse-submodules git@github.com:visualkhh/ros2.git
# 이미 받았다면
git submodule update --init srf
```

`srf/`가 비어 있으면 submodule을 아직 받지 않은 것이다. 위 명령을 실행한다.

### 갱신 받을 때 (pull)

```bash
git pull
git submodule update --init srf     # ros2가 가리키는 srf 커밋으로 맞춘다
```

한 번만 `git config submodule.recurse true`를 해 두면 `git pull`이 submodule까지 같이 맞춘다.

### srf 코드를 고칠 때

```bash
cd srf
git switch main                     # submodule은 보통 detached HEAD로 받아진다
# ... 수정, 빌드, 테스트 ...
git commit -am "..."
git push origin main                # 1) srf를 먼저 push
cd ..
git add srf                         # 2) ros2가 새 srf 커밋을 가리키게
git commit -m "chore: srf 갱신"
git push                            # 3) ros2 push
```

순서가 중요하다. srf를 push하지 않고 ros2만 push하면, 다른 기기는 받을 수 없는 srf 커밋을 가리키게 된다.

### srf 최신 main으로 올릴 때

```bash
git submodule update --remote srf   # .gitmodules의 branch = main 최신으로
git add srf && git commit -m "chore: srf 최신 main" && git push
```

### Pi (pull 전용)

- Pi에 GitHub SSH 키가 없으면 그 Pi에서만 HTTPS로 받게 한다 (srf는 공개 저장소):
  `git config submodule.srf.url https://github.com/dooboostore-develop/srf.git && git submodule update --init srf`
