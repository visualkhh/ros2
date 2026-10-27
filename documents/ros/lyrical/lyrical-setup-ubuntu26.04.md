# ROS 2 Lyrical (Luth) 세팅 - Ubuntu 26.04 Resolute 기준

> 원본 참고: `/home/pank/workspaces/pa/source/Lv2_bee_Assignment/docker/Dockerfile`, `docker-compose.yml`
> 원본은 Ubuntu 24.04 Noble + Jetson Orin Nano(JetPack 7) 기준. 이 문서는 그중 **26.04에서 그대로 되는 부분만** 추림.
> 24.04 전용(Isaac CDN, noble pin)은 제외.
> OpenCR는 분리됨: [OpenCR 세팅](../../opencr/opencr-setup-ubuntu26.04.md)
> Docker 설정파일 세트: [Docker README](../../docker/README.md)

- 호스트: Ubuntu 26.04 LTS Resolute Raccoon (amd64 / arm64 Tier 1)
- ROS: Lyrical Luth (LTS, ~2031.05, Python 3.12~3.14 / 26.04 기본 3.13)
- 공식 deb 지원: Ubuntu 26.04 resolute → Isaac 저장소 불필요

## 1. 24.04 Jetson용 vs 26.04 차이표

| 항목 | 24.04 참고파일 (제외/수정) | 26.04 처리 |
|---|---|---|
| `FROM ubuntu:24.04` | Jetson Orin Nano 맞춤 | `FROM ubuntu:26.04`로 변경 |
| NVIDIA Isaac ROS apt 2줄 (`isaac.download.nvidia.com ... noble main`) | noble arm64용 Lyrical 우회 제공 | **삭제**. 26.04는 공식 ROS apt에 Lyrical 있음 |
| `ros2-apt-source_1.3.0.noble_all.deb` | noble 고정 | `resolute`용으로 변경. `$(lsb_release -cs)` 사용 |
| `PYTHONPATH=/opt/ros/lyrical/lib/python3.12/site-packages` | Noble Python 3.12 | 26.04는 `python3.13/site-packages` (3.13 기본). 하드코딩보다 `setup.bash` 의존 권장 |
| `ros-lyrical-ros-base`, nav2, moveit, ros2control, robot-localization 등 | Isaac CDN 경유 설치 | 26.04는 공식 apt에서 그대로 설치 (패키지명 동일) |
| `pick_ik` 소스빌드 (7c97fe2) | C++20, moveit-core 기반 | 26.04에서도 유효. 그대로 사용 |
| `torch CPU + ultralytics + "numpy<2"` pip | faker YOLO용 | 26.04에서도 유효. `--break-system-packages` 유지 |
| `arduino-cli + gcc-arm-none-eabi + OpenCR 1.5.3` | arm64 수동설치 우회, x86_64 i386 툴체인 대체 | 분리 문서로 이동 → [OpenCR 세팅](../../opencr/opencr-setup-ubuntu26.04.md) |
| `shm_size: 4gb`, `network_mode: host`, `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `FASTDDS_DEFAULT_PROFILES_FILE` | 영상 DDS 유실 방지용 | 26.04에서도 유효. 그대로 사용 |
| `volumes: ../lv2_module6/...` | Lv2 과제 경로 하드코딩 | 26.04 문서에서는 일반화 (아래 참조) |

## 2. 네이티브 설치 (26.04 권장)

Docker 없이 바로 까는 방법. Jetson 우회 없이 공식 저장소만 사용.

```bash
# locale + universe
sudo apt update && sudo apt install -y locales software-properties-common curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y

# ROS 2 apt source (resolute - noble 하드코딩 금지)
export ROS_APT_SOURCE_VERSION=1.3.0
curl -fsSL -o /tmp/ros2-apt-source.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(lsb_release -cs)_all.deb
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

# base (Bare Bones - TurtleBot/Pi류는 full 대신 base 권장)
sudo apt install -y ros-lyrical-ros-base
sudo apt install -y ros-dev-tools python3-colcon-common-extensions
```

동작 확인:

```bash
source /opt/ros/lyrical/setup.bash
printenv ROS_DISTRO  # lyrical
ros2 --help
```

### 2-1. 참고 Dockerfile의 apt 패키지 (26.04 그대로 사용 가능)

```bash
sudo apt install -y \
  ros-lyrical-nav2-msgs \
  ros-lyrical-moveit-msgs \
  ros-lyrical-moveit-task-constructor-msgs \
  ros-lyrical-control-msgs \
  ros-lyrical-controller-interface \
  ros-lyrical-hardware-interface \
  ros-lyrical-pluginlib \
  ros-lyrical-std-srvs \
  ros-lyrical-controller-manager \
  ros-lyrical-joint-state-broadcaster \
  ros-lyrical-imu-sensor-broadcaster \
  ros-lyrical-diff-drive-controller \
  ros-lyrical-joint-trajectory-controller \
  ros-lyrical-parallel-gripper-controller \
  ros-lyrical-robot-state-publisher \
  ros-lyrical-xacro \
  ros-lyrical-ros2controlcli \
  ros-lyrical-twist-mux \
  ros-lyrical-diagnostic-updater \
  ros-lyrical-diagnostic-aggregator \
  ros-lyrical-teleop-twist-keyboard \
  ros-lyrical-robot-localization \
  ros-lyrical-rosx-introspection \
  ros-lyrical-cv-bridge \
  libopencv-dev rapidjson-dev socat curl ca-certificates

# Nav2 실체 (bringup 제외 - 의존성 수백개 방지)
sudo apt install -y \
  ros-lyrical-nav2-lifecycle-manager \
  ros-lyrical-nav2-bt-navigator \
  ros-lyrical-nav2-behavior-tree \
  ros-lyrical-nav2-planner \
  ros-lyrical-nav2-navfn-planner \
  ros-lyrical-nav2-controller \
  ros-lyrical-nav2-dwb-controller \
  ros-lyrical-nav2-behaviors \
  ros-lyrical-nav2-costmap-2d \
  ros-lyrical-nav2-velocity-smoother \
  ros-lyrical-nav2-collision-monitor \
  ros-lyrical-nav2-map-server

# MoveIt 실체
sudo apt install -y \
  ros-lyrical-moveit-ros-move-group \
  ros-lyrical-moveit-kinematics \
  ros-lyrical-moveit-planners-ompl \
  ros-lyrical-moveit-simple-controller-manager \
  ros-lyrical-moveit-ros-visualization

# RTAB-Map (26.04 공식 apt 기준 - noble때처럼 메타패키지 없으면 rtabmap 패키지로)
sudo apt install -y ros-lyrical-rtabmap-msgs ros-lyrical-rtabmap
```

### 2-2. pick_ik (소스빌드 - 26.04 유지)

apt에 없어 소스빌드 후 `/opt/ros/lyrical`에 복사. 원본 Dockerfile과 동일.

```bash
sudo apt install -y git ros-lyrical-rsl ros-lyrical-generate-parameter-library \
  ros-lyrical-moveit-core ros-lyrical-tf2-kdl \
  librange-v3-dev libfmt-dev libexpected-dev
mkdir -p /tmp/pick_ik_ws/src && cd /tmp/pick_ik_ws/src
git clone https://github.com/PickNikRobotics/pick_ik.git && cd pick_ik && git checkout 7c97fe2
cd /tmp/pick_ik_ws && source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
# --install-base /opt/ros/lyrical 직접 설치는 setup.bash 덮어쓰므로 금지. 폴더만 복사:
sudo cp -r install/pick_ik/. /opt/ros/lyrical/
rm -rf /tmp/pick_ik_ws
```

### 2-3. YOLO CPU (26.04 유지)

```bash
sudo apt install -y libgl1 libglib2.0-0 python3-pip ros-lyrical-ament-cmake-pytest
pip install --break-system-packages --ignore-installed --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu
pip install --break-system-packages --ignore-installed --no-cache-dir "numpy<2" ultralytics pytest
```

### 2-4. OpenCR은 분리 문서 참조

펌웨어 빌드/업로드 부분은 여기에 두지 않음:

- [OpenCR 세팅 (Ubuntu 26.04, Arduino CLI)](../../opencr/opencr-setup-ubuntu26.04.md)

## 3. Docker로 할 때 (26.04용 변환)

원본 `FROM ubuntu:24.04` + Isaac 2줄을 빼고 아래처럼.

```dockerfile
FROM ubuntu:26.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=lyrical
# PYTHONPATH 하드코딩 금지 (3.12→3.13 깨짐). setup.bash source로 해결.

# 공식 ROS apt만 사용 (Isaac CDN 삭제)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.resolute_all.deb \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-lyrical-ros-base \
    && rm -rf /var/lib/apt/lists/*
# 이후 apt 패키지들은 2-1절과 동일
```

빌드/실행은 원본과 동일:

```bash
cd docker/
docker compose build
docker compose up -d && docker compose exec lyrical bash
```

## 4. docker-compose.yml (26.04에서 그대로 쓰는 부분)

원본 Lv2 경로(`../lv2_module6/...`)를 자신의 워크스페이스로만 바꾸면 됨. 핵심 3가지는 26.04에서도 필수.

```yaml
services:
  lyrical:
    build:
      context: ..
      dockerfile: docker/Dockerfile
    image: lv2-module6:testbed
    working_dir: /ws
    shm_size: 4gb  # Fast DDS 640x480 영상 유실 방지 (기본 64MB면 구독자 많을 때 70초 이상 안 옴)
    environment:
      - ROS_DOMAIN_ID=25  # 같은 LAN 다른 팀 DDS와 분리
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET  # LOCALHOST면 노드 40개 중 일부 discovery 실패
      - FASTDDS_DEFAULT_PROFILES_FILE=/ws/fastdds.xml  # TypeLookup OFF 등
    stdin_open: true
    tty: true
    network_mode: host
    command: bash
    volumes:
      # 예시 - 자신의 경로로 변경
      - ../ros2_ws/src:/ws/src
      - ../ros2_ws/fastdds.xml:/ws/fastdds.xml
```

왜 유지하는지 (원본 주석 요약):
- `shm_size: 4gb`: fastdds.xml이 노드마다 32MB 구역 요구. 64MB 기본값이면 큰 영상 토픽 조각나서 유실.
- `ROS_DOMAIN_ID=25`: 기본 0이면 `/robot_description` 같은 토픽을 남의 로봇이 가로챔.
- `SUBNET`: LOCALHOST로 두면 Nav2 멈춤, `/map` 발행자 0 등 실측 장애.

## 5. 26.04에서 버릴 것 (24.04 Jetson 전용)

1. `https://isaac.download.nvidia.com/isaac-ros/... noble main` 2줄 — 26.04 resolute와 codename 불일치로 apt 깨짐. 삭제.
2. `ros2-apt-source_*.noble_all.deb` 하드코딩 — `resolute_all.deb` 또는 `$(lsb_release -cs)`로 변경.
3. JetPack 7 / Orin Nano 가정 — 26.04 x86_64/라즈베리Pi arm64는 해당 없음.
4. `PYTHONPATH=...python3.12` 고정 — 26.04는 3.13이므로 `setup.bash` source 방식으로 대체. Dockerfile ENV 고정 + entrypoint 우회용이라는 원래 의도가 있으면 `python3.13`으로 갱신.
5. `../lv2_module6/...` 볼륨 경로 — 과제 레포 구조 그대로 가져오지 말고 자신의 `ros2_ws` 경로로 변경. `build/install/log`는 컨테이너 안에 두고 `/debug/install` 복사는 필요시에만 (`build-ws.sh` 로직).

## 6. 검증

```bash
# 네이티브
source /opt/ros/lyrical/setup.bash
ros2 doctor --report | head -n 50

# Docker
docker compose up -d
docker compose exec lyrical bash -c 'source /opt/ros/lyrical/setup.bash && ros2 doctor --report | head -n 20'
```

- `ros-lyrical-ros-base` 설치 후 `ros2 pkg list | grep lyrical` 로 패키지 확인
- DDS 분리 확인: `echo $ROS_DOMAIN_ID` (25), `echo $FASTDDS_DEFAULT_PROFILES_FILE`
- OpenCR: [OpenCR 문서](../../opencr/opencr-setup-ubuntu26.04.md) 참조 (`arduino-cli core list`, `ls -l /dev/opencr`)

---
출처: Lv2_bee_Assignment/docker/Dockerfile (24.04/Isaac 기준) → 26.04 Resolute + Lyrical 공식 지원 기준으로 필터링. Lyrical은 26.04 Tier 1, 24.04 Tier 3.
