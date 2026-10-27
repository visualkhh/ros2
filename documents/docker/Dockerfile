# ROS 2 Lyrical — Ubuntu 26.04 Resolute 테스트베드 (26.04 전용)
# 원본: Lv2_bee_Assignment/docker/Dockerfile (24.04 Noble + Jetson Orin Nano + Isaac CDN)
# 26.04 변경점: FROM 26.04, Isaac 2줄 삭제, ros-apt-source resolute, PYTHONPATH python3.13
# 상세 비교: ../ros/lyrical/lyrical-setup-ubuntu26.04.md, ../opencr/opencr-setup-ubuntu26.04.md
FROM ubuntu:26.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=lyrical \
    AMENT_PREFIX_PATH=/opt/ros/lyrical \
    CMAKE_PREFIX_PATH=/opt/ros/lyrical \
    PYTHONPATH=/opt/ros/lyrical/lib/python3.13/site-packages
# NOTE: 24.04 원본은 python3.12 고정. 26.04 기본 Python 3.13이므로 3.13으로 갱신.
# setup.bash는 셸에서 source해야 적용되지만, IDE Docker 툴체인처럼
# entrypoint를 우회하는 실행에서도 find_package가 동작하도록 ENV로 고정.

# ROS 2 Lyrical 공식 apt 저장소 — Ubuntu 26.04(resolute) Tier 1
# 24.04 원본의 NVIDIA Isaac ROS 2줄(isaac.download.nvidia.com ... noble)은 26.04에서 삭제.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.resolute_all.deb \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-lyrical-ros-base \
    && rm -rf /var/lib/apt/lists/*

# 노드 빌드에 필요한 메시지·인터페이스 + 도커에서 실제로 돌리는 것 (ros2_control · robot_state_publisher · EKF)
# + faker 파일 연동(rosx_introspection · OpenCV).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-dev-tools \
    python3-colcon-common-extensions \
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
    libopencv-dev \
    rapidjson-dev \
    socat \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# 진짜 Nav2 — bringup 없이 서버 직접 기동용 (nav2_bringup은 시뮬 의존성 수백개라 제외)
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    ros-lyrical-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*

# 진짜 move_group
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-moveit-ros-move-group \
    ros-lyrical-moveit-kinematics \
    ros-lyrical-moveit-planners-ompl \
    ros-lyrical-moveit-simple-controller-manager \
    ros-lyrical-moveit-ros-visualization \
    && rm -rf /var/lib/apt/lists/*

# pick_ik — MoveIt IK 플러그인. Lyrical apt에 없어 소스로 빌드해 /opt/ros/lyrical에 설치.
#   커밋 고정 (재현성): PickNikRobotics/pick_ik 7c97fe2 (2026-09-18)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ros-lyrical-rsl ros-lyrical-generate-parameter-library ros-lyrical-moveit-core ros-lyrical-tf2-kdl \
    librange-v3-dev libfmt-dev libexpected-dev \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /tmp/pick_ik_ws/src && cd /tmp/pick_ik_ws/src \
    && git clone https://github.com/PickNikRobotics/pick_ik.git && cd pick_ik && git checkout 7c97fe2 \
    && cd /tmp/pick_ik_ws && . /opt/ros/lyrical/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cp -r install/pick_ik/. /opt/ros/lyrical/ \
    && rm -rf /tmp/pick_ik_ws
# (colcon --install-base /opt/ros/lyrical 는 ROS의 setup.bash를 덮어써서 안 씀 — 패키지 폴더만 복사: lib · share · ament_index)

# RTAB-Map
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-rtabmap-msgs \
    ros-lyrical-rtabmap \
    && rm -rf /var/lib/apt/lists/*

# YOLO perception (CPU 추론용) — torch CPU + ultralytics. libGL은 cv2용.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 python3-pip ros-lyrical-ament-cmake-pytest \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --break-system-packages --ignore-installed --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu \
    && pip install --break-system-packages --ignore-installed --no-cache-dir "numpy<2" ultralytics pytest

# Arduino CLI (OpenCR 펌웨어 빌드·업로드용)
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | BINDIR=/usr/local/bin sh

# OpenCR 보드 코어 + Dynamixel2Arduino 라이브러리
# 상세 분기(x86_64/i386 대체, arm64 수동설치)는 opencr 문서 참조:
# documents/opencr/opencr-setup-ubuntu26.04.md
ENV ARDUINO_UPDATER_ENABLE_NOTIFICATION=false \
    OPENCR_VERSION=1.5.3 \
    OPENCR_SHA256=418656e5e6d99d45d187ffdb28dece0f450c6707da3f6db56769f3ecafdc413c
RUN arduino-cli config init --overwrite \
    && arduino-cli config add board_manager.additional_urls \
        https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json \
    && arduino-cli core update-index \
    && apt-get update && apt-get install -y --no-install-recommends gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    && rm -rf /var/lib/apt/lists/* \
    && if [ "$(uname -m)" = "x86_64" ]; then \
           arduino-cli core install OpenCR:OpenCR \
           && rm -rf /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin \
           && mkdir -p /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin \
           && for f in /usr/bin/arm-none-eabi-*; do ln -s "$f" /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/; done; \
       else \
           curl -fsSL "https://github.com/ROBOTIS-GIT/OpenCR/releases/download/${OPENCR_VERSION}/opencr.tar.bz2" \
               -o /tmp/opencr.tar.bz2 \
           && echo "${OPENCR_SHA256}  /tmp/opencr.tar.bz2" | sha256sum -c - \
           && mkdir -p /root/.arduino15/packages/OpenCR/hardware/OpenCR/${OPENCR_VERSION} \
               /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin \
           && tar xzf /tmp/opencr.tar.bz2 -C /tmp \
           && cp -a /tmp/opencr/. /root/.arduino15/packages/OpenCR/hardware/OpenCR/${OPENCR_VERSION}/ \
           && rm -rf /tmp/opencr /tmp/opencr.tar.bz2 \
           && for f in /usr/bin/arm-none-eabi-*; do ln -s "$f" /root/.arduino15/packages/OpenCR/tools/opencr_gcc/5.4.0-2016q2/bin/; done; \
       fi \
    && arduino-cli lib install Dynamixel2Arduino

WORKDIR /ws

COPY entrypoint.sh /entrypoint.sh
COPY build-ws.sh /usr/local/bin/build-ws
COPY test-serial.sh /usr/local/bin/test-serial
COPY test-firmware.sh /usr/local/bin/test-firmware
COPY test-packages.sh /usr/local/bin/test-packages
COPY test-hardware.sh /usr/local/bin/test-hardware
COPY test-bringup.sh /usr/local/bin/test-bringup
COPY test-faker.sh /usr/local/bin/test-faker
COPY test-all.sh /usr/local/bin/test-all
COPY ros-kill.sh /usr/local/bin/ros-kill
COPY ros-restart.sh /usr/local/bin/ros-restart
COPY ros-faker-restart.sh /usr/local/bin/ros-faker-restart
RUN chmod +x /entrypoint.sh /usr/local/bin/build-ws /usr/local/bin/test-* /usr/local/bin/ros-* \
    && echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
