FROM ubuntu:26.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    g++ \
    gcc \
    gdb \
    git \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libglvnd-dev \
    libx11-dev \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    ninja-build \
    novnc \
    openbox \
    pkg-config \
    qt6-base-dev \
    qt6-base-dev-tools \
    qt6-qpa-plugins \
    scrot \
    websockify \
    x11-utils \
    x11vnc \
    xauth \
    xvfb \
 && rm -rf /var/lib/apt/lists/*

# 컨테이너 안에서는 GPU/디스플레이가 없어도 Qt가 도는 offscreen을 기본으로 쓴다.
# 실제 창이 필요하면 docker run -e QT_QPA_PLATFORM=xcb ... + X 포워딩을 쓴다.
ENV QT_QPA_PLATFORM=offscreen

WORKDIR /workspace/srf

CMD ["/bin/bash"]
