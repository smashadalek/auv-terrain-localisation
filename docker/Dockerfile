FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y \
    git \
    nano \
    curl \
    wget \
    gnupg \
    lsb-release \
    pkg-config \
    build-essential \
    cmake \
    cppcheck \
    protobuf-compiler \
    libeigen3-dev \
    libgles2-mesa-dev \
    cppzmq-dev \
    python3-pip \
    python3-venv \
    python3-dbg \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    qtbase5-dev \
    ruby \
    software-properties-common \
    ros-jazzy-rviz2 \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /workspace

CMD ["bash"]
