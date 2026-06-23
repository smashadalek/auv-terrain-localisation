FROM osrf/ros:jazzy-desktop

RUN apt update && apt install -y \
    git \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /workspace

CMD ["bash"]
