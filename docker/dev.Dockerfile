FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    build-essential \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

# Python dependencies — numpy pinned <2 (cv_bridge compiled against 1.x C API)
COPY ros2_ws/src/perception/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

WORKDIR /workspace
