FROM nvcr.io/nvidia/l4t-ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# ZED SDK should be installed on the host
# Container accesses it via bind-mount
WORKDIR /workspace
