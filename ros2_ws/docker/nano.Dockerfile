FROM nvcr.io/nvidia/l4t-ros:humble-ros-base

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
