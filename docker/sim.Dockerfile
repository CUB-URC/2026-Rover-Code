# ros 2 humble + gazebo harmonic
# ros-humble-ros-gz* (fortress) conflicts with ros-humble-ros-gzharmonic
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y curl lsb-release gnupg \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-yaml \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
