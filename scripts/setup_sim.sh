#!/bin/bash
# installs gazebo harmonic and ros-humble-ros-gzharmonic
# removes ros-humble-ros-gz* (fortress) first since they conflict

set -e

echo "=== CUB-URC Simulation Setup (Gazebo Harmonic) ==="

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && cd .. && pwd)"

if ! grep -q "22.04" /etc/os-release; then
    echo "Error: expected Ubuntu 22.04 (ROS 2 Humble). Use the Docker sim service instead."
    exit 1
fi
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS2 Humble not found at /opt/ros/humble/setup.bash"
    echo "Please install ROS2 Humble first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo ""
echo "==> Removing Gazebo Fortress ROS packages, if any (they conflict with Harmonic's)"
FORTRESS_PKGS=$(dpkg-query -W -f='${Package}\n' 'ros-humble-ros-gz*' 'ros-humble-ros-ign*' 2>/dev/null \
    | grep -v gzharmonic || true)
if [ -n "$FORTRESS_PKGS" ]; then
    echo "$FORTRESS_PKGS"
    sudo apt-get remove -y $FORTRESS_PKGS
else
    echo "none installed"
fi

echo ""
echo "==> Adding the OSRF Gazebo apt repository"
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

echo ""
echo "==> Installing Gazebo Harmonic + ROS bridge (large download)"
sudo apt-get update
sudo apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-teleop-twist-keyboard \
    python3-numpy python3-yaml python3-matplotlib

# ~/rover_ws has its own rover_description
if grep -q "rover_ws/install/setup.bash" ~/.bashrc 2>/dev/null; then
    echo ""
    echo "==> Removing the old ~/rover_ws line from ~/.bashrc (backup: ~/.bashrc.bak-sim)"
    cp ~/.bashrc ~/.bashrc.bak-sim
    sed -i '/rover_ws\/install\/setup.bash/d' ~/.bashrc
fi

echo ""
echo "==> Building the simulation packages"
cd "$REPO_ROOT/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to simulation

echo ""
echo "✓ Simulation setup complete. Open a NEW terminal, then:"
echo "    source $REPO_ROOT/ros2_ws/install/setup.bash"
echo "    ros2 launch simulation sim.launch.py"
echo "Drive it (second terminal):"
echo "    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/drive/cmd_vel"
