#!/bin/bash

set -e

echo "=== CUB-URC Rover Setup ==="

# Get the repository root directory
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && cd .. && pwd)"
ROS2_WS="$REPO_ROOT/ros2_ws"

# Check if ROS2 Humble is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS2 Humble not found at /opt/ros/humble/setup.bash"
    echo "Please install ROS2 Humble first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✓ ROS2 Humble found"

# Build workspace if install doesn't exist
if [ ! -d "$ROS2_WS/install" ]; then
    echo ""
    echo "Building ROS2 workspace (first time setup)..."
    cd "$ROS2_WS"
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    echo "✓ Build complete"
else
    echo "✓ ROS2 workspace already built"
fi

# Create a sourcing function to avoid embedding long paths
echo ""
echo "Adding sourcing function to ~/.bashrc..."

# Remove old setup lines if they exist
sed -i '/# CUB-URC Rover Setup/d; /source \/opt\/ros\/humble\/setup.bash/d; /source.*ros2_ws\/install\/setup.bash/d; /export ROS_DOMAIN_ID=1/d' ~/.bashrc 2>/dev/null || true

# Add new setup function
cat >> ~/.bashrc << 'EOF'

# CUB-URC Rover Setup
setup_cub_urc() {
    source /opt/ros/humble/setup.bash
    local repo_root="$(cd "${CUB_URC_REPO:-.}" 2>/dev/null && pwd)" || pwd
    source "$repo_root/ros2_ws/install/setup.bash"
    export ROS_DOMAIN_ID=1
    export CUB_URC_REPO="$repo_root"
    echo "CUB-URC environment loaded (Domain ID: 1)"
}

# Auto-source on shell startup
setup_cub_urc
EOF

echo "✓ Setup complete!"
echo ""
echo "Next time you open a new terminal, the environment will be automatically loaded."
echo "Or run 'setup_cub_urc' to reload it in the current terminal."
echo ""
echo "To build changes: cd $ROS2_WS && colcon build --symlink-install"
echo "To run simulation: ros2 run drive drive_controller_sim" 
