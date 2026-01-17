#!/bin/bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $(pwd)/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc 
