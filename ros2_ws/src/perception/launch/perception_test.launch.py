"""
perception_test.launch.py

use (webcam):
  ros2 launch perception perception_test.launch.py \
    model_path:=/abs/path/to/model.onnx \
    use_webcam:=true   # uses usb_cam, publishes /image_raw + /camera_info

use (bag / ZED publishing /image_raw):
  ros2 launch perception perception_test.launch.py \
    model_path:=/abs/path/to/model.onnx \
    use_webcam:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_perception = get_package_share_directory("perception")

    default_aruco_config = os.path.join(pkg_perception, "config", "aruco_config.yaml")

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="abs path to .onnx/.pt YOLO model file",
    )

    aruco_config_arg = DeclareLaunchArgument(
        "aruco_config",
        default_value=default_aruco_config,
        description="abs path to aruco_config.yaml",
    )

    confidence_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.5",
        description="YOLO confidence threshold (0.0 - 1.0)",
    )

    use_webcam_arg = DeclareLaunchArgument(
        "use_webcam",
        default_value="true",
        description="launch a usb_cam node for a USB webcam",
    )

    camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video0",
        description="webcam device path (only used when use_webcam:=true)",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="RViz for visual debugging",
    )

    # usb_cam (install: sudo apt install ros-$ROS_DISTRO-usb-cam)
    # publishes: /image_raw  and  /camera_info
    webcam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="webcam",
        parameters=[
            {
                "video_device": LaunchConfiguration("camera_device"),
                "framerate": 30.0,
                "image_width": 640,
                "image_height": 480,
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_webcam")),
    )

    aruco_node = Node(
        package="perception",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=[
            {
                "config_path": LaunchConfiguration("aruco_config"),
                # camera intrinsics come from /camera/camera_info automatically
            }
        ],
        remappings=[
            ("/image", "/image_raw"),
            ("/zed/zed_node/rgb/color/rect/camera_info", "/camera_info"),
        ],
    )

    yolo_node = Node(
        package="perception",
        executable="yolo_node",
        name="yolo_node",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            }
        ],
        remappings=[
            ("/image", "/image_raw"),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            model_path_arg,
            aruco_config_arg,
            confidence_arg,
            use_webcam_arg,
            camera_device_arg,
            use_rviz_arg,
            webcam_node,
            aruco_node,
            yolo_node,
            rviz_node,
        ]
    )
