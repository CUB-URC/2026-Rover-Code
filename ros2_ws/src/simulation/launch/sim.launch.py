"""Gazebo Harmonic simulation of the rover, bridged to ROS 2.

    ros2 launch simulation sim.launch.py
    ros2 launch simulation sim.launch.py world:=test_terrain rviz:=true
    ros2 launch simulation sim.launch.py headless:=true          # no Gazebo window
    ros2 launch simulation sim.launch.py publish_odom_tf:=false  # when an EKF owns odom->base_link

Drive it the same way as the real rover (it listens on /drive/cmd_vel):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/drive/cmd_vel
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLDS = ("rover_course", "test_terrain", "empty")


def launch_setup(context):
    sim = get_package_share_directory("simulation")
    desc = get_package_share_directory("rover_description")
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    world_name = arg("world")
    if world_name not in WORLDS:
        raise RuntimeError("world:=%s is not one of %s" % (world_name, ", ".join(WORLDS)))
    world = os.path.join(sim, "worlds", world_name + ".sdf")
    robot_description = xacro.process_file(
        os.path.join(sim, "urdf", "rover_sim.urdf.xacro")).toxml()
    with open(os.path.join(desc, "config", "generated.yaml")) as f:
        spawn_z = arg("z") or str(yaml.safe_load(f)["spawn_z"])

    actions = []
    for share in {os.path.dirname(sim), os.path.dirname(desc)}:
        actions.append(AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", share))

    plugin_dir = os.path.join(get_package_prefix("simulation"), "lib")
    if os.path.exists(os.path.join(plugin_dir, "libRockerDifferential.so")):
        actions.append(AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_dir))
    else:
        actions.append(LogInfo(msg="[sim.launch] RockerDifferential plugin was not built (Gazebo "
                                   "Harmonic dev packages missing at build time). The rockers will "
                                   "only be held by their backup spring."))

    gz_args = "-r -v 3 " + ("-s " if arg("headless") == "true" else "") + world
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "rover",
                        "-x", arg("x"), "-y", arg("y"), "-z", spawn_z, "-Y", arg("yaw")]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="sim_bridge",
             output="screen",
             parameters=[{"config_file": os.path.join(sim, "config", "bridge.yaml"),
                          "use_sim_time": True}]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="sim_odom_tf_bridge",
             condition=IfCondition(arg("publish_odom_tf")),
             arguments=["/model/rover/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
             remappings=[("/model/rover/tf", "/tf")],
             parameters=[{"use_sim_time": True}]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(arg("rviz")),
             arguments=["-d", os.path.join(sim, "rviz", "sim.rviz")],
             parameters=[{"use_sim_time": True}]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="rover_course",
                              description="One of: " + ", ".join(WORLDS)),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run the Gazebo server only, no GUI window"),
        DeclareLaunchArgument("publish_odom_tf", default_value="true",
                              description="Bridge the DiffDrive odom->base_link TF to /tf"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="",
                              description="Spawn height; empty = spawn_z from rover_description"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])
