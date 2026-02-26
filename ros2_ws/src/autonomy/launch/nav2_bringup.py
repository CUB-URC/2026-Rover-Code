import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_autonomy = get_package_share_directory("autonomy")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    nav2_params_file = os.path.join(
        pkg_autonomy, "config", "nav2_params.yaml"
    )  # general nav2 params
    rl_params_file = os.path.join(
        pkg_autonomy, "config", "nav2_localization.yaml"
    )  # reinforcement learning parameters
    rviz_config_file = os.path.join(
        pkg_autonomy, "config", "nav2_default_view.rviz"
    )  # rviz config file

    use_gps = LaunchConfiguration("use_gps")
    use_rviz = LaunchConfiguration("use_rviz")

    """ map -> odom tf broadcaster for static map 
    
    * if gps:=false => nav2 will use local frame from rover's starting position (for demos without gps)
    * if gps:=true => ekf_filter_node_map publishes this TF (for demos with gps)
    
    """
    static_map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=UnlessCondition(use_gps),
    )

    """ bridge zed_camera_link -> base_link
    
    * ZED wrapper pubs odom -> zed_camera_link
    * nav2 needs odom -> base_link
    * this TF bridges the two
    
    TODO: measure actual camera mounting offset (x forward, z up from base_link) OR use URDF if/when that is ready
    
    [ x, y, z, roll, pitch, yaw, parent_frame, child_frame ]
    
    """
    static_camera_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_camera_base_tf",
        arguments=["0.2", "0", "0.3", "0", "0", "0", "base_link", "zed_camera_link"],
    )

    """ local EKF (odom -> base_link, ZED VIO only)
    
    * if gps:=true => fuse ZED /odom vels + IMU
    * must disable ZED TF publishing (set `publish_tf: false` in ZED params)
    
    """
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[rl_params_file],
        remappings=[("odometry/filtered", "/odometry/local")],
        condition=IfCondition(use_gps),
    )

    """ global EKF (map -> odom, GPS + ZED VIO)"""

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[rl_params_file],
        remappings=[("odometry/filtered", "/odometry/global")],
        condition=IfCondition(use_gps),
    )

    """ navsat_transform (GPS [WGS84] -> map [cartesion odom]) """
    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[rl_params_file],
        remappings=[
            ("imu/data", "/zed/zed_node/imu/data"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "/odometry/global"),
            ("odometry/gps", "/odometry/gps"),
        ],
        condition=IfCondition(use_gps),
    )

    """ nav2 stack delegation to navigation_launch.py """
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": "false",
            "autostart": "true",
            "use_composition": "false",
        }.items(),
    )

    """ RViz """
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gps",
                default_value="false",
                description="GPS vs No GPS for localization",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="false", description="Launch RViz or not"
            ),
            static_map_odom_tf,
            static_camera_base_tf,
            ekf_local,
            ekf_global,
            navsat_transform,
            nav2_launch,
            rviz,
        ]
    )
