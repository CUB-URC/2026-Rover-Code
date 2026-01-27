import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
       get_package_share_directory('arm'),
       'config',
       'arm_config.yaml'
   )


   return LaunchDescription([
       Node(
           package='arm',
           executable='arm_controller_node',
           name='arm_controller',
           output='screen',
           emulate_tty=True,
           parameters=[config]
       )
   ])