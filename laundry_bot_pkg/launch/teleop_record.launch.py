"""
teleop_record.launch.py — teleop + recording bring-up.

Starts:
  * arm_driver_node      (CAN -> arm)
  * teleop_quest_node    (Quest 2 -> IK -> /joint_target)
  * lerobot_recorder_node (captures demos in LeRobot format)

The OAK-D camera is started by its own launch (depthai / Isaac ROS), and the
Quest2ROS2 bridge runs on the Quest/PC — both are external to this file.

Usage:
  ros2 launch laundry_bot teleop_record.launch.py
  ros2 launch laundry_bot teleop_record.launch.py use_wrist:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('laundry_bot')
    cfg = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument('use_wrist', default_value='false'),
        DeclareLaunchArgument('dataset_root', default_value=''),
        DeclareLaunchArgument('auto_home', default_value='false'),

        Node(
            package='laundry_bot',
            executable='arm_driver_node',
            name='arm_driver_node',
            parameters=[os.path.join(cfg, 'arm.yaml'),
                        {'auto_home_on_start': LaunchConfiguration('auto_home')}],
        ),
        Node(
            package='laundry_bot',
            executable='teleop_quest_node',
            name='teleop_quest_node',
            parameters=[os.path.join(cfg, 'teleop.yaml')],
        ),
        Node(
            package='laundry_bot',
            executable='lerobot_recorder_node',
            name='lerobot_recorder_node',
            parameters=[os.path.join(cfg, 'record.yaml'),
                        {'use_wrist': LaunchConfiguration('use_wrist'),
                         'root': LaunchConfiguration('dataset_root')}],
        ),
    ])
