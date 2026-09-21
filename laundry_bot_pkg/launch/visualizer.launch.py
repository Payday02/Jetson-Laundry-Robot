"""
visualizer.launch.py — RViz2 visualizer: arm TF + camera TF + workspace env.

Standalone (use alongside bringup_virtual or real bringup):
    ros2 launch laundry_bot visualizer.launch.py

Or all-in-one with the virtual pipeline (recommended first run):
    ros2 launch laundry_bot bringup_virtual.launch.py with_visualizer:=true

What it starts:
  * robot_state_publisher  (URDF from share/laundry_bot/urdf)
  * visualizer_node         (front_cam static TF, wrist_cam FK TF, env cloud)
  * rviz2 with share/laundry_bot/rviz/laundry.rviz

Notes:
  * The URDF is a *visual placeholder* sized from the UR7e DH table. When your
    arm's real dimensions are in, update urdf/laundry_arm.urdf.xacro to match
    config/ur7e_like.yaml (they must agree, or the model will not track the
    arm).
  * Set your real camera drivers' frame_id to front_cam / wrist_cam so their
    clouds/images appear in the right place (see visualizer_node.py header).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('laundry_bot')
    cfg = os.path.join(pkg_share, 'config')
    # robot_state_publisher wants plain URDF XML — run xacro at launch time
    urdf = Command([os.path.join(pkg_share, 'urdf', 'laundry_arm.urdf.xacro')])
    rviz_config = os.path.join(pkg_share, 'rviz', 'laundry.rviz')

    use_wrist = LaunchConfiguration('use_wrist')

    return LaunchDescription([
        DeclareLaunchArgument('use_wrist', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf,
                         'use_sim_time': False}],
        ),
        Node(
            package='laundry_bot',
            executable='visualizer_node',
            name='visualizer_node',
            parameters=[{'kin_config': os.path.join(cfg, 'ur7e_like.yaml'),
                         'enable_wrist': use_wrist}],
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
