"""
bringup_virtual.launch.py — full pipeline, zero hardware.

Starts:
  * arm_driver_node    (can_interface=virtual: python-can loopback, no bus)
  * fake_teleop_node   (scripted controller-pose trajectory)
  * fake_camera_node   (synthetic front/wrist images)
  * lerobot_recorder_node  (records the whole thing into LeRobot v3.0 format)

Purpose: prove the entire chain — teleop mapping, IK, driver pulse math,
recorder, dataset layout — on the Jetson (or any desktop) before a single
motor, camera, or headset is involved.

Usage:
    ros2 launch laundry_bot bringup_virtual.launch.py
    ros2 launch laundry_bot bringup_virtual.launch.py dataset_root:=/tmp/ds
    # then, in another terminal:
    ros2 run laundry_bot record_cli start "pick item from basket"
    ...watch it loop...
    ros2 run laundry_bot record_cli stop

Add the RViz2 visualizer (arm model + TF + camera frames + workspace):
    ros2 launch laundry_bot bringup_virtual.launch.py with_visualizer:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('laundry_bot')
    cfg = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument('dataset_root', default_value=''),
        DeclareLaunchArgument('use_wrist', default_value='true'),
        DeclareLaunchArgument('waypoints', default_value=''),
        DeclareLaunchArgument('with_visualizer', default_value='false'),

        # ── Driver on the virtual CAN bus (no hardware) ────────────────
        Node(
            package='laundry_bot',
            executable='arm_driver_node',
            name='arm_driver_node',
            parameters=[os.path.join(cfg, 'arm.yaml'),
                        {'can_interface': 'virtual', 'can_channel': 0,
                         'auto_enable_on_start': True}],
        ),
        # ── Scripted "operator" ────────────────────────────────────────
        Node(
            package='laundry_bot',
            executable='fake_teleop_node',
            name='fake_teleop_node',
            parameters=[os.path.join(cfg, 'teleop.yaml'),
                        {'kin_config': os.path.join(cfg, 'ur7e_like.yaml'),
                         'waypoints_file': LaunchConfiguration('waypoints')}],
        ),
        # ── Synthetic cameras ──────────────────────────────────────────
        Node(
            package='laundry_bot',
            executable='fake_camera_node',
            name='fake_camera_node',
            parameters=[{'enable_wrist': LaunchConfiguration('use_wrist')}],
        ),
        # ── Recorder ───────────────────────────────────────────────────
        Node(
            package='laundry_bot',
            executable='lerobot_recorder_node',
            name='lerobot_recorder_node',
            parameters=[os.path.join(cfg, 'record.yaml'),
                        {'use_wrist': LaunchConfiguration('use_wrist'),
                         'root': LaunchConfiguration('dataset_root')}],
        ),

        # ── Optional visualizer (arm TF + env) ────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'visualizer.launch.py')),
            launch_arguments={'use_wrist': LaunchConfiguration('use_wrist'),
                               'rviz': LaunchConfiguration('rviz')}.items(),
            condition=IfCondition(LaunchConfiguration('with_visualizer')),
        ),
    ])
