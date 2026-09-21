#!/usr/bin/env python3
"""
fake_teleop_node.py — scripted controller-pose trajectory for hardware-free
testing of the full teleop -> IK -> driver -> recorder pipeline.

Publishes exactly what the Quest2ROS2 bridge would publish:
    /q2r_right_hand_pose   (geometry_msgs/PoseStamped, 30 Hz)
    teleop/latch           (std_msgs/Bool)

The trajectory is defined in the ROBOT BASE frame (the default below mimics
one full laundry micro-cycle: home -> reach into basket -> lift item ->
carry to table -> flatten in a small circle -> return home) and is
converted to the Quest world frame using the inverse of teleop's pos_map,
so it drives the arm identically to a real controller.

Pure logic (player, waypoints, frame conversion) lives in laundry_bot/trajectory.py
and is self-testable:  python3 -m laundry_bot.trajectory

Usage:
    ros2 run laundry_bot fake_teleop_node \
        --ros-args -p kin_config:=$(find laundry_bot)/config/ur7e_like.yaml \
                   -p waypoints_file:=$(find laundry_bot)/config/fake_waypoints.yaml
"""
from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from laundry_bot.trajectory import (
    TrajectoryPlayer, default_waypoints, base_to_quest, quat_yaw)


class FakeTeleopNode(Node):
    def __init__(self):
        super().__init__('fake_teleop_node')

        self.declare_parameter('kin_config', '')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('pose_topic', '/q2r_right_hand_pose')
        self.declare_parameter('latch_topic', 'teleop/latch')
        self.declare_parameter('pos_map', [
            [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        self.declare_parameter('start_latched', True)

        kin_path = self.get_parameter('kin_config').value
        self._ws_min = None
        self._ws_max = None
        if kin_path:
            from laundry_bot import kinematics as K
            kin = K.load_kin_config(kin_path)
            ws = kin.get('workspace')
            if ws:
                self._ws_min = np.array(ws['min'])
                self._ws_max = np.array(ws['max'])

        self._pos_map = np.array(
            self.get_parameter('pos_map').value, dtype=float).reshape(3, 3)
        if np.linalg.norm(self._pos_map.T @ self._pos_map - np.eye(3)) > 1e-6:
            raise RuntimeError("pos_map is not a rotation matrix")

        rate = float(self.get_parameter('rate').value)
        wps_file = self.get_parameter('waypoints_file').value
        if wps_file:
            import yaml
            with open(wps_file) as f:
                wps = yaml.safe_load(f)['waypoints']
        else:
            wps = default_waypoints()
        self._player = TrajectoryPlayer(wps, rate)

        self._pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('pose_topic').value, 10)
        self._latch_pub = self.create_publisher(
            Bool, self.get_parameter('latch_topic').value, 10)
        self._timer = self.create_timer(1.0 / rate, self._tick)

        if self.get_parameter('start_latched').value:
            self._latch_pub.publish(Bool(data=True))
        self.get_logger().info(
            f"Fake teleop: {len(wps)} waypoints @ {rate} Hz, "
            f"looping. pos_map verified.")

    def _tick(self):
        p_base, yaw = self._player.step()
        if self._ws_min is not None:
            p_base = np.clip(p_base, self._ws_min, self._ws_max)
        p_quest = base_to_quest(self._pos_map, p_base)
        q = quat_yaw(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'q2r_world'
        msg.pose.position.x = float(p_quest[0])
        msg.pose.position.y = float(p_quest[1])
        msg.pose.position.z = float(p_quest[2])
        msg.pose.orientation.w = float(q[0])
        msg.pose.orientation.x = float(q[1])
        msg.pose.orientation.y = float(q[2])
        msg.pose.orientation.z = float(q[3])
        self._pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
