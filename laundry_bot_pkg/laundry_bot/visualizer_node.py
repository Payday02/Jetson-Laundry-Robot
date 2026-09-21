#!/usr/bin/env python3
"""
visualizer_node.py — TF + environment publishing for the RViz visualizer.

Publishes:
  * static TF:  base_link -> front_cam   (fixed elevated camera mount)
  * dynamic TF: tcp       -> wrist_cam   (FK-driven, follows the arm)
  * PointCloud2: /laundry_bot/env_cloud  (synthetic table + baskets + racks,
    so RViz shows the workspace context even with fake cameras)

Run it alongside robot_state_publisher (see launch/visualizer.launch.py) and
view with:
    rviz2 -d $(ros2 pkg prefix laundry_bot)/share/laundry_bot/rviz/laundry.rviz

Camera frame ids: set your real camera drivers to use frame_id=front_cam /
wrist_cam so OAK-D point clouds and images land in the right place in RViz
(depthai_ros2: set the `frame_id` param; usb_cam: `camera_info`/`frame_id`).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .kinematics import fk, load_kin_config


# ── synthetic environment ────────────────────────────────────────────────
# Default workspace layout (m, base frame) — matches the teleop workspace box
# in config/ur7e_like.yaml. Override via the `env_boxes` parameter:
#   list of [cx, cy, cz, sx, sy, sz]
DEFAULT_ENV_BOXES = [
    # table  (top surface at z≈0.76)
    [0.30,  0.00,  0.74,  0.90,  0.90,  0.04],
    # incoming basket (hamper side, -y)
    [0.55, -0.35,  0.70,  0.30,  0.30,  0.25],
    # outgoing basket (+y)
    [0.55,  0.35,  0.70,  0.30,  0.30,  0.25],
    # incoming hanger rack (-y)
    [0.30, -0.52,  0.98,  0.35,  0.05,  0.15],
    # outgoing hanger rack (+y)
    [0.30,  0.52,  0.98,  0.35,  0.05,  0.15],
]


def boxes_to_points(boxes, spacing=0.04):
    """Sample the 6 faces of each box into (x, y, z, intensity) tuples."""
    pts = []
    for (cx, cy, cz, sx, sy, sz) in boxes:
        x0, x1 = cx - sx / 2, cx + sx / 2
        y0, y1 = cy - sy / 2, cy + sy / 2
        z0, z1 = cz - sz / 2, cz + sz / 2
        xs = np.arange(x0, x1 + 1e-9, spacing)
        ys = np.arange(y0, y1 + 1e-9, spacing)
        zs = np.arange(z0, z1 + 1e-9, spacing)
        for z in (z0, z1):
            for x in xs:
                for y in ys:
                    pts.append((float(x), float(y), float(z), 0.5))
        for x in (x0, x1):
            for y in ys:
                for z in zs:
                    pts.append((float(x), float(y), float(z), 0.6))
        for y in (y0, y1):
            for x in xs:
                for z in zs:
                    pts.append((float(x), float(y), float(z), 0.6))
    return pts


class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter('kin_config', '')
        self.declare_parameter('enable_wrist', True)
        self.declare_parameter('front_cam_pos', [0.05, 0.00, 1.35])
        self.declare_parameter('front_cam_rpy', [0.0, -0.90, 0.0])  # pitch down ~52°
        self.declare_parameter('wrist_cam_pos', [0.02, 0.00, 0.005])  # in tcp frame
        self.declare_parameter('wrist_cam_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('env_boxes', DEFAULT_ENV_BOXES)
        self.declare_parameter('env_topic', '/laundry_bot/env_cloud')
        self.declare_parameter('tf_rate', 50.0)

        kin_path = self.get_parameter('kin_config').value
        self.kcfg = load_kin_config(kin_path) if kin_path else None
        self.dh = self.kcfg['dh'] if self.kcfg else []

        # ── publishers ────────────────────────────────────────────────
        self._sbf = StaticTransformBroadcaster(self)
        self._tbf = TransformBroadcaster(self)
        self._cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('env_topic').value, 10)

        # ── front camera static TF ────────────────────────────────────
        self._front_t = self._make_transform(
            'base_link', 'front_cam',
            self.get_parameter('front_cam_pos').value,
            self.get_parameter('front_cam_rpy').value)
        self._broadcast_static(self._front_t)

        # ── env cloud (built once, republished @1 Hz) ─────────────────
        boxes = self.get_parameter('env_boxes').value
        self._cloud = self._build_cloud(boxes)
        self._last_cloud = 0.0

        # ── latest joint states ───────────────────────────────────────
        from sensor_msgs.msg import JointState
        self._q = None
        self.create_subscription(JointState, 'joint_states', self._state_cb, 10)

        self._tf_dt = 1.0 / max(1.0, float(self.get_parameter('tf_rate').value))
        self._cloud_dt = 1.0
        self._timer = self.create_timer(min(self._tf_dt, self._cloud_dt),
                                        self._tick)

        self.get_logger().info(
            f'visualizer ready: front_cam@{self.get_parameter("front_cam_pos").value}, '
            f'wrist_cam={"on" if self.get_parameter("enable_wrist").value else "off"}, '
            f'env {len(self._cloud["pts"])} pts')

    # ── helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _rpy_to_quat(r, p, y):
        cr, sr = np.cos(r / 2), np.sin(r / 2)
        cp, sp = np.cos(p / 2), np.sin(p / 2)
        cy, sy = np.cos(y / 2), np.sin(y / 2)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        yy = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        n = float(np.sqrt(w * w + x * x + yy * yy + z * z)) or 1.0
        return tuple(float(v / n) for v in (w, x, yy, z))

    def _make_transform(self, parent, child, pos, rpy):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation = Vector3(x=float(pos[0]), y=float(pos[1]),
                                          z=float(pos[2]))
        w, x, y, z = self._rpy_to_quat(float(rpy[0]), float(rpy[1]),
                                       float(rpy[2]))
        t.transform.rotation = (w, x, y, z)
        return t

    def _broadcast_static(self, t):
        self._sbf.send_transform(t)

    def _build_cloud(self, boxes):
        pts = boxes_to_points(boxes)
        import struct
        msg = PointCloud2()
        msg.height = 1
        msg.width = len(pts)
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(pts)
        fields = []
        from sensor_msgs.msg import PointField
        for name, off in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12)):
            f = PointField()
            f.name = name
            f.offset = off
            f.datatype = PointField.FLOAT32
            f.count = 1
            fields.append(f)
        msg.fields = fields
        data = bytearray()
        for (x, y, z, i) in pts:
            data += struct.pack('<ffff', x, y, z, i)
        msg.data = bytes(data)
        msg.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        return {'msg': msg, 'pts': pts}

    # ── callbacks ─────────────────────────────────────────────────────

    def _state_cb(self, msg):
        if msg.name and len(msg.position) == 6:
            self._q = np.array(msg.position[:6], dtype=float)

    def _tick(self):
        now = self.get_clock().now()

        # wrist camera dynamic TF
        if (self.get_parameter('enable_wrist').value and self.dh
                and self._q is not None):
            T = fk(self.dh, self._q)
            tcp_pos = T[:3, 3]
            # wrist_cam offset in tcp frame (fixed) → base
            wpos = self.get_parameter('wrist_cam_pos').value
            wrpy = self.get_parameter('wrist_cam_rpy').value
            R = T[:3, :3]
            p = tcp_pos + R @ np.array(wpos, dtype=float)
            # orientation: tcp R @ rpy offset
            Ro = np.array([
                [1, 0, 0],
                [0, np.cos(wrpy[1]), -np.sin(wrpy[1])],
                [0, np.sin(wrpy[1]), np.cos(wrpy[1])],
            ])  # default offset: identity-ish (pitch-only fast path)
            Rw = R @ Ro
            w, x, y, z = self._rot_to_quat(Rw)
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'tcp'
            t.child_frame_id = 'wrist_cam'
            t.transform.translation = Vector3(x=float(p[0]), y=float(p[1]),
                                              z=float(p[2]))
            t.transform.rotation = (w, x, y, z)
            self._tbf.send_transform(t)

        # republish env cloud
        if now.nanoseconds / 1e9 - self._last_cloud > self._cloud_dt:
            self._cloud['msg'].header.stamp = now.to_msg()
            self._cloud_pub.publish(self._cloud['msg'])
            self._last_cloud = now.nanoseconds / 1e9

    @staticmethod
    def _rot_to_quat(R):
        tr = np.trace(R)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        n = float(np.sqrt(w * w + x * x + y * y + z * z)) or 1.0
        return tuple(float(v / n) for v in (w, x, y, z))


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
