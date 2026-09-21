#!/usr/bin/env python3
"""
fake_camera_node.py — synthetic Image publisher for hardware-free bring-up.

Publishes a moving pattern (scrolling gradient + moving circle + frame
counter overlay-free) on one or two Image topics so the LeRobot recorder
and the whole image path can be exercised before any real camera exists.
The moving content is deliberate: a stale-image bug makes the pattern
freeze, which is obvious when you inspect the recorded dataset.
"""
from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


def make_frame(t: int, w: int, h: int, seed: int) -> np.ndarray:
    """BGR uint8 HxWx3 frame with a moving diagonal band + orbiting blob."""
    y = np.arange(h)[:, None]
    x = np.arange(w)[None, :]
    phase = (x + y + t * 3) % (w + h)
    band = (phase < 40).astype(np.uint8) * 255
    img = np.full((h, w, 3), 30, dtype=np.uint8)
    img[..., 0] = (y % 256).astype(np.uint8)
    img[..., 1] = (x % 256).astype(np.uint8)
    img[..., 2] = band
    cx = int(w / 2 + 0.35 * w * np.cos(t * 0.05 + seed))
    cy = int(h / 2 + 0.30 * h * np.sin(t * 0.05 + seed))
    r = max(3, min(w, h) // 8)
    yy, xx = np.ogrid[:h, :w]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r
    img[mask] = (255, 255, 0)
    return img


class FakeCameraNode(Node):
    def __init__(self):
        super().__init__('fake_camera_node')
        self.declare_parameter('topic_front', '/oak_d/color/image_raw')
        self.declare_parameter('topic_wrist', '/wrist_cam/image_raw')
        self.declare_parameter('enable_wrist', False)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)

        self._pubs = [self.create_publisher(
            Image, self.get_parameter('topic_front').value, 10)]
        self._seeds = [0]
        if self.get_parameter('enable_wrist').value:
            self._pubs.append(self.create_publisher(
                Image, self.get_parameter('topic_wrist').value, 10))
            self._seeds.append(2.1)

        self._t = 0
        self._cv = CvBridge() if CvBridge else None
        self._timer = self.create_timer(1.0 / fps, self._tick)
        self.get_logger().info(
            f"Fake camera: {len(self._pubs)} topic(s) @ {fps} Hz "
            f"{w}x{h} (front={self.get_parameter('topic_front').value})")

    def _encode(self, bgr: np.ndarray, stamp) -> Image:
        if self._cv is not None:
            return self._cv.cv2_to_imgmsg(bgr, encoding='bgr8')
        # manual fallback
        msg = Image()
        msg.header = Header(stamp=stamp, frame_id='fake_cam')
        msg.height = bgr.shape[0]
        msg.width = bgr.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = bgr.shape[1] * 3
        msg.data = bgr.tobytes()
        return msg

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        for pub, seed in zip(self._pubs, self._seeds):
            bgr = make_frame(self._t, int(self.get_parameter('width').value),
                             int(self.get_parameter('height').value), seed)
            pub.publish(self._encode(bgr, stamp))
        self._t += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
