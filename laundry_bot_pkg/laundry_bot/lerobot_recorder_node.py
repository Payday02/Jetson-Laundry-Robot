#!/usr/bin/env python3
"""
lerobot_recorder_node.py — captures teleoperation demos in LeRobotDataset
format (v3.0) for GR00T / LeRobot policy fine-tuning.

Runs on the Orin Nano at the dataset fps (default 30). For each frame it
captures:
    observation.images.front   <- OAK-D RGB (Isaac ROS / image_raw)
    observation.images.wrist   <- wrist webcam (optional, if mounted)
    observation.state          <- current arm joint state (rad)  [6]
    action                     <- commanded joint target (rad)   [6]
    task                       <- natural-language task string

The operator drives the arm with teleop_quest_node (or any other teleop),
and this node records everything in lock-step at the dataset rate.

Episode control (ROS 2 services, so a second human / script can toggle):
    /record/start_episode   (StartEpisode)  task string -> begin buffering
    /record/stop_episode    (StopEpisode)   discard=false saves, true throws away

Storage layout is standard LeRobot (meta/, data/*.parquet, videos/*.mp4 per
camera) so it can be uploaded to the HF Hub and fed straight to
`lerobot-train` or Isaac GR00T fine-tuning.

NOTE on images: LeRobot stores RGB as (H, W, 3) uint8 for image dtype or
encodes to mp4 for video dtype. We feed uint8 HxWx3 arrays.
"""
from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

from laundry_bot_interfaces.srv import StartEpisode, StopEpisode


class LeRobotRecorderNode(Node):
    def __init__(self):
        super().__init__('lerobot_recorder_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('repo_id', 'laundry_bot/teleop')
        self.declare_parameter('root', '')                 # local dataset dir
        self.declare_parameter('fps', 30)
        self.declare_parameter('use_videos', True)
        self.declare_parameter('robot_type', 'laundry_bot_ur7e_like')
        self.declare_parameter('front_image_topic', '/camera/color/image_raw')
        self.declare_parameter('wrist_image_topic', '/wrist_cam/image_raw')
        self.declare_parameter('use_wrist', False)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('joint_target_topic', 'joint_target')
        self.declare_parameter('num_joints', 6)

        self._fps = int(self.get_parameter('fps').value)
        self._use_videos = bool(self.get_parameter('use_videos').value)
        self._use_wrist = bool(self.get_parameter('use_wrist').value)
        self._H = int(self.get_parameter('image_height').value)
        self._W = int(self.get_parameter('image_width').value)
        self._nq = int(self.get_parameter('num_joints').value)
        root = self.get_parameter('root').value
        repo_id = self.get_parameter('repo_id').value
        robot_type = self.get_parameter('robot_type').value

        # ── Build LeRobot dataset (write mode) ─────────────────────────
        from lerobot.datasets import LeRobotDataset

        dtype_img = "video" if self._use_videos else "image"
        features = {
            "observation.state": {"dtype": "float32", "shape": (self._nq,), "names": None},
            "action":            {"dtype": "float32", "shape": (self._nq,), "names": None},
            "observation.images.front": {
                "dtype": dtype_img, "shape": (3, self._H, self._W),
                "names": ["channels", "height", "width"]},
        }
        if self._use_wrist:
            features["observation.images.wrist"] = {
                "dtype": dtype_img, "shape": (3, self._H, self._W),
                "names": ["channels", "height", "width"]}

        existing = bool(root) and os.path.isdir(os.path.join(root, "meta"))
        if existing:
            self._ds = LeRobotDataset.resume(repo_id=repo_id, root=root or None)
            self.get_logger().info(f"Resumed existing dataset at {root}")
        else:
            self._ds = LeRobotDataset.create(
                repo_id=repo_id, fps=self._fps, features=features,
                root=root or None, robot_type=robot_type,
                use_videos=self._use_videos)
            self.get_logger().info(
                f"Created new dataset repo_id={repo_id} "
                f"fps={self._fps} videos={self._use_videos}")

        # ── State ──────────────────────────────────────────────────────
        self._cv = CvBridge()
        self._latest_front = None      # HxWx3 uint8
        self._latest_wrist = None
        self._latest_state = np.zeros(self._nq, dtype=np.float32)
        self._latest_action = np.zeros(self._nq, dtype=np.float32)
        self._recording = False
        self._current_task = ""
        self._frame_count = 0
        self._img_lock = threading.Lock()

        # ── ROS interfaces ─────────────────────────────────────────────
        img_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image,
                                 self.get_parameter('front_image_topic').value,
                                 self._front_cb, img_qos)
        if self._use_wrist:
            self.create_subscription(Image,
                                     self.get_parameter('wrist_image_topic').value,
                                     self._wrist_cb, img_qos)
        self.create_subscription(JointState,
                                 self.get_parameter('joint_states_topic').value,
                                 self._state_cb, 10)
        self.create_subscription(JointState,
                                 self.get_parameter('joint_target_topic').value,
                                 self._action_cb, 10)

        self.create_service(StartEpisode, 'record/start_episode', self._svc_start)
        self.create_service(StopEpisode, 'record/stop_episode', self._svc_stop)
        self._status_pub = self.create_publisher(String, 'record/status', 10)

        # ── Capture timer at dataset fps ───────────────────────────────
        self._timer = self.create_timer(1.0 / self._fps, self._capture_tick)
        self.get_logger().info(
            f"Recorder armed. Front topic "
            f"{self.get_parameter('front_image_topic').value}, wrist={self._use_wrist}")

    # ── Image / state callbacks (latest-value) ─────────────────────────

    def _img_to_hwc(self, msg: Image) -> np.ndarray:
        arr = self._cv.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        arr = cv2.resize(arr, (self._W, self._H))
        return cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

    def _front_cb(self, msg):
        try:
            img = self._img_to_hwc(msg)
        except Exception as e:
            self.get_logger().warn(f"front image decode failed: {e}")
            return
        with self._img_lock:
            self._latest_front = img

    def _wrist_cb(self, msg):
        try:
            img = self._img_to_hwc(msg)
        except Exception:
            return
        with self._img_lock:
            self._latest_wrist = img

    def _state_cb(self, msg):
        if len(msg.position) >= self._nq:
            self._latest_state = np.array(
                msg.position[:self._nq], dtype=np.float32)

    def _action_cb(self, msg):
        if len(msg.position) >= self._nq:
            self._latest_action = np.array(
                msg.position[:self._nq], dtype=np.float32)

    # ── Episode services ───────────────────────────────────────────────

    def _svc_start(self, req, _):
        if self._recording:
            self.get_logger().warn("Already recording; start ignored.")
            return StartEpisode.Response(
                success=False, episode_index=self._ds.num_episodes,
                message="already recording")
        self._recording = True
        self._current_task = req.task
        self._frame_count = 0
        self.get_logger().info(f"▶ START episode task='{req.task}'")
        return StartEpisode.Response(
            success=True, episode_index=self._ds.num_episodes,
            message="recording")

    def _svc_stop(self, req, _):
        if not self._recording:
            return StopEpisode.Response(success=False, num_frames=0,
                                        message="not recording")
        self._recording = False
        n = self._frame_count
        try:
            if req.discard:
                self._ds.clear_episode_buffer(delete_images=True)
                self.get_logger().warn(f"✖ DISCARD episode ({n} frames)")
            else:
                self._ds.save_episode(parallel_encoding=True)
                self.get_logger().info(f"✔ SAVE episode ({n} frames)")
        except Exception as e:
            self.get_logger().error(f"stop_episode failed: {e}")
            return StopEpisode.Response(success=False, num_frames=n, message=str(e))
        return StopEpisode.Response(success=True, num_frames=n,
                                    message="discarded" if req.discard else "saved")

    # ── Capture tick (dataset fps) ─────────────────────────────────────

    def _capture_tick(self):
        if not self._recording:
            return
        with self._img_lock:
            front = self._latest_front
            wrist = self._latest_wrist
        if front is None:
            # No camera frame yet this cycle — skip (don't push a stale/None)
            return
        frame = {
            "task": self._current_task,
            "observation.state": self._latest_state.copy(),
            "action": self._latest_action.copy(),
            "observation.images.front": front.copy(),
        }
        if self._use_wrist:
            if wrist is None:
                return
            frame["observation.images.wrist"] = wrist.copy()
        self._ds.add_frame(frame)
        self._frame_count += 1

    def destroy_node(self):
        try:
            if self._recording:
                self.get_logger().warn("Shutting down mid-episode; discarding buffer.")
                self._ds.clear_episode_buffer(delete_images=True)
            self._ds.finalize()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeRobotRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
