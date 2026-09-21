#!/usr/bin/env python3
"""
teleop_quest_node.py — Meta Quest 2 (via Quest2ROS2) -> arm joint targets.

Bridge contract (verified against Taokt/Quest2ROS2, HRI 2026, ROS 2 Humble):
    /q2r_right_hand_pose    geometry_msgs/PoseStamped     controller pose
    /q2r_right_hand_inputs  quest2ros/OVR2ROSInputs       buttons:
                                bool button_upper          (touchpad press)
                                bool button_lower          (lower button)
                                float32 press_index        (trigger)
                                float32 press_middle       (grip)
                                ... thumbstick fields
    /q2r_right_hand_twist   geometry_msgs/Twist           velocity (unused here)

Frame note: the pose is in the Quest world frame (X right, Y forward, Z up,
right-handed), NOT the robot base frame. `pos_map` (3x3, row-major) rotates
the controller position into the base frame. The default assumes you stand
behind the arm facing it (so VR-forward = robot +X, VR-right = robot -Y):
    robot_x = +quest_y ; robot_y = -quest_x ; robot_z = +quest_z
If motion feels mirrored/wrong, change pos_map (see docs/quest2ros2_setup.md).
Translation is handled automatically by the latch offset; a constant yaw
misalignment is also absorbed at latch time.

Clutch operation (Q2R2 convention — lower button toggles motion):
    * press LOWER BUTTON -> LATCH: offset = arm_TCP - pos_map*controller_pos,
      yaw_offset = arm_yaw - controller_yaw
    * while latched      -> arm follows your hand (filtered, clamped)
    * press again        -> UNLATCH, arm holds
    * a Bool on `teleop/latch` (true=latch, false=unlatch) also works — wire
      a keyboard node or physical button to it if you prefer.

Safety:
    * IK residual > ik_max_error  -> target NOT sent (arm holds) + warning
    * TCP clamped to workspace box (ur7e_like.yaml)
    * per-tick joint velocity clamped (max_jstep_rad per control tick)
    * |q5| < singularity_q5 -> wrist-singularity warning
    * stale controller feed > latch_timeout_s -> hold
"""
from __future__ import annotations

import math

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from laundry_bot import kinematics as K

# Default: Quest world (X right, Y fwd, Z up) -> base (X fwd, Y left, Z up)
DEFAULT_POS_MAP = [[0.0, 1.0, 0.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0]]


class TeleopQuestNode(Node):
    def __init__(self):
        super().__init__('teleop_quest_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('kin_config', '')              # ur7e_like.yaml
        self.declare_parameter('hand', 'right')
        self.declare_parameter('pose_topic', '')              # '' = default per hand
        self.declare_parameter('inputs_topic', '')            # '' = default per hand
        self.declare_parameter('latch_topic', 'teleop/latch')  # Bool fallback
        self.declare_parameter('latch_button', 'button_lower')  # or 'press_index'
        self.declare_parameter('pos_map', DEFAULT_POS_MAP)    # 3x3 row-major
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('joint_target_topic', 'joint_target')
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('pos_gain', 1.0)               # scale (0.4-1.0 = finer)
        self.declare_parameter('pos_smooth', 0.35)            # low-pass per tick
        self.declare_parameter('orientation_mode', 'yaw')     # 'yaw' | 'full'
        self.declare_parameter('ik_max_error', 0.03)          # m; above -> hold
        self.declare_parameter('max_jstep_rad', 0.05)         # per 30 Hz tick
        self.declare_parameter('latch_timeout_s', 10.0)

        kin_path = self.get_parameter('kin_config').value
        if not kin_path:
            raise RuntimeError("Parameter 'kin_config' (path to kinematics YAML) is required.")
        self.kin = K.load_kin_config(kin_path)
        self.dh = self.kin['dh']
        self.theta_offset = self.kin['theta_offset']
        self.tool_offset = self.kin['tool_offset']
        self.home_pose = self.kin.get('home_pose', np.zeros(6))
        ws = self.kin.get('workspace')
        self.ws_min = np.array(ws['min']) if ws else np.array([0.0, -1.0, -0.3])
        self.ws_max = np.array(ws['max']) if ws else np.array([1.0, 1.0, 0.8])
        self.sing_q5 = float(ws['singularity_q5']) if ws else 0.15

        self._rate = float(self.get_parameter('control_rate').value)
        self._gain = float(self.get_parameter('pos_gain').value)
        self._smooth = float(self.get_parameter('pos_smooth').value)
        self._orient_full = self.get_parameter('orientation_mode').value == 'full'
        self._ik_max = float(self.get_parameter('ik_max_error').value)
        self._max_jstep = float(self.get_parameter('max_jstep_rad').value)
        self._pos_map = np.array(
            self.get_parameter('pos_map').value, dtype=float).reshape(3, 3)

        hand = self.get_parameter('hand').value
        pose_topic = self.get_parameter('pose_topic').value or f'/q2r_{hand}_hand_pose'
        inputs_topic = self.get_parameter('inputs_topic').value or f'/q2r_{hand}_hand_inputs'

        # ── State ──────────────────────────────────────────────────────
        self._last_pos = None            # (3,) controller, quest frame
        self._last_quat = None           # (w,x,y,z)
        self._last_stamp = None
        self._latched = False
        self._pos_offset = np.zeros(3)
        self._yaw_offset = 0.0
        self._target_q = self.home_pose.copy()
        self._last_lower = False
        self._filter_pos = None
        self._warn_ik = False
        self._warn_sing = False

        # ── ROS ────────────────────────────────────────────────────────
        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        try:
            # custom message from the `quest2ros` msg package (Taokt/Quest2ROS2)
            from quest2ros.msg import OVR2ROSInputs
            self.create_subscription(OVR2ROSInputs, inputs_topic, self._inputs_cb, 10)
            self._have_inputs = True
        except Exception as e:
            self.get_logger().warn(
                f"Could not import quest2ros.msg.OVR2ROSInputs ({e}); "
                "using Bool latch topic only.")
            self._have_inputs = False
        self.create_subscription(JointState,
                                 self.get_parameter('joint_states_topic').value,
                                 self._state_cb, 10)
        self.create_subscription(Bool, self.get_parameter('latch_topic').value,
                                 self._latch_cb, 10)
        self._pub = self.create_publisher(
            JointState, self.get_parameter('joint_target_topic').value, 10)
        self._status_pub = self.create_publisher(String, 'teleop/status', 10)

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"Teleop ready. pose={pose_topic} inputs={inputs_topic} "
            f"(inputs={'yes' if self._have_inputs else 'NO — Bool latch only'}), "
            f"hand={hand}, gain={self._gain}, mode={self.get_parameter('orientation_mode').value}, "
            f"home={np.round(self.home_pose, 2)}")

    # ── Callbacks ──────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._last_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.orientation
        self._last_quat = K.quat_normalize(np.array([q.w, q.x, q.y, q.z]))
        self._last_stamp = self.get_clock().now()

    def _inputs_cb(self, msg):
        """Edge-detect the latch button."""
        btn = self.get_parameter('latch_button').value
        val = bool(getattr(msg, btn, False))
        if btn == 'press_index':
            val = bool(getattr(msg, 'press_index', 0.0))
        if val != self._last_lower:
            self._last_lower = val
            if val:
                self._do_latch()
            else:
                self._do_unlatch()

    def _state_cb(self, msg: JointState):
        if len(msg.position) == 6:
            self._arm_state = np.array(msg.position, dtype=float)

    def _latch_cb(self, msg: Bool):
        if msg.data:
            self._do_latch()
        else:
            self._do_unlatch()

    def _do_latch(self):
        if self._latched:
            return
        if self._last_pos is None:
            self.get_logger().warn("Latch requested before first controller pose; ignored.")
            return
        self._latched = True
        p_now, _ = K.fk_pose(self.dh, self._target_q,
                             self.theta_offset, self.tool_offset)
        # offset chosen so that at latch time target == current TCP exactly
        self._pos_offset = p_now - self._gain * (self._pos_map @ self._last_pos)
        _, q_now = K.fk_pose(self.dh, self._target_q,
                             self.theta_offset, self.tool_offset)
        self._yaw_offset = self._quat_yaw(q_now) - self._quat_yaw(self._last_quat)
        self.get_logger().info("LATCHED — arm follows controller.")

    def _do_unlatch(self):
        if not self._latched:
            return
        self._latched = False
        self._filter_pos = None
        self.get_logger().info("Unlatched — arm holding.")

    @staticmethod
    def _quat_yaw(q):
        w, x, y, z = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # ── Control tick ───────────────────────────────────────────────────

    def _tick(self):
        now = self.get_clock().now()
        if self._last_pos is None:
            return
        age = (now - self._last_stamp).nanoseconds * 1e-9
        if age > self.get_parameter('latch_timeout_s').value:
            return

        if self._latched:
            cp = self._last_pos
            if self._filter_pos is None:
                self._filter_pos = cp.copy()
            else:
                self._filter_pos = (self._smooth * cp
                                    + (1.0 - self._smooth) * self._filter_pos)
            target_pos = self._pos_map @ self._filter_pos * self._gain + self._pos_offset
            target_pos = np.clip(target_pos, self.ws_min, self.ws_max)

            if self._orient_full:
                target_quat = self._last_quat
            else:
                # yaw mode: arm wrist yaw = controller yaw + latch offset,
                # achieved by rotating the CURRENT wrist orientation about
                # base Z by the *delta* (preserves wrist pitch/roll).
                _, q_now = K.fk_pose(self.dh, self._target_q,
                                     self.theta_offset, self.tool_offset)
                target_yaw = self._quat_yaw(self._last_quat) + self._yaw_offset
                delta = target_yaw - self._quat_yaw(q_now)
                yaw_q = K.quat_axis_angle(np.array([0.0, 0.0, 1.0]), delta)
                target_quat = K.quat_normalize(K.quat_mul(yaw_q, q_now))
        else:
            target_pos, target_quat = K.fk_pose(
                self.dh, self._target_q, self.theta_offset, self.tool_offset)

        q_new, pos_err, rot_err = K.ik_solve_robust(
            self.dh, self._target_q, target_pos, target_quat,
            self.theta_offset, self.tool_offset,
            pos_weight=1.0, rot_weight=1.0 if self._orient_full else 0.3)

        if pos_err > self._ik_max:
            if not self._warn_ik:
                self.get_logger().warn(
                    f"IK residual {pos_err * 1000:.1f} mm > {self._ik_max * 1000:.0f} mm — holding")
                self._warn_ik = True
            return
        self._warn_ik = False

        dq = np.clip(q_new - self._target_q, -self._max_jstep, self._max_jstep)
        q_new = self._target_q + dq

        if abs(q_new[4]) < self.sing_q5:
            if not self._warn_sing:
                self.get_logger().warn(
                    f"Wrist near singularity |q5|={abs(q_new[4]):.2f} < {self.sing_q5}")
                self._warn_sing = True
        else:
            self._warn_sing = False

        self._target_q = q_new
        self._publish_target(q_new)

    def _publish_target(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i + 1}' for i in range(6)]
        msg.position = [float(v) for v in q]
        self._pub.publish(msg)
        tcp = K.fk_pose(self.dh, q, self.theta_offset, self.tool_offset)[0]
        self._status_pub.publish(String(
            data=f"latched={int(self._latched)} tcp={np.round(tcp, 3)}"))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopQuestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
