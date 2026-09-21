#!/usr/bin/env python3
"""
arm_driver_node.py — ROS 2 driver for the laundry bot's 6-axis arm.

Bridges ROS 2 and the 6x MKS SERVO42D/57D CAN stepper drivers (protocol from
can_frames.py). The node converts a 6-joint target in radians into per-axis
absolute pulse commands, and publishes joint states from encoder readback.

Topics
  ~  /joint_target   (sensor_msgs/JointState)  IN  6-joint target (rad)
  ~  /joint_states   (sensor_msgs/JointState)  OUT encoder readback (~100 Hz)
  ~  /arm/status     (std_msgs/String)         OUT human-readable state line

Services
  ~  /arm/enable_motors   (laundry_bot_interfaces/EnableMotors)
  ~  /arm/axis_home       (laundry_bot_interfaces/AxisHome)
  ~  /arm/emergency_stop  (laundry_bot_interfaces/EmergencyStop)
  ~  /arm/read_encoder    (laundry_bot_interfaces/ReadEncoder)
  ~  /arm/set_current     (laundry_bot_interfaces/SetCurrent)

Config (YAML, --ros-args -p config:=<file>) — see config/arm.yaml.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import can
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from laundry_bot import can_frames as cf
from laundry_bot_interfaces.srv import (
    EnableMotors, AxisHome, EmergencyStop, ReadEncoder, SetCurrent)


class AxisConfig:
    """Per-axis mapping from joint angle (rad) -> absolute pulses."""

    def __init__(self, name, can_id, pulses_per_rev, direction=1,
                 home_pulses=0, max_speed=300, acc=5, current_ma=1600,
                 min_rad=-3.14159, max_rad=3.14159):
        self.name = name
        self.can_id = can_id
        self.pulses_per_rev = pulses_per_rev
        self.direction = direction
        self.home_pulses = home_pulses
        self.max_speed = max_speed
        self.acc = acc
        self.current_ma = current_ma
        self.min_rad = min_rad
        self.max_rad = max_rad

    def rad_to_pulses(self, rad: float) -> int:
        """Absolute pulse position for a joint angle (relative to home zero)."""
        pulses = (rad / (2.0 * math.pi)) * self.pulses_per_rev
        # direction sign + home offset -> absolute driver coordinate
        return int(round(pulses * self.direction)) + self.home_pulses

    def pulses_to_rad(self, pulses: int) -> float:
        raw = (pulses - self.home_pulses) / self.direction
        return raw / self.pulses_per_rev * (2.0 * math.pi)

    def clamp_rad(self, rad: float) -> float:
        return max(self.min_rad, min(self.max_rad, rad))


class ArmDriverNode(Node):
    def __init__(self):
        super().__init__('arm_driver_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('can_interface', 'socketcan')   # or candle/virtual
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('command_rate', 30.0)          # Hz for joint_target
        self.declare_parameter('feedback_rate', 100.0)        # Hz for joint_states
        self.declare_parameter('axes', None)                  # list of axis dicts
        self.declare_parameter('auto_enable_on_start', False)
        self.declare_parameter('auto_home_on_start', False)

        cfg = self.get_parameter('axes').get_parameter_value().yaml
        if not cfg:
            raise RuntimeError(
                "Parameter 'axes' (list of axis dicts) is required. "
                "See config/arm.yaml for the schema.")
        self.axes = [self._build_axis(i, a) for i, a in enumerate(cfg)]
        self.get_logger().info(
            f"Loaded {len(self.axes)} axes: "
            + ", ".join(f"{a.name}(id={a.can_id})" for a in self.axes))

        self._cmd_rate = float(self.get_parameter('command_rate').value)
        self._fb_rate = float(self.get_parameter('feedback_rate').value)

        # ── CAN bus ────────────────────────────────────────────────────
        iface = self.get_parameter('can_interface').value
        chan = self.get_parameter('can_channel').value
        br = int(self.get_parameter('can_bitrate').value)
        self._bus = can.Bus(iface=iface, channel=chan, bitrate=br)
        self._bus_lock = threading.Lock()
        self.get_logger().info(f"CAN bus open: {iface}:{chan} @ {br} bps")

        # ── State ──────────────────────────────────────────────────────
        self._encoders = {a.can_id: a.home_pulses for a in self.axes}
        self._enabled = {a.can_id: False for a in self.axes}
        self._last_target = [0.0] * len(self.axes)   # rad, smoothed
        self._estopped = False

        # ── ROS interfaces ─────────────────────────────────────────────
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._target_sub = self.create_subscription(
            JointState, 'joint_target', self._target_cb, qos)
        self._state_pub = self.create_publisher(
            JointState, 'joint_states', qos)
        self._status_pub = self.create_publisher(
            String, 'arm/status', 10)

        self.create_service(EnableMotors, 'arm/enable_motors', self._svc_enable)
        self.create_service(AxisHome, 'arm/axis_home', self._svc_home)
        self.create_service(EmergencyStop, 'arm/emergency_stop', self._svc_estop)
        self.create_service(ReadEncoder, 'arm/read_encoder', self._svc_read_enc)
        self.create_service(SetCurrent, 'arm/set_current', self._svc_set_current)

        # ── Threads ────────────────────────────────────────────────────
        self._stop_event = threading.Event()
        self._cmd_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._fb_thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self._cmd_thread.start()
        self._fb_thread.start()

        if self.get_parameter('auto_enable_on_start').value:
            self._enable_all(True)
        if self.get_parameter('auto_home_on_start').value:
            self._enable_all(True)
            for a in self.axes:
                self._home_axis(a)
            self.get_logger().info("Auto-homed all axes on start.")

    def _build_axis(self, i, a: dict) -> AxisConfig:
        return AxisConfig(
            name=a.get('name', f'joint{i + 1}'),
            can_id=int(a['can_id']),
            pulses_per_rev=int(a['pulses_per_rev']),
            direction=int(a.get('direction', 1)),
            home_pulses=int(a.get('home_pulses', 0)),
            max_speed=int(a.get('max_speed', 300)),
            acc=int(a.get('acc', 5)),
            current_ma=int(a.get('current_ma', 1600)),
            min_rad=float(a.get('min_rad', -math.pi)),
            max_rad=float(a.get('max_rad', math.pi)),
        )

    # ── CAN send ───────────────────────────────────────────────────────

    def _send(self, frame: list):
        """frame = [can_id, *data, crc]. Send one standard frame."""
        with self._bus_lock:
            self._bus.send(can.Message(arbitration_id=frame[0],
                                       data=frame[1:-1], is_extended_id=False))

    def _tx(self, can_id: int, data: list):
        self._send(cf._frame(can_id, data))

    # ── High-level axis ops ────────────────────────────────────────────

    def _enable_axis(self, a: AxisConfig, on: bool):
        self._tx(a.can_id, [0xF3, 0x01 if on else 0x00])
        self._enabled[a.can_id] = on

    def _enable_all(self, on: bool):
        for a in self.axes:
            self._enable_axis(a, on)
        self.get_logger().info(f"Motors {'enabled' if on else 'disabled'} (all).")

    def _home_axis(self, a: AxisConfig):
        """Home sequence: set params, go home, set zero, enable."""
        self.get_logger().info(f"Homing {a.name} (can_id={a.can_id}) ...")
        self._tx(a.can_id, [0x90, 0x01, 1, (300 >> 8) & 0xFF, 300 & 0xFF, 1])
        self._tx(a.can_id, [0x91])
        time.sleep(1.0)          # let it reach the limit switch
        self._tx(a.can_id, [0x92])
        self._enable_axis(a, True)
        self._encoders[a.can_id] = a.home_pulses

    def _move_axis_abs(self, a: AxisConfig, rad: float):
        if self._estopped:
            return
        rad = a.clamp_rad(rad)
        pulses = a.rad_to_pulses(rad)
        self._tx(a.can_id, [0xFE,
                            (a.max_speed >> 8) & 0xFF, a.max_speed & 0xFF,
                            a.acc,
                            (pulses & 0xFFFFFF) >> 16,
                            (pulses & 0xFFFFFF) >> 8,
                            pulses & 0xFF])

    # ── Callbacks ──────────────────────────────────────────────────────

    def _target_cb(self, msg: JointState):
        if len(msg.position) < len(self.axes):
            self.get_logger().warn(
                f"joint_target has {len(msg.position)} positions, "
                f"expected {len(self.axes)}; ignoring.")
            return
        self._last_target = [float(p) for p in msg.position[:len(self.axes)]]

    def _svc_enable(self, req, _):
        if req.axis < 0:
            self._enable_all(req.enable)
            return EnableMotors.Response(success=True,
                                         message="all axes")
        a = self.axes[req.axis]
        self._enable_axis(a, req.enable)
        return EnableMotors.Response(success=True, message=a.name)

    def _svc_home(self, req, _):
        a = self.axes[req.axis]
        self._home_axis(a)
        return AxisHome.Response(success=True, message=f"{a.name} homed")

    def _svc_estop(self, req, _):
        self._estopped = True
        for a in self.axes:
            self._tx(a.can_id, [0xF7])
            self._tx(a.can_id, [0xF3, 0x00])
        self.get_logger().error("EMERGENCY STOP: all axes stopped & disabled.")
        return EmergencyStop.Response(success=True, message="estop sent to all axes")

    def _svc_read_enc(self, req, _):
        a = self.axes[req.axis]
        val = int(self._encoders.get(a.can_id, a.home_pulses))
        return ReadEncoder.Response(success=True, encoder_value=val,
                                    message=a.name)

    def _svc_set_current(self, req, _):
        a = self.axes[req.axis]
        ma = max(0, min(5200, req.ma))
        self._tx(a.can_id, [0x83, (ma >> 8) & 0xFF, ma & 0xFF])
        a.current_ma = ma
        return SetCurrent.Response(success=True,
                                   message=f"{a.name} current={ma}mA")

    # ── Loops ──────────────────────────────────────────────────────────

    def _command_loop(self):
        period = 1.0 / self._cmd_rate
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            if not self._estopped:
                for a, rad in zip(self.axes, self._last_target):
                    self._move_axis_abs(a, rad)
            self._sleep(period - (time.monotonic() - t0))

    def _feedback_loop(self):
        period = 1.0 / self._fb_rate
        # round-robin encoder poll: one 0x31 query per axis per cycle
        idx = 0
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            if not self._estopped:
                a = self.axes[idx % len(self.axes)]
                self._tx(a.can_id, [0x31])
                idx += 1
                # drain responses for this axis (non-blocking)
                self._drain_responses(a)
            # publish joint_states
            self._publish_state()
            self._sleep(period - (time.monotonic() - t0))

    def _drain_responses(self, a: AxisConfig):
        try:
            while True:
                msg = self._bus.recv(timeout=0.0)
                if msg is None:
                    break
                if msg.arbitration_id == a.can_id:
                    val = cf.parse_query_response("encoder_add", bytes(msg.data))
                    if val is not None:
                        self._encoders[a.can_id] = int(val)
        except Exception:
            pass

    def _publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [a.name for a in self.axes]
        msg.position = [a.pulses_to_rad(int(self._encoders[a.can_id]))
                        for a in self.axes]
        msg.velocity = [0.0] * len(self.axes)
        self._state_pub.publish(msg)

        en = "".join("1" if self._enabled[a.can_id] else "0" for a in self.axes)
        self._status_pub.publish(String(
            data=f"axes={en} enc="
                 + " ".join(f"{a.pulses_to_rad(int(self._encoders[a.can_id])):.2f}"
                            for a in self.axes)))

    def _sleep(self, dt):
        if dt > 0:
            self._stop_event.wait(dt)

    def destroy_node(self):
        self._stop_event.set()
        try:
            self._enable_all(False)
            self._bus.destroy()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
