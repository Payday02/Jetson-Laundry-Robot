# Status & Next Steps — 2026-09-20

State of the project: all code written and verified offline (pure-logic + simulated).
Nothing has touched real hardware yet.

## What works today (no hardware needed)

- `ros2 launch laundry_bot bringup_virtual.launch.py dataset_root:=/tmp/ds`
  runs the full pipeline on a virtual CAN bus: driver → fake Quest teleop →
  IK → joint targets → LeRobot v3.0 dataset recording (2 synthetic cameras).
- `ros2 run laundry_bot record_cli start|stop` controls episodes.
- Verified: CAN protocol byte-exact, IK 25/25 round-trips @ <0.01 mm,
  120 s end-to-end sim with 0 safety holds.

## Blocked on the arm design (fill these when dimensions are final)

`config/ur7e_like.yaml`
- `dh`: six rows of `[d, a, alpha]` — currently the **official UR7e/UR5e table
  as a placeholder**. Swap in your real link lengths, then verify with:
  `python3 kinematics.py config/ur7e_like.yaml` (self-test runs on load).
- `home_pose`: must equal the physical pose after your limit-switch homing,
  and stay well-conditioned (avoid |q5| near 0 — wrist singularity).
  Current placeholder: `[0.0, -1.85, 0.0, -1.75, 1.6, 0.0]`.

`config/arm.yaml` (per axis 1–6)
- `pulses_per_rev` — full steps × microstep setting (from the driver config)
- `min_rad` / `max_rad` — your real joint limits
- `direction` — flip if the joint moves backward vs. the sign convention
- `current_ma` — running current (keep modest; enable stall protection 0x88
  on every driver — steppers have no force feedback)

## Jetson bring-up sequence (when the arm is assembled)

1. JetPack 6.2 (L4T 36, Ubuntu 22.04) + ROS 2 Humble
2. `colcon build` in this workspace (deps: rclpy, python-can, numpy,
   lerobot==0.6.1, depthai + OAK-D ros driver, webcam via v4l2)
3. `arm.yaml`: `can_interface: candle` (CANalyst-II) or `socketcan` (USB adapter),
   channel + bitrate 500 k, driver CAN IDs 1–6
4. Single-axis test: home, `ros2 topic echo /joint_states` vs. `/joint_target`,
   hit estop. Then all six axes.
5. OAK-D: fixed/elevated mount (~40–60 cm above workspace, angled down),
   depthai-ros driver. Webcam: wrist mount.

## Quest 2 teleop

See `docs/quest2ros2_setup.md` — uses Taokt/Quest2ROS2 (Humble-compatible).
Latch = lower button (Q2R2 convention), matches the teleop node's clutch.
Verify link with `ros2 run q2r2_bringup CheckTCPconnection` before arming.

## Fine-tuning (on the AMD 7900 XTX desktop)

- PyTorch + ROCm supports the 7900 XTX officially; 24 GB is plenty.
- Data is already in LeRobot v3.0 format — `lerobot-train` runs on ROCm
  for SmolVLA/ACT/Diffusion policies; GR00T fine-tuning via the
  Isaac-GR00T/LeRobot path.
- Caveat: TensorRT engine export for the Jetson (INT8/FP8) is NVIDIA-only —
  do that step on the Jetson itself, or rent cloud NVIDIA time briefly.

## Model plan (from earlier evaluation)

- Primary on-board: GR00T N1.5 (INT8, ~2–5 Hz + action chunking on 8 GB Orin Nano —
  fine for laundry speeds)
- On-board fallback: SmolVLA 450M (comfortable real-time; swap-in via vla_node)
- Baseline reference: π0.5 (not publicly runnable — openpi π0 is the runnable relative)
- Decomposition: classify (VLM) → per-skill policies (grab / dress / hook / drop)
  → ROS 2 state machine with retry. Collect ~200–300 teleop episodes total.
