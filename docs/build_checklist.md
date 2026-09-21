# Build Checklist — from finished arm to first training data

Step-by-step for when the arm hardware exists. Each step says **what to do**,
**how to verify success**, and **what to do if it fails**. Work top to bottom;
each step is independently useful.

Estimated wall-clock if nothing goes wrong: **4–6 focused days** for steps 1–6.
Data collection (step 7) then runs in parallel over the following weeks.

---

## 0. Before you start — inventory

| Item | Status |
|---|---|
| Arm: 6× steppers + MKS SERVO42D/57D drivers, assembled | ☐ |
| USB CAN adapter (the one used with the old tkinter GUI) | ☐ |
| Physical e-stop button (normally-open, wired to the drivers' EN/limit inputs or a relay) | ☐ |
| Jetson Orin Nano flashed with JetPack 6.2 | ☐ |
| OAK-D + USB webcam, both on the same USB bus as the CAN adapter is fine (USB3 preferred) | ☐ |
| Quest 2 headset + Windows/Linux PC or the Jetson itself (for the bridge) | ☐ |
| Power: one bench PSU per driver bank, fused | ☐ |

If the e-stop isn't wired yet, **do step 2 before anything else** — a software
estop only works if the software is running.

---

## 1. Fill in the real dimensions (30 min, laptop)

When the design is final, you need these numbers:

**`config/ur7e_like.yaml`**
- `dh`: six rows of `[d, a, alpha]` — measure link lengths (axis to axis) and
  offsets. If your arm is a UR7e clone with different link lengths, only `a2`
  (J1→J2), `a3` (J2→J3), `d1` (base height) and the wrist `d4/d5/d6` change.
- `home_pose`: the joint angles the arm physically sits in after homing
  (encoders zeroed at the limit switches). It must be well-conditioned:
  keep `|q5| > 0.15` and check the condition number (step 1b).

**`config/urdf/laundry_arm.urdf.xacro`** — the `<xacro:property>` values must
match `dh` or the RViz model won't track the real arm.

**`config/arm.yaml`**, per axis:
- `can_id`: what you set on each driver (cmd `0x8B` from the old GUI)
- `pulses_per_rev`: full steps × microstep (400×16 = 6400 for 42D,
  200×16 = 3200 for 57D at 1/16 — read from your driver config)
- `min_rad` / `max_rad`: the physical joint limits minus ~5° of margin
- `direction`: set `1`, test (step 4), flip to `-1` if the joint moves
  backwards
- `current_ma`: your driver's working current
- `max_speed`: keep conservative (120 RPM for the big joints, 200 wrist)

**Verify (1a):**
```bash
cd ~/laundry_bot/laundry_bot_pkg   # your colcon workspace
python3 -m laundry_bot.kinematics config/ur7e_like.yaml
# → "self-test PASS: 25/25 targets converged, max err < 1e-3 mm"
```

**Check conditioning (1b):**
```bash
python3 -c "
import numpy as np, sys; sys.path.insert(0,'.')
from laundry_bot.kinematics import load_kin_config, numerical_jacobian
c = load_kin_config('config/ur7e_like.yaml')
J = numerical_jacobian(c['dh'], np.array(c['home_pose']))
print('cond(J) =', np.linalg.cond(J))"
# → want < ~100. If worse, nudge q5 in home_pose away from 0.
```

---

## 2. Wire the e-stop (1 h)

1. Route the e-stop button through the MKS drivers' enable inputs (or a relay
   that cuts the drivers' VCC). Normally-open: button not pressed = safe.
2. Verify by hand: press button → all drivers drop out (you should hear/feel
   the steps go free), release → nothing happens until the software re-enables.
3. **Do not** rely on the software `estop` service as your primary stop — it's
   the fast secondary, the hardware button is the primary.

**Verify:** with motors enabled by the old tkinter GUI, the physical button
stops everything within one CAN frame (~1 ms).

---

## 3. Jetson software base (2–3 h, mostly waiting)

```bash
# JetPack 6.2 should already be on. Install ROS 2 Humble + deps:
sudo apt install -y ros-humble-ros-base ros-humble-xacro ros-humble-joint-state-publisher-gui
# rviz2 comes with ros-humble-desktop; skip if you're headless:
sudo apt install -y ros-humble-desktop   # only for the Jetson, for rviz

# CAN driver + interface (USB adapter):
sudo apt install -y can-utils
sudo ip link add can0 type can bitrate 500000
sudo ip link set can0 up
candump can0        # → should show nothing, no errors, adapter present

# workspace:
mkdir -p ~/laundry_bot && cd ~/laundry_bot
git clone https://github.com/Payday02/Jetson-Laundry-Robot.git src
colcon build
source install/setup.bash
```

**Verify:** `ros2 run laundry_bot arm_driver_node --help` prints (no crash),
and `candump can0` shows no `ERROR`/`OVERLOAD` frames.

**If it fails:** `dmesg | tail` after `ip link set can0 up` usually tells you
(usually: adapter not detected → try another USB port, or `lsusb`).

---

## 4. Driver bring-up, one axis (1 h) — **first power to motors**

Leave `auto_enable_on_start: false` and `auto_home_on_start: false` in
`arm.yaml`. Start the driver, then drive axis 1 from the terminal:

Start the driver alone (no launch file yet — one axis at a time):
```bash
ros2 run laundry_bot arm_driver_node --ros-args \
  -p can_interface:=socketcan -p can_channel:=can0
```

In a second terminal:
```bash
# 1. enable just the first axis by calling the service with its id,
#    or set auto_enable_on_start:=true temporarily
ros2 service call /arm_driver/enable laundry_bot_interfaces/srv/EnableMotors \
  "{enable: true}"

# 2. watch feedback
ros2 topic echo /joint_states

# 3. command the first joint 30° and watch it move
ros2 topic pub --once /joint_target sensor_msgs/msg/JointState \
  "{name: [shoulder_pan, shoulder_lift, elbow_flex, wrist_1, wrist_2, wrist_3],
    position: [0.5236, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

**Verify:** the joint turns ~30°, `/joint_states` position for axis 1 tracks
the command within ±1° (encoder resolution), and the old GUI's sequence
"Read Encoder (0x31)" agrees.

**If it moves the wrong way:** flip that axis's `direction` in `arm.yaml`,
restart. If it moves but feedback is wrong: check `pulses_per_rev` and the
encoder wiring (A/B swapped gives negative counts). **If it moves wildly or
overshoots:** press the physical e-stop and check `max_speed`/`acc` — the
driver interpolates on its own, so a wrong `pulses_per_rev` makes a 30°
command look like 3000°.

Then repeat for axes 2–6, one at a time, **holding the arm** the first time
each joint moves.

---

## 5. Homing + full-arm motion (1 h)

1. With the arm physically at its home pose (limit switches definable), call
   `AxisHome` and confirm each encoder reads 0 at the stop.
2. Run the old GUI's "Go Home" sequence against the same drivers to
   cross-check: the two should agree to within 1 pulse.
3. Cycle all six joints through their full range slowly (`max_speed: 60`
   temporarily) while watching for binding, creaking, and CAN errors:
   ```bash
   ros2 topic hz /joint_states        # steady 100 Hz?
   candump can0 | grep -i error       # empty?
   ```
4. Restore `max_speed` values.

**Verify:** a full-range cycle of every joint is smooth, `/joint_states`
stays at 100 Hz, no CAN errors, estop works mid-motion.

---

## 6. Cameras (1 h)

**OAK-D (fixed, elevated mount):**
```bash
# depthai_ros2 driver (see quest2ros2_setup.md §5 for install):
ros2 run depthai_ros_driver oak_d_launch.py          # topic names in the doc
ros2 topic hz /oak_d/color/image_raw                 # ~30 Hz
ros2 topic echo /oak_d/color/camera_info --once      # intrinsics present?
```

**Wrist webcam:**
```bash
sudo apt install ros-humble-usb-cam
ros2 run usb_cam usb_cam_node --ros-args -p image_transport:=raw
ros2 topic hz /wrist_cam/image_raw
```

**Set frame_ids** (this is what the visualizer depends on):
- OAK-D: `frame_id: front_cam`
- usb_cam: `frame_id: wrist_cam`

**Verify in RViz:**
```bash
ros2 launch laundry_bot visualizer.launch.py
```
→ robot model + TF tree + environment cloud appear; the two Image displays
show the cameras. Move the arm (step 4 commands) and confirm the wrist_cam
frame moves with the model.

**If the arm model is frozen in RViz:** `ros2 run tf2_tools tf2_echo base_link tcp`
— no output means robot_state_publisher isn't getting joint states (check
`ros2 topic hz /joint_states`).

---

## 7. Teleop: Quest 2 (1–2 h)

Follow `docs/quest2ros2_setup.md` (bridge install, `CheckTCPconnection`),
then:

```bash
# terminal 1 — arm + cameras + visualizer:
ros2 launch laundry_bot teleop_record.launch.py use_wrist:=true

# terminal 2 — the bridge side (Quest 2 over WiFi)
```

Put the headset on, press the **lower button** to latch, and move.

**Verify in order:**
1. RViz shows the arm tracking your hand smoothly (no jumps, no vibration).
2. `ros2 topic echo /joint_target` rates at ~30 Hz while moving.
3. The velocity clamp holds: move your hand as fast as possible — the arm
   caps out instead of lurching (`max_speed` per axis in `teleop.yaml`).
4. The workspace clamp: push the target into the table/floor — the arm stops
   at the boundary (watch the RViz TCP marker vs. the env cloud).
5. The singularity guard: try to twist the wrist flat (q5 → 0) — the node
   logs a warning and holds.
6. Drop the laptop to your knee (30 m away) and repeat — if it gets choppy,
   see the latency section of the setup doc.

**If the arm jumps when you latch:** that's the first-frame offset — re-latch
a few times; if persistent, check `pos_map` (you may be mirrored).

---

## 8. First data collection (the real thing)

Before recording, re-read `docs/next_steps.md` §"episode plan". Quick rules:
- one skill per session, same home pose every episode (homing guarantees it)
- 5–10 s of settling before `start`
- failed episode → `record_cli stop --discard`, re-do it
- ~50 episodes per skill is the first fine-tune target

```bash
# terminal 1
ros2 launch laundry_bot teleop_record.launch.py use_wrist:=true dataset_root:=~/datasets

# terminal 2 (per episode)
ros2 run laundry_bot record_cli start "pick shirt from hamper and place on table"
# ...perform the skill...
ros2 run laundry_bot record_cli stop

# sanity check one episode immediately:
python3 -c "
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset('laundry_arm', root='~/datasets/pick_shirt')
print(len(ds), 'frames in episode 0')
print('features:', list(ds.features.keys()))"
```

**Verify:** the episode plays back in LeRobot's viewer
(`python -m lerobot.datasets.viewer --repo-id=...` or the HuggingFace dataset
viewer once pushed) and the wrist image visibly follows the hand.

---

## 9. Fine-tuning on the desktop (7900 XTX)

See `docs/next_steps.md` §"fine-tuning plan" — ROCm install, the
LeRobot/Isaac-GR00T fine-tune path, and the TensorRT export step that runs on
the Jetson (or a rented A100 for a few dollars).

---

## 10. Full pipeline + state machine (after data is flowing)

- classifier node (garment type from the front camera)
- ROS 2 action-server state machine: `grab → classify → (hang|drop)`
- retry logic per skill
- (much later) second arm

---

## Troubleshooting quick table

| Symptom | First check |
|---|---|
| Joint doesn't move | driver enabled? `candump can0` shows the FE frame? CAN id matches `arm.yaml`? |
| Joint moves wrong way | `direction: -1` for that axis |
| Feedback jumps ±large | encoder A/B swapped; or `pulses_per_rev` wrong |
| `/joint_states` missing | driver node crashed → check `ros2 run` terminal; CAN bus-off (`candump` shows `ERROR`) |
| Arm shakes at rest | raise `current_ma` a notch, or check stepper resonance (slow down `max_speed`) |
| RViz model frozen | `ros2 run tf2_tools tf2_echo base_link tcp` |
| Quest teleop choppy | bridge latency (setup doc §7); reduce image rates first (they compete for CPU) |
| Record fails to save episode | disk full? `df -h` — 100 episodes of 2-cam video ≈ 20–40 GB |
