# Quest 2 → ROS 2 Teleop Setup (Jetson Orin Nano)

Verified against **Taokt/Quest2ROS2** (github.com/Taokt/Quest2ROS2, Apache-2.0,
HRI 2026, targets ROS 2 **Humble** / Ubuntu 22.04 — exactly the JetPack 6.2 stack).
It uses the Quest2ROS app on the headset (quest2ros.github.io) + a ROS-TCP
endpoint on the robot, no extra PC, no leader arm.

## Architecture

```
Meta Quest 2 (headset)                      Jetson Orin Nano
┌─────────────────────────┐  WiFi (5 GHz)  ┌──────────────────────────────────┐
│ Quest2ROS app           │ ──TCP:10000──> │ ros_tcp_communication (endpoint) │
│  • controller poses     │                │  (guguroro fork, ROS 2 Humble)   │
│  • buttons/grip/trigger │                └────────────────┬─────────────────┘
└─────────────────────────┘                                 │ topics
                                                            ▼
                                              ┌───────────────────────────────┐
                                              │ /q2r_right_hand_pose          │
                                              │   geometry_msgs/PoseStamped   │
                                              │ /q2r_right_hand_inputs        │
                                              │   quest2ros/OVR2ROSInputs     │
                                              │ /q2r_right_hand_twist         │
                                              │   geometry_msgs/Twist         │
                                              └───────────────┬───────────────┘
                                                              ▼
                                              teleop_quest_node (IK, clutch,
                                              safety) -> /joint_target -> arm
```

## Topics & message contract (verified from the repo)

| Topic                  | Type                      | Used for                          |
|------------------------|---------------------------|-----------------------------------|
| `/q2r_right_hand_pose` | `geometry_msgs/PoseStamped` | controller pose (quest world frame: X right, Y forward, Z up) |
| `/q2r_right_hand_inputs` | `quest2ros/OVR2ROSInputs`  | buttons: `button_upper` (touchpad), `button_lower` (lower button), `press_index` (trigger, 0..1), `press_middle` (grip, 0..1), thumbstick x/y |
| `/q2r_right_hand_twist`| `geometry_msgs/Twist`     | velocity — not used (pose mode)   |

Left-hand topics (`/q2r_left_hand_*`) exist too — swap `hand:=left` on the
teleop node if you'd rather drive with the left controller.

**Button convention (Q2R2):** the **lower button** toggles arm motion and, on
latch, performs an *anchor reset* (snaps the virtual target back to the
robot's current position so there's no jump). `teleop_quest_node` implements
the same clutch: `latch_button: button_lower` (default). If you prefer the
trigger: set `latch_button: press_index`. A plain `teleop/latch` (Bool)
topic also latches/unlatches — wire a physical button or a keyboard node to
it if you want a hand-free fallback.

## Install on the Jetson (ROS 2 Humble environment)

```bash
# 1) workspace
mkdir -p ~/quest2ros_ws/src && cd ~/quest2ros_ws/src

# 2) maintained ROS-TCP endpoint (upstream ROS-TCP-Endpoint is not
#    compatible with this Quest2ROS2 version — do not use it)
git clone https://github.com/guguroro/ros_tcp_communication.git

# 3) the quest2ros message package (the app requires the package name
#    'quest2ros' exactly)
git clone https://github.com/Taokt/Quest2ROS2.git
ros2 pkg create --build-type ament_cmake quest2ros
cp -r Quest2ROS2/Files_for_msg_pkg/* quest2ros/

# 4) build everything (including your laundry_bot packages)
cd ~/quest2ros_ws
colcon build
source install/setup.bash
```

Add `source install/setup.bash` to your `.bashrc` (after the ROS 2 source).

## Configure

**Endpoint** — in `src/ros_tcp_communication/launch/endpoint.py`, set the
`ROS_IP` variable to the Jetson's static IP (default is `0.0.0.0`). Give the
Jetson a static IP and keep port 10000 open (no firewall between headset and
Jetson — they must be on the same WiFi LAN; a direct 5 GHz access point with
no internet is fine and gives the lowest latency).

**Quest2ROS app** (on the headset, from the Meta Quest Store /
quest2ros.github.io): enter the Jetson's IP and port `10000`, press Apply.

## Verify before touching the arm

```bash
ros2 launch ros_tcp_endpoint endpoint.py        # terminal 1
ros2 run q2r2_bringup CheckTCPconnection        # terminal 2 (from their repo)
```

Put the headset on, wave the controllers. You should see both hands' pose,
inputs, and twist updating. If it times out after 2 s: IP/port, same subnet,
5 GHz (2.4 GHz is too lossy), or the app not actually connected.

Quick rate check: `ros2 topic hz /q2r_right_hand_pose` (expect 30–120 Hz).

## Frame alignment — the one thing that always needs a physical check

The pose arrives in the **Quest world frame** (origin where you put it, X
right, Y forward, Z up). `teleop_quest_node` rotates it into the robot base
frame with `pos_map` (config/teleop.yaml). The translation is absorbed by the
latch offset and any constant heading error is absorbed by `_yaw_offset` at
latch time, so **you only need pos_map to have the right axis assignment**:

| You stand ...            | pos_map (3x3, row-major)                                   |
|--------------------------|-------------------------------------------------------------|
| BEHIND the arm, facing it (default) | `[[0,1,0],[-1,0,0],[0,0,1]]`  (robot +X = toward you) |
| at the arm's LEFT side, facing its +X | `[[1,0,0],[0,1,0],[0,0,1]]` (identity)              |
| IN FRONT of the arm, facing away | `[[-1,0,0],[0,-1,0],[0,0,1]]`                        |

Rule of thumb: pick the map where **moving your hand toward the arm's +X
makes the TCP go +X**. If motion feels mirrored on any axis, flip the sign of
that row. After changing `pos_map`, restart the teleop node.

In-VR alignment aids: in the Quest2ROS app, pressing **A+B (right) / X+Y
(left)** sets the controller frame alignment to the current controller pose —
use it after each session so your "standing spot" is consistent.

## Running the real stack

```bash
# terminal 1: endpoint
ros2 launch ros_tcp_endpoint endpoint.py

# terminal 2: arm + teleop + recorder (real cameras)
ros2 launch laundry_bot teleop_record.launch.py use_wrist:=true

# terminal 3: episode control
ros2 run laundry_bot record_cli start "pick shirt from incoming basket"
ros2 run laundry_bot record_cli stop            # or: stop --discard
```

First-session checklist:
1. Arm at home pose, homed (`/arm/axis_home`), motors enabled.
2. Put headset on, stand at your chosen spot, A+B to align.
3. `ros2 topic echo /q2r_right_hand_pose --once` — sanity-check pose values.
4. Lower button → LATCH (arm should not move). Move your hand slowly →
   arm follows. Lower button → arm holds.
5. If it's too fast: `pos_gain` 0.6 → 0.3. Too floaty: `pos_smooth` 0.35 → 0.2.
   Jerky near the workspace edge is normal (clamp + IK hold).
6. Record one throwaway episode, `stop --discard`, then start real episodes.

## Latency / tuning notes

- Typical end-to-end (hand → arm) with 5 GHz WiFi + 30 Hz control + low-pass
  is ~100–150 ms. For quasi-static laundry motions that's fine — you're
  driving a ~1.5 rad/s-clamped arm, not doing fast swipes.
- If it feels laggy: (a) move closer to the AP / switch to 5 GHz only,
  (b) raise `pos_smooth` (0.35→0.5) so the filter hides jitter,
  (c) nothing else — the IK/driver side is already <30 ms.
- Keep the headset charged and away from USB-CAN/stepper wiring (RFI into
  CAN is real: if you ever see phantom encoder jumps, separate the CAN cable
  from the arm power loom — it's a wiring fix, not a teleop fix).

## Troubleshooting

| Symptom | Fix |
|---|---|
| No topics after 2 s | IP/port; same subnet; 5 GHz; app shows "connected"? |
| Pose updates, inputs don't | `quest2ros` msg package not built/sourced (inputs use a custom type) |
| Arm jumps when latching | pos_map wrong axis — re-pick from the table above |
| Arm drifts/holds randomly | `ros2 topic hz /q2r_right_hand_pose` — if <20 Hz, WiFi; else check `latch_timeout_s` |
| Wrist singularity warning spam | you're pointing the wrist straight up/down (q5≈0); rotate controller to change wrist pitch — expected protection, not a bug |
| IK residual warnings (holding) | controller asked for a point outside reach; pull hand back. Enlarge workspace in `ur7e_like.yaml` only if it's genuinely in reach. |
