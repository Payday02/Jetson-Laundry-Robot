# laundry_bot

ROS 2 stack for a single-arm laundry robot on a Jetson Orin Nano (JetPack 6.2, ROS 2 Humble, Isaac ROS 3.x).

Pipeline: fixed OAK-D depth camera + wrist webcam → teleop (Quest 2 via Quest2ROS2) →
LeRobot v3.0 dataset collection → fine-tune a VLA (GR00T N1.x / SmolVLA) → deploy on the Jetson.

The arm is UR7e-like, 6 joints, driven by MKS SERVO42D/57D stepper drivers over CAN (SocketCAN, 500 kbps).

## Layout

```
laundry_bot_interfaces/   ROS 2 service definitions (enable, home, estop, episode control)
laundry_bot_pkg/
  laundry_bot/
    kinematics.py         DH FK + Levenberg-Marquardt IK (self-test: python3 kinematics.py <cfg>)
    can_frames.py         byte-exact MKS CAN protocol (ported from the original test GUI)
    arm_driver_node.py    /joint_target (rad) -> CAN abs-pulse cmds; encoders -> /joint_states
    teleop_quest_node.py  Quest2ROS2 controller pose -> IK -> /joint_target (latch clutch)
    fake_teleop_node.py   synthetic teleop trajectory for hardware-free pipeline tests
    fake_camera_node.py   synthetic camera images for hardware-free pipeline tests
    lerobot_recorder_node  LeRobotDataset v3.0 recorder (2 cams + state + action @ 30 Hz)
    visualizer_node.py    camera TF (front static + wrist via FK) + workspace env cloud
    record_cli            terminal episode control: start <task> / stop [--discard]
    estop.py              standalone emergency stop (no ROS required)
  config/                 arm.yaml (axis specs) · ur7e_like.yaml (DH/home/workspace)
                          teleop.yaml · record.yaml
  urdf/                   laundry_arm.urdf.xacro — visual URDF (must match ur7e_like.yaml)
  rviz/                   laundry.rviz — preconfigured RViz2 scene
  launch/                 teleop_record · bringup_virtual · visualizer launch files
docs/quest2ros2_setup.md  Quest 2 bridge install + topic contract + tuning
docs/next_steps.md        status, what's blocked on the arm design, bring-up sequence
docs/build_checklist.md   step-by-step: finished arm → first training data
```

## Build

```bash
mkdir -p ~/ws/src && cp -r laundry_bot_interfaces laundry_bot_pkg ~/ws/src/
cd ~/ws && colcon build --symlink-install
source install/setup.bash
```

## Hardware-free bringup

```bash
ros2 launch laundry_bot bringup_virtual.launch.py dataset_root:=/tmp/ds
ros2 run laundry_bot record_cli start "virtual demo"   # other terminal
ros2 run laundry_bot record_cli stop
```

## Visualizer (RViz2)

Arm model + TF tree + camera frames + a synthetic workspace (table, baskets,
hanger racks) — works with the virtual bringup or with real cameras:

```bash
# standalone (needs arm_driver_node running for joint_states):
ros2 launch laundry_bot visualizer.launch.py

# or attach it to the virtual pipeline:
ros2 launch laundry_bot bringup_virtual.launch.py with_visualizer:=true
```

Point your real camera drivers at frame_ids `front_cam` / `wrist_cam` and
their point clouds/images land in the right place (OAK-D: `frame_id` param;
usb_cam: same). The URDF is a placeholder sized from the UR7e DH table —
keep `urdf/laundry_arm.urdf.xacro` in sync with `config/ur7e_like.yaml`.

## TODO before real hardware

- `config/ur7e_like.yaml` → replace placeholder DH table with measured link lengths
- `config/arm.yaml` → per-axis pulses_per_rev, limits, direction, current
- physical e-stop button wired per `docs/` (estop.py runs standalone)
