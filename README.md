# SANCHO — Tracked UGV for Post-Earthquake Rescue Support

SANCHO is a tracked autonomous ground vehicle that follows a fluorescent trail left by rescue
workers, transporting battery packs to power drones, tools, and medical devices in areas
unreachable by conventional vehicles.

University project — Politecnico di Milano, Design and Robotics, 14th edition, 2025-26.

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Main computer | Arduino UNO Q | QRB2210 MPU (Linux + ROS 2) + STM32U585 MCU |
| Motor driver | **IBT-2 (BTS7960)** | 43 A continuous per channel; L298N rejected (2 A too low) |
| Motors | 2 × JGB37-520, 12 V | ~333 RPM output shaft |
| Camera | Logitech C270 | 640×480 @ 30 fps, UVC, fixed focus |
| Distance sensor | HC-SR04 ultrasonic | (firmware — not yet wired into ROS 2) |
| IMU | MPU-6050 | (firmware — not yet wired into ROS 2) |
| Power / payload | DJI Power 2000 | ~22 kg; payload delivered to rescue site |
| Drivetrain | Tracked, differential drive | Wheel separation 0.30 m, wheel diameter 0.06 m |

---

## Repository structure

```
sancho-rover/
├── ros2_ws/src/
│   ├── sancho_perception/   # camera_node, sim_node (visual input / simulator)
│   ├── sancho_control/      # controller_node (PID + state machine)
│   ├── sancho_bridge/       # motor_bridge_node (cmd_vel → motor PWM over Arduino Bridge)
│   └── sancho_bringup/      # launch file + sancho_params.yaml for the full stack
│
├── arduino/sancho_bridge/   # STM32U585 firmware (Arduino Bridge RPC)
├── docker/                  # Dockerfile for deployment on the QRB2210
├── docs/architecture.md     # full node / topic / Bridge reference
└── tools/                   # stand-alone dev utilities (no ROS needed)
    └── calibrate_hsv.py     # HSV calibration tool for the trail colour
```

---

## Current status

| Component | State | Notes |
|---|---|---|
| `camera_node` | ✅ working | Multi-strip ROI line fit, morphological cleanup, EMA smoothing, lost-trail debouncing |
| `controller_node` | ✅ working | PID + 3-state machine (`FOLLOWING` / `TRAIL_LOST` / `OBSTACLE_STOP`). Publishes `/cmd_vel` at 20 Hz |
| `motor_bridge_node` | ✅ working (dry-run) | Diff-drive kinematics, watchdog, publishes `/motor_pwm` telemetry. Bridge RPC call stubbed until deployed |
| `sim_node` | ✅ working | Software-in-the-loop simulator with virtual trail, wheel dashboard, closed- and open-loop modes |
| `tools/calibrate_hsv.py` | ✅ working | Drag over the trail to auto-sample HSV bounds |
| MCU firmware | ⚠️ stale | Still written for the L298N — needs IBT-2 rewrite |
| `sensor_node` | ❌ not implemented | Ultrasonic + IMU plumbing from MCU to ROS 2 topics |

---

## Build

Ubuntu 22.04 + ROS 2 Humble on the dev machine (Jazzy on the rover).

```bash
cd ~/Scrivania/sancho/sancho-rover/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs the two `source` lines before running any node.

---

## Run — the full stack in one command

```bash
ros2 launch sancho_bringup sancho_launch.py
```

This starts `camera_node`, `controller_node`, and `motor_bridge_node` with shared parameters
from `install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml`. On a dev machine,
`motor_bridge_node` auto-detects that the Arduino Bridge library is absent and runs in
`dry_run` mode (logs PWMs instead of sending them to a MCU).

---

## Run — individual nodes

Each command below assumes the workspace is sourced. All use the shared params YAML so their
settings match.

**Camera (real C270 trail detection):**
```bash
ros2 run sancho_perception camera_node --ros-args \
  --params-file install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml
```
Opens two debug windows (ROI + mask). Publishes `/trail_error` ∈ `[-1, +1]` or `NaN`.

**Controller (brain):**
```bash
ros2 run sancho_control controller_node --ros-args \
  --params-file install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml
```
Subscribes `/trail_error` + `/scan`. Publishes `/cmd_vel` (`geometry_msgs/Twist`) at 20 Hz.

**Motor bridge (kinematics + MCU RPC):**
```bash
ros2 run sancho_bridge motor_bridge_node --ros-args \
  --params-file install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml
```
Subscribes `/cmd_vel`. Publishes `/motor_pwm` (`std_msgs/Int16MultiArray`, `[left, right]`).
Calls `set_motors(left, right)` on the MCU via Arduino Bridge when deployed.

---

## Simulation — two scenarios

`sim_node` is a software-in-the-loop simulator with a top-down view and a wheel dashboard.

### Scenario A — fully virtual (no camera, no hardware)

Closes the loop `sim (virtual trail) → controller → motor_bridge → sim (motion)`.
Best for tuning the PID and validating `motor_bridge_node`'s kinematics.

```bash
# Terminal 1 — controller + motor_bridge (via launch file)
ros2 launch sancho_bringup sancho_launch.py

# Terminal 2 — simulator (closed-loop, driven by motor_pwm telemetry)
ros2 run sancho_perception sim_node --ros-args \
  -p publish_trail_error:=true \
  -p motion_source:=motor_pwm
```
**Caveat**: the launch file also starts `camera_node`. If both `sim_node` (closed-loop) and
`camera_node` are running, they'll both publish `/trail_error` and conflict. Either stop
`camera_node` or launch only the control+bridge subset.

### Scenario B — real camera, virtual motors

Runs the actual vision pipeline on the C270 and visualises what the rover *would* do.
Best for validating the camera → brain → motor_bridge path end-to-end without moving any motor.

```bash
# Terminal 1 — full stack (camera + controller + motor_bridge)
ros2 launch sancho_bringup sancho_launch.py

# Terminal 2 — simulator (open-loop, just visualises)
ros2 run sancho_perception sim_node --ros-args \
  -p publish_trail_error:=false \
  -p motion_source:=motor_pwm
```

Wheel dashboard in the sim window shows the live PWM values going to `set_motors()`.

---

## Calibration tools

### HSV calibration for the trail

```bash
python3 tools/calibrate_hsv.py 2    # argument = camera index (C270 is usually /dev/video2)
```

Drag a rectangle over the trail in the live feed. The tool auto-samples HSV bounds from
those pixels, updates the mask live, and lets you fine-tune with trackbars. Press `s` to
print the values ready to paste into `sancho_params.yaml`.

Find the right camera index with:
```bash
v4l2-ctl --list-devices
```

---

## Parameters

All tunable values live in
[`ros2_ws/src/sancho_bringup/config/sancho_params.yaml`](ros2_ws/src/sancho_bringup/config/sancho_params.yaml)
and are loaded into every node by the launch file.

| Node | Key params |
|---|---|
| `camera_node` | `hsv_lower`, `hsv_upper`, `roi_height_percent`, `num_roi_strips`, `ema_alpha`, `lost_trail_patience` |
| `controller_node` | `pid_kp`, `pid_ki`, `pid_kd`, `base_speed`, `trail_lost_timeout`, `max_angular_z` |
| `motor_bridge_node` | `wheel_separation`, `wheel_diameter`, `motor_rpm`, `deadband_pwm`, `invert_left`, `invert_right`, `watchdog_timeout` |

Changes take effect on the next node start — no rebuild needed.

---

## Topics

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/trail_error` | `std_msgs/Float32` | `camera_node` → `controller_node` |
| `/trail_heading` | `std_msgs/Float32` | `camera_node` → (unused for now; future feedforward) |
| `/cmd_vel` | `geometry_msgs/Twist` | `controller_node` → `motor_bridge_node` |
| `/motor_pwm` | `std_msgs/Int16MultiArray` | `motor_bridge_node` → `sim_node` (telemetry) |
| `/scan` | `sensor_msgs/Range` | `sensor_node` (not implemented) → `controller_node` |

---

## What's next

### 1. Rewrite the MCU firmware for the IBT-2 (BTS7960)

The sketch in [`arduino/sancho_bridge/sancho_bridge.ino`](arduino/sancho_bridge/sancho_bridge.ino)
still drives an L298N (`IN1/IN2` direction pins + `EN` PWM). The IBT-2 needs a different
control interface — two PWM pins per channel (`RPWM` for forward, `LPWM` for reverse) plus
two enable pins (`R_EN`, `L_EN`).

**To do this, I need:**
- MCU pin numbers for **left** IBT-2: `L_RPWM`, `L_LPWM`, `L_R_EN`, `L_L_EN`
- MCU pin numbers for **right** IBT-2: `R_RPWM`, `R_LPWM`, `R_R_EN`, `R_L_EN`
- Whether `R_EN` / `L_EN` are tied together to one MCU pin (simplifies firmware)

The ROS 2 side (`motor_bridge_node`) does **not** change — the Bridge API
`set_motors(left, right)` stays identical.

### 2. Implement `sensor_node`

A new node in `sancho_bridge/` that receives Bridge notifications from the MCU (HC-SR04
distance + MPU-6050 gyro) and republishes them as `/scan` (`sensor_msgs/Range`) and
`/imu/data` (`sensor_msgs/Imu`). Once this exists, `controller_node`'s `OBSTACLE_STOP`
state becomes active.

**To do this, I need:**
- Confirmation the HC-SR04 and MPU-6050 are still the intended sensors (or which replaced them)
- Any pin changes from the current firmware defaults

### 3. On-bench motor calibration

Once firmware is running: bench-test deadband PWM, verify direction inverts, measure real
`v_max`, and update `sancho_params.yaml` accordingly.

### 4. Docs update

`docs/architecture.md` still refers to the old single-package layout (`sancho_rover`) and
L298N. Needs a pass to match the 4-package structure and IBT-2 decision.

---

## Architecture

Full node / topic / Bridge RPC reference in [`docs/architecture.md`](docs/architecture.md).
(Note: that file is partially outdated — see "What's next" §4.)
