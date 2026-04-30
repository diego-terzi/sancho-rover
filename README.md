# SANCHO — Tracked UGV for Post-Earthquake Rescue Support

SANCHO is a tracked autonomous ground vehicle that follows a fluorescent trail
left by rescue workers, transporting battery packs to power drones, tools, and
medical devices in areas unreachable by conventional vehicles.

University project — Politecnico di Milano, Design and Robotics, 14th edition,
2025-26.

The full pipeline (camera → controller → motor bridge → MCU → BTS7960 → motors)
is **operational on the rover** as of commit `0593829`.

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Main computer | Arduino UNO Q | QRB2210 MPU (Linux + ROS 2) + STM32U585 MCU |
| Motor driver | **BTS7960 (IBT-2)** | 43 A continuous per channel; L298N rejected (2 A too low) |
| Motors | 2 × JGB37-520, 12 V | 333 RPM measured at the output shaft under load |
| Camera | Logitech C270 | 640×480 @ 30 fps, mounted at ~17 cm height, ~53° down |
| Distance sensor | HC-SR04 ultrasonic | (planned — firmware/sensor_node TBD) |
| IMU | MPU-6050 | (planned — firmware/sensor_node TBD) |
| Power / payload | DJI Power 2000 | ~22 kg; payload delivered to rescue site |
| Drivetrain | Tracked, differential drive | Track separation 26.5 cm, sprocket diameter 6 cm |

---

## Repository structure

```
sancho-rover/
├── ros2_ws/src/                          # ROS 2 workspace, 4 packages
│   ├── sancho_perception/                # camera_node, sim_node
│   ├── sancho_control/                   # controller_node (PID + state machine)
│   ├── sancho_bridge/                    # motor_bridge_node (cmd_vel → UDP → MCU)
│   └── sancho_bringup/                   # launch + sancho_params.yaml
│
├── arduino/sancho_bridge/                # Arduino App Lab "app" (firmware + UDP shim)
│   ├── sketch/sketch.ino                 # MCU firmware: BTS7960 driver + Bridge RPC
│   └── python/main.py                    # UDP listener → Bridge.notify forwarder
│
├── docker/Dockerfile                     # Builds the ROS 2 deployment image
│
├── docs/
│   ├── deployment.md                     # ⭐ step-by-step on-rover deploy guide
│   ├── implementation_report.md          # detailed technical record
│   └── architecture.md                   # original target architecture (partially stale)
│
└── tools/calibrate_hsv.py                # standalone HSV calibration tool (no ROS needed)
```

---

## Current status

| Component | State | Notes |
|---|---|---|
| `camera_node` | ✅ working on rover | Multi-strip ROI line fit, morphology, EMA, lost-trail debouncing, headless-safe |
| `controller_node` | ✅ working on rover | PID + 3-state machine (`FOLLOWING` / `TRAIL_LOST` / `OBSTACLE_STOP`), 20 Hz |
| `motor_bridge_node` | ✅ working on rover | Diff-drive kinematics, 500 ms watchdog, telemetry on `/motor_pwm`, UDP forward |
| `sim_node` | ✅ working | SIL simulator: virtual trail, wheel dashboard, closed/open-loop |
| MCU firmware | ✅ working on rover | BTS7960 driver via `Arduino_RouterBridge`, 500 ms MCU-side watchdog |
| App Lab Python shim | ✅ working on rover | UDP→`Bridge.notify("set_motors", L, R)` |
| `tools/calibrate_hsv.py` | ✅ working | Drag-and-sample HSV bounds in seconds |
| `sensor_node` | ❌ not implemented | Ultrasonic + IMU plumbing to ROS 2 topics — see [implementation_report §7.6](docs/implementation_report.md#76-pending-work) |

---

## Run on the rover (TL;DR)

The detailed deployment story (with prerequisites, troubleshooting, and the
runtime architecture diagram) is in [`docs/deployment.md`](docs/deployment.md).
Short version, on the Arduino UNO Q via SSH:

```bash
# 1. Make sure the App Lab Python container is up
#    (started from Arduino App Lab on a dev PC, "rover1" app)
docker ps                          # rover1-main-1 should be running

# 2. Find the C270 (its index can change between boots)
v4l2-ctl --list-devices

# 3. Run the ROS 2 stack on App Lab's Docker network
docker run --rm -it \
  --device /dev/video0 \
  --network rover1_default \
  sancho_rover:latest
```

Build the Docker image once with `docker build -f docker/Dockerfile -t sancho_rover:latest .`
in the repo root.

---

## Build (development on a laptop)

Ubuntu 22.04 + ROS 2 Humble on the dev machine.

```bash
cd ~/Scrivania/sancho/sancho-rover/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs the two `source` lines before running any node.

---

## Run individual nodes (development)

Each command below assumes the workspace is sourced. All use the shared params YAML.

**Camera (real C270 trail detection):**
```bash
ros2 run sancho_perception camera_node --ros-args \
  --params-file install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml
```
Opens debug windows when DISPLAY is set. Publishes `/trail_error` ∈ `[-1, +1]` or `NaN`.

**Controller (brain):**
```bash
ros2 run sancho_control controller_node --ros-args \
  --params-file install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml
```
Subscribes `/trail_error` + `/scan`. Publishes `/cmd_vel` at 20 Hz.

**Motor bridge (kinematics + UDP forward):**
```bash
ros2 run sancho_bridge motor_bridge_node --ros-args \
  --params-file install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml \
  -p dry_run:=true                 # log PWMs instead of sending UDP
```
Subscribes `/cmd_vel`. Publishes `/motor_pwm` (`std_msgs/Int16MultiArray`, `[L, R]`)
and sends UDP to `udp_target_host:udp_target_port` (default `rover1-main-1:9001`).
Use `dry_run:=true` on a dev machine without an App Lab Python shim listening.

---

## Simulation — three scenarios

`sim_node` is a software-in-the-loop simulator with a top-down view and a wheel
dashboard.

### Scenario A — fully virtual (no camera, no hardware)

Closes the loop `sim → /trail_error → controller → /cmd_vel → motor_bridge → /motor_pwm → sim`.
Best for tuning the PID and validating the kinematics.

```bash
# Terminal 1
ros2 run sancho_control controller_node --ros-args \
  --params-file install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml

# Terminal 2
ros2 run sancho_bridge motor_bridge_node --ros-args \
  --params-file install/sancho_bringup/share/sancho_bringup/config/sancho_params.yaml \
  -p dry_run:=true

# Terminal 3 — sim publishes virtual /trail_error and reads /motor_pwm
ros2 run sancho_perception sim_node --ros-args \
  -p publish_trail_error:=true -p motion_source:=motor_pwm
```

### Scenario B — real camera, virtual motors (laptop)

Vision pipeline runs on the C270, but the rover's motion is only visualised.

```bash
# Terminal 1 — camera + controller + motor_bridge (dry-run)
ros2 launch sancho_bringup sancho_launch.py

# Terminal 2 — sim in open-loop, motion driven by the bridge's /motor_pwm
ros2 run sancho_perception sim_node --ros-args \
  -p publish_trail_error:=false -p motion_source:=motor_pwm
```

### Scenario C — full deployment on the rover

Real camera, real motors. See [`docs/deployment.md`](docs/deployment.md).

---

## Calibration tools

### HSV bounds for the trail

The HSV thresholds in [`sancho_params.yaml`](ros2_ws/src/sancho_bringup/config/sancho_params.yaml)
are sensitive to ambient light. To recalibrate:

```bash
v4l2-ctl --list-devices                  # find the C270's /dev/videoN
python3 tools/calibrate_hsv.py 2         # argument = the index above
```

Drag a rectangle over the trail in the live feed. The tool auto-samples bounds
with margins (5–95 percentile), shows the resulting mask live, and prints values
ready to paste into the YAML when you press `s`.

---

## Parameters

All tunable values live in
[`ros2_ws/src/sancho_bringup/config/sancho_params.yaml`](ros2_ws/src/sancho_bringup/config/sancho_params.yaml)
and are loaded into every node by the launch file.

| Node | Key params |
|---|---|
| `camera_node` | `hsv_lower`, `hsv_upper`, `roi_height_percent`, `num_roi_strips`, `ema_alpha`, `lost_trail_patience` |
| `controller_node` | `pid_kp`, `pid_ki`, `pid_kd`, `base_speed`, `trail_lost_timeout`, `max_angular_z` |
| `motor_bridge_node` | `wheel_separation`, `wheel_diameter`, `motor_rpm`, `deadband_pwm`, `invert_left/right`, `watchdog_timeout`, `udp_target_host/port` |

Changes take effect on the next node start — no rebuild needed.

---

## Topics

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/trail_error` | `std_msgs/Float32` | `camera_node` → `controller_node` |
| `/trail_heading` | `std_msgs/Float32` | `camera_node` → (reserved for feed-forward, see implementation_report §5.1) |
| `/cmd_vel` | `geometry_msgs/Twist` | `controller_node` → `motor_bridge_node` |
| `/motor_pwm` | `std_msgs/Int16MultiArray` | `motor_bridge_node` → `sim_node` (telemetry) |
| `/scan` | `sensor_msgs/Range` | `sensor_node` (not implemented) → `controller_node` |

---

## Documentation

| Document | Audience |
|---|---|
| [`docs/deployment.md`](docs/deployment.md) | Operators: how to flash firmware, build the image, run on the rover, debug |
| [`docs/implementation_report.md`](docs/implementation_report.md) | Engineers and reviewers: per-component design rationale, kinematic formulas, parameter justifications, validated test scenarios, deployment architecture |
| [`docs/architecture.md`](docs/architecture.md) | Original target architecture (partially stale — see implementation_report §7.6) |

---

## What's next

The MCU firmware and App Lab integration are **done**. Remaining work, in
priority order (details in [implementation_report §7.6](docs/implementation_report.md#76-pending-work)):

1. **`sensor_node`** + sensor support in firmware → activates `OBSTACLE_STOP`
2. **On-rover motor calibration**: deadband PWM, invert flags, scale factors
3. **Feed-forward on `/trail_heading`** in the controller (curve anticipation)
4. **Live PID tuning** via `set_parameters_callback` in `controller_node`
5. **Update [`docs/architecture.md`](docs/architecture.md)** to match the
   current 4-package layout, BTS7960 driver, and the UDP-bridge deployment
   architecture
