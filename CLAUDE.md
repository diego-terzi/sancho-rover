# SANCHO — Claude Code Project Instructions

## Project Overview

SANCHO is a tracked UGV (Unmanned Ground Vehicle) designed for post-earthquake rescue support.
It follows a fluorescent trail left by a rescuer on the ground, autonomously transporting
battery packs to power drones, tools, and medical devices in areas unreachable by conventional vehicles.

This is a university project (Politecnico di Milano, Design and Robotics, 14th edition, 2025-26).
The codebase must remain simple, testable, and prototype-feasible.

---

## Repository Structure

```
sancho-rover/
├── CLAUDE.md
├── README.md
├── .gitignore
├── ros2_ws/
│   └── src/
│       └── sancho_rover/        # TODO (resolved): package name is sancho_rover
│           ├── sancho_rover/    # Python nodes go here
│           │   ├── __init__.py
│           │   ├── camera_node.py
│           │   ├── controller_node.py
│           │   ├── motor_bridge_node.py
│           │   └── sensor_node.py
│           ├── launch/
│           │   └── sancho.launch.py
│           ├── config/
│           │   └── sancho_params.yaml
│           ├── resource/
│           │   └── sancho_rover
│           ├── package.xml
│           └── setup.py
├── arduino/
│   └── sancho_bridge/
│       └── sancho_bridge.ino
├── docker/
│   └── Dockerfile
└── docs/
    └── architecture.md
```
# LEARNED: 2026-04-21 — Package name decided: sancho_rover (snake_case of repo name)

---

## Hardware Platform

- **Main computer**: Arduino UNO Q (Qualcomm QRB2210 MPU + STM32U585 MCU)
- **Motor driver**: L298N dual H-bridge
- **Distance sensor**: HC-SR04 ultrasonic
- **IMU**: MPU-6050
- **Camera**: USB webcam
- **Drivetrain**: tracked, differential drive
- **Motors**: DC motors, left and right symmetric — specific model TBD

---

## Software Architecture

ROS 2 Jazzy Jalisco running inside a Docker container on the QRB2210 (Debian Linux).
The STM32U585 MCU runs an Arduino sketch independently of ROS 2.
Communication between the Docker container and the MCU uses the Arduino Bridge library (RPC over internal UART).

### Nodes and Topics

| Node | Subscribes | Publishes | Notes |
|---|---|---|---|
| `camera_node` | — | `/trail_error` | Float32, range [-1.0, +1.0], NaN if trail not found |
| `controller_node` | `/trail_error`, `/scan` | `/cmd_vel` | PID + state machine |
| `motor_bridge_node` | `/cmd_vel` | — | Calls `set_motors()` on MCU via Bridge RPC |
| `sensor_node` | Bridge notifications | `/scan` (`sensor_msgs/Range`), `/imu/data` | Converts raw MCU data to ROS 2 standard messages |

### State Machine (controller_node)

- `FOLLOWING` — trail detected, PID active
- `TRAIL_LOST` — no trail for configurable timeout, rover stops
- `OBSTACLE_STOP` — obstacle within safety threshold, rover stops immediately

### Bridge RPC Interface (MCU side)

- `set_motors(int left, int right)` — PWM values in range [-255, +255]
- `emergency_stop()` — zeros both PWM outputs immediately
- MCU sends Bridge notifications every 100ms: HC-SR04 distance + MPU-6050 gyro data

---

## Coding Conventions

- Language: Python (ROS 2 nodes), C++ / Arduino sketch (MCU)
- All variable names and comments in **English**
- Each node in its own file, one responsibility per node
- No hardcoded values — use ROS 2 parameters or clearly marked constants at top of file
- Mark all placeholder values with a comment: `# TODO: tune this value`

---

## What NOT to do

- Do not add features not described in this file without asking
- Do not merge ROS 2 logic into the Arduino sketch
- Do not put safety-critical logic (emergency stop) inside ROS 2 — it lives on the MCU
- Do not use external AI/ML libraries (no PyTorch, no TensorFlow) — OpenCV only
- Do not overcomplicate: this is a student prototype, not a production system

---

## Current Project Phase

Phase 3 (Develop) is in progress. Code development is underway.
Many parameters and hardware decisions are still TBD — see `# TODO` comments throughout.
When in doubt, implement the simpler option and leave a clear TODO comment.

## Claude Code — Self-Update Rules

### When to update these files autonomously
Claude must update CLAUDE.md and the relevant SKILL.md files when:
- A technical or architectural decision changes (e.g. package name decided, motor specs confirmed, HSV color chosen)
- A bug is solved that reveals a wrong assumption in the current documentation
- A new pattern or constraint emerges during implementation that should be remembered
- A TODO item gets resolved

### How to update
- Update CLAUDE.md for project-wide changes (hardware, architecture, conventions)
- Update the relevant SKILL.md for domain-specific changes (e.g. Bridge patterns, safety rules)
- Add a `# LEARNED: <date> — <short description>` comment near the updated section
- Never delete old TODOs — mark them as resolved: `# TODO (resolved): was X, now Y`

### Error and solution log
When a bug is found and fixed, append it here:

#### Log
<!-- Claude appends entries here automatically -->