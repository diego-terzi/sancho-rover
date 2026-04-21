# SANCHO — Tracked UGV for Post-Earthquake Rescue Support

SANCHO is a tracked autonomous ground vehicle that follows a fluorescent trail left by rescue workers,
transporting battery packs to power drones, tools, and medical devices in areas unreachable by conventional vehicles.

University project — Politecnico di Milano, Design and Robotics, 14th edition, 2025-26.

---

## Hardware

| Component | Part |
|---|---|
| Main computer | Arduino UNO Q (QRB2210 MPU + STM32U585 MCU) |
| Motor driver | L298N dual H-bridge |
| Distance sensor | HC-SR04 ultrasonic |
| IMU | MPU-6050 |
| Camera | USB webcam |
| Drivetrain | Tracked, differential drive |

---

## Repository Structure

```
sancho-rover/
├── ros2_ws/src/sancho_rover/   # ROS 2 Python package
│   ├── sancho_rover/           # Node source files
│   ├── launch/                 # Launch files
│   ├── config/                 # Parameter YAML files
│   ├── package.xml
│   └── setup.py
├── arduino/sancho_bridge/      # STM32 sketch (Arduino Bridge)
├── docker/                     # Deployment Dockerfile (QRB2210 only)
└── docs/                       # Architecture documentation
```

---

## Running on Hardware

### 1. Flash the Arduino sketch

Open `arduino/sancho_bridge/sancho_bridge.ino` in the Arduino IDE and upload to the STM32U585 MCU.
Requires the Arduino Bridge library.

### 2. Build the ROS 2 workspace (inside Docker or natively)

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sancho_rover
source install/setup.bash
```

### 3. Launch all nodes

```bash
ros2 launch sancho_rover sancho.launch.py
```

### 4. Deploy with Docker (QRB2210 only)

```bash
docker build -f docker/Dockerfile -t sancho_rover .
docker run --device /dev/video0 --privileged sancho_rover
```

---

## Tuning Parameters

All tunable values are in `ros2_ws/src/sancho_rover/config/sancho_params.yaml`.
Items marked `# TODO` require hardware testing to calibrate.

---

## Architecture

See `docs/architecture.md` for the node/topic diagram and Bridge RPC interface documentation.
