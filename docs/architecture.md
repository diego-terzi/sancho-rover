# SANCHO — Technical Architecture Documentation

---

## 1. PROJECT OVERVIEW

SANCHO (a working title) is a tracked autonomous ground vehicle built as a university prototype at Politecnico di Milano (Design and Robotics programme, 14th edition, 2025-26).

The problem it addresses: in the aftermath of an earthquake, rescue teams often need to operate power-hungry equipment — drones, medical devices, hydraulic tools — in areas that wheeled vehicles or human carriers cannot easily reach. Rescue workers carry fluorescent spray paint and mark a trail on the ground as they advance. SANCHO follows that trail autonomously, carrying battery packs to the front line and freeing the rescue team from logistical burden.

At the highest level, SANCHO is a self-contained mobile platform with:
- A **tracked drivetrain** (differential drive) for moving over rubble and uneven terrain.
- A **camera** pointing at the ground to detect the fluorescent paint trail.
- An **ultrasonic distance sensor** facing forward to detect obstacles.
- An **onboard computer** (Arduino UNO Q) running the decision-making software.
- A **microcontroller** on the same board that drives the motors and reads sensors.

---

## 2. HOW THE ROBOT WORKS (HIGH LEVEL)

### The trail-following concept

A rescue worker walks into a disaster zone and periodically sprays a line of fluorescent paint on the ground. SANCHO's downward-facing camera continuously looks at the ground in front of it. By detecting the bright paint and calculating where it sits in the camera frame — left, right, or center — the robot steers itself to keep the trail centered under the camera.

Think of it like a driver keeping a painted lane marking centered in their field of view: if the line drifts to the right in the image, the driver turns right; if it drifts left, they turn left. SANCHO does the same thing algorithmically, using a PID controller to make the corrections smooth rather than jerky.

If no trail is visible — either because SANCHO has gone too far off-track or reached the end of the painted section — it stops and waits. It will not wander randomly.

### The energy delivery mission

SANCHO carries a payload of battery packs. Its role is purely transport: it follows the trail, reaches the rescue team's position, and waits for a human to unload the batteries and reload it with empty packs for the return trip. There is no autonomous docking, charging, or payload manipulation — these are handled by humans.

### Emergency stop behavior

SANCHO operates near people. If anything comes within approximately 15 cm of its front sensor, it stops immediately. This stop is enforced at the hardware level by the microcontroller — it happens even if the main computer crashes, even if the software is frozen, and even if the power to the computer is interrupted. The motors simply receive no power.

There is a second, software-level stop: the main computer also monitors the distance sensor and commands the motors to stop if an obstacle is detected. This gives two independent layers of protection.

---

## 3. SYSTEM ARCHITECTURE

### The two-processor design of the Arduino UNO Q

SANCHO's main computer is an Arduino UNO Q — a single circuit board that contains two separate processors:

| Processor | Chip | Role |
|---|---|---|
| MPU (Main Processing Unit) | Qualcomm QRB2210 | Runs Linux, Docker, and ROS 2 |
| MCU (Microcontroller Unit) | STM32U585 | Runs the Arduino sketch, drives motors, reads sensors |

This split is intentional and important. The MPU is a powerful application processor that can run a full Linux operating system and complex software, but it is not designed to directly control hardware pins at precise timing. The MCU is a real-time microcontroller optimised for exactly that: generating PWM signals for motors, reading ultrasonic sensors with microsecond precision, and reacting to hardware events with deterministic latency.

The two processors communicate over an internal UART channel using the Arduino Bridge library — no external wiring is needed. From the software's perspective, the MPU calls functions on the MCU and the MCU sends back sensor readings, all through a simple RPC interface.

### ROS 2 and why it is used

ROS 2 (Robot Operating System 2, specifically the Jazzy Jalisco release) is a framework for building robot software. Its core idea is that robot software is made up of independent programs called **nodes**, each with one responsibility, that communicate by passing typed messages over named **topics**.

For SANCHO, ROS 2 provides:
- **Structured message passing** — each piece of data (camera error, velocity command, sensor reading) has a defined type and flows through a named channel. This makes it easy to test individual components in isolation.
- **Parameters** — all tunable values (PID gains, speed limits, detection thresholds) can be set from a YAML file without recompiling the code.
- **Launch system** — a single command starts all four nodes simultaneously and loads their parameters.
- **Ecosystem** — standard message types (Twist, Range, Imu) are already defined and work with existing tools like `ros2 topic echo` for debugging.

### Docker and why it is used only on the robot

Docker is a containerisation tool that packages an application together with all its dependencies into an isolated environment called a container.

On the Arduino UNO Q, the QRB2210 runs **Debian Trixie ARM64**. Debian Trixie has no official pre-compiled ROS 2 Jazzy packages, and compiling ROS 2 from source on that hardware would take several hours. Docker solves this by encapsulating an **Ubuntu 24.04** environment inside the container, where ROS 2 Jazzy packages are available as standard `apt` packages — no compilation from source required. The container image is built once (on a faster machine if needed) and then deployed to the robot.

During development (on a developer's laptop or WSL2 machine running Ubuntu 24.04), Docker is not needed. Developers install ROS 2 Jazzy directly and build the workspace natively. This is faster and allows easier access to debugging tools.

### The Arduino Bridge

The Bridge library is the communication layer between the MPU (Linux/ROS 2) and the MCU (Arduino sketch). It provides two mechanisms:

1. **RPC calls (MPU → MCU)**: The Python code calls a named function on the MCU and waits for it to execute. SANCHO uses this to command motor speeds: `bridge.call("set_motors", left_pwm, right_pwm)`. Each call takes approximately 5–15 ms.

2. **Notifications (MCU → MPU)**: The MCU pushes data to the MPU without being asked. SANCHO uses this to stream sensor readings: every 100 ms, the MCU reads the ultrasonic sensor and IMU and sends the values to Python via `Bridge.notify("sensor_data", msg)`.

The Bridge uses MessagePack serialisation internally — the developer sees Python dictionaries and C++ structs; the binary framing is handled automatically.

### Data flow through the full system

```
USB webcam
    │
    ▼
camera_node  ──(/trail_error, Float32)──►  controller_node
                                                  │
sensor_node  ──(/scan, Range)───────────►         │
                  ▲                               │ /cmd_vel (Twist)
                  │                               ▼
          Bridge notification            motor_bridge_node
                  │                               │
          STM32U585 MCU  ◄────── Bridge RPC ───────┘
                  │
          L298N motor driver
                  │
           Left / Right motors
```

---

## 4. PROJECT STRUCTURE

### Folder tree

```
sancho-rover/
├── CLAUDE.md                          # AI assistant instructions and project rules
├── README.md                          # Quick-start guide for humans
├── .gitignore
│
├── ros2_ws/                           # ROS 2 workspace root
│   └── src/
│       └── sancho_rover/              # ROS 2 package (ament_python)
│           ├── sancho_rover/          # Python package — node source files
│           │   ├── __init__.py
│           │   ├── camera_node.py
│           │   ├── controller_node.py
│           │   ├── motor_bridge_node.py
│           │   └── sensor_node.py
│           ├── launch/
│           │   └── sancho.launch.py   # Starts all four nodes
│           ├── config/
│           │   └── sancho_params.yaml # All tunable parameters
│           ├── resource/
│           │   └── sancho_rover       # ament index marker (required, empty)
│           ├── package.xml            # Package metadata and dependencies
│           └── setup.py               # Python package build config
│
├── arduino/
│   └── sancho_bridge/
│       └── sancho_bridge.ino          # STM32U585 firmware
│
├── docker/
│   └── Dockerfile                     # Deployment image for QRB2210
│
└── docs/
    └── architecture.md                # This file
```

### File-by-file reference

#### `CLAUDE.md`
Project instructions for the Claude Code AI assistant. Contains hardware specs, architectural rules, coding conventions, and a self-update protocol. Not relevant to building or running the code, but governs how AI-assisted development proceeds.

#### `README.md`
Human-readable quick-start: hardware bill of materials, how to build the workspace, how to run the system, how to flash the Arduino sketch, and how to deploy with Docker.

#### `ros2_ws/src/sancho_rover/sancho_rover/camera_node.py`
The visual perception node. Opens the USB webcam, processes frames at 30 FPS, and publishes a normalised horizontal error value. Communicates with nothing except the camera driver (via OpenCV) and the `/trail_error` topic. Does not know that motors exist.

#### `ros2_ws/src/sancho_rover/sancho_rover/controller_node.py`
The brain of the robot. Reads the trail error and obstacle distance, decides what the robot should do, and commands a velocity. Implements the PID controller and the three-state machine. The only node that produces `/cmd_vel`.

#### `ros2_ws/src/sancho_rover/sancho_rover/motor_bridge_node.py`
The hardware interface for the motors. Subscribes to `/cmd_vel`, converts the velocity command into PWM values using differential drive kinematics, and calls `set_motors()` on the MCU via the Arduino Bridge. The only node that calls Bridge RPC functions.

#### `ros2_ws/src/sancho_rover/sancho_rover/sensor_node.py`
The hardware interface for the sensors. Listens for Bridge notifications from the MCU, converts the raw values (cm → meters, deg/s → rad/s), and publishes them as standard ROS 2 messages. Does not filter or interpret sensor data — that is the controller's job.

#### `ros2_ws/src/sancho_rover/launch/sancho.launch.py`
The single entry point for running the full system. Launches all four nodes and loads `sancho_params.yaml` as their shared parameter source. Run with: `ros2 launch sancho_rover sancho.launch.py`.

#### `ros2_ws/src/sancho_rover/config/sancho_params.yaml`
All tunable values in one place, grouped by node. This is the only file that should change during hardware tuning — no source code needs to be edited. All values marked `# TODO` require calibration on the physical robot.

#### `ros2_ws/src/sancho_rover/package.xml`
ROS 2 package manifest. Declares the package name (`sancho_rover`), version, maintainer, and runtime dependencies (`rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `python3-opencv`). Required by the ROS 2 build system.

#### `ros2_ws/src/sancho_rover/setup.py`
Python packaging configuration. Declares the four console entry points (`camera_node`, `controller_node`, `motor_bridge_node`, `sensor_node`) so they can be run with `ros2 run`. Also installs the launch file and config YAML into the package share directory.

#### `ros2_ws/src/sancho_rover/resource/sancho_rover`
An empty file required by the ament build system to register the package in the ROS 2 package index. Do not delete or modify it.

#### `arduino/sancho_bridge/sancho_bridge.ino`
The STM32U585 firmware. Exposes `set_motors()` and `emergency_stop()` to the Bridge, reads the HC-SR04 and MPU-6050 every 100 ms, and sends results as Bridge notifications. Also implements the hardware emergency stop in `loop()`, which runs unconditionally and is independent of ROS 2.

#### `docker/Dockerfile`
Builds a deployable Docker image for the QRB2210 (arm64 architecture). Installs ROS 2 Jazzy, OpenCV, and the workspace. The entry point runs the launch file. Used only for deployment on the physical robot — not needed during development.

---

## 5. NODE REFERENCE

### camera_node

**Purpose**: Visual perception. Detects the fluorescent trail in the camera frame and publishes how far off-center it is.

**Source file**: `ros2_ws/src/sancho_rover/sancho_rover/camera_node.py`

**Inputs**:
- USB webcam at device index `camera_index` (opened via `cv2.VideoCapture`)

**Outputs**:
- `/trail_error` (`std_msgs/Float32`) at ~30 Hz
  - `0.0` — trail is centered
  - `+1.0` — trail is at the far right of the frame
  - `-1.0` — trail is at the far left of the frame
  - `NaN` — no trail detected; never use `0.0` to mean "no trail"

**Processing pipeline**:
1. Capture frame from webcam
2. Crop the lower `crop_fraction` portion of the frame (the ground region)
3. Convert BGR → HSV colour space
4. Apply HSV threshold (`hsv_lower` to `hsv_upper`) to isolate the trail colour
5. Find the largest contour in the resulting binary mask
6. Compute the centroid of that contour
7. Normalise the centroid's horizontal position to [-1, +1]

**Parameters in `sancho_params.yaml`**:

| Parameter | Default | Description |
|---|---|---|
| `camera_index` | `0` | OpenCV device index for the USB webcam |
| `hsv_lower` | `[40, 80, 80]` | Lower HSV bound for trail colour — **must be tuned** |
| `hsv_upper` | `[80, 255, 255]` | Upper HSV bound for trail colour — **must be tuned** |
| `crop_fraction` | `0.5` | Fraction of frame height to use (bottom half) — tune per mounting |

**If this node crashes or loses input**: `/trail_error` stops being published. After `trail_lost_timeout` seconds, `controller_node` transitions to `TRAIL_LOST` and stops the rover.

---

### controller_node

**Purpose**: Decision-making. Converts trail error and obstacle distance into velocity commands. Implements the robot's behaviour logic.

**Source file**: `ros2_ws/src/sancho_rover/sancho_rover/controller_node.py`

**Inputs**:
- `/trail_error` (`std_msgs/Float32`) — from `camera_node`
- `/scan` (`sensor_msgs/Range`) — from `sensor_node`

**Outputs**:
- `/cmd_vel` (`geometry_msgs/Twist`) at 20 Hz
  - `linear.x` — forward speed in m/s
  - `angular.z` — rotation rate in rad/s (negative = turn right)

**State machine**:

```
              trail valid                     obstacle clears
TRAIL_LOST ──────────────► FOLLOWING ◄──────────────────── OBSTACLE_STOP
     ▲                        │  │                                ▲
     │  trail lost > timeout   │  │  obstacle < threshold         │
     └────────────────────────┘  └───────────────────────────────┘
```

- **FOLLOWING**: PID is active. The node computes angular velocity proportional to trail error and publishes a forward Twist.
- **TRAIL_LOST**: No valid trail has been received for more than `trail_lost_timeout` seconds. Publishes zero Twist (stop). Returns to FOLLOWING as soon as a valid (non-NaN) trail error arrives.
- **OBSTACLE_STOP**: The distance from `/scan` is below `obstacle_distance_m`. Publishes zero Twist immediately. Obstacle check has the highest priority — it overrides FOLLOWING. Returns to FOLLOWING when the obstacle clears.

**PID controller**:
The PID output is an angular velocity correction. Given trail error `e`:
```
angular = Kp * e + Ki * ∫e dt + Kd * (de/dt)
cmd.angular.z = -angular   (negative because rightward error → right turn)
```
The integral is reset to zero whenever the rover leaves FOLLOWING state.

**Parameters in `sancho_params.yaml`**:

| Parameter | Default | Description |
|---|---|---|
| `pid_kp` | `1.0` | Proportional gain — **must be tuned** |
| `pid_ki` | `0.0` | Integral gain — **must be tuned** |
| `pid_kd` | `0.0` | Derivative gain — **must be tuned** |
| `trail_lost_timeout` | `2.0` | Seconds without trail before stopping — tune |
| `obstacle_distance_m` | `0.3` | Distance threshold for OBSTACLE_STOP in meters — tune |
| `base_speed` | `0.2` | Forward speed in m/s while FOLLOWING — **must be tuned** |

**If this node crashes**: `/cmd_vel` stops being published. `motor_bridge_node` receives no commands and stops calling `set_motors()`. Motors hold their last state until the MCU-side watchdog (TBD) fires.

---

### motor_bridge_node

**Purpose**: Hardware interface for the motors. Translates ROS 2 velocity commands into L298N PWM values and sends them to the MCU via the Arduino Bridge.

**Source file**: `ros2_ws/src/sancho_rover/sancho_rover/motor_bridge_node.py`

**Inputs**:
- `/cmd_vel` (`geometry_msgs/Twist`) — from `controller_node`

**Outputs**:
- Arduino Bridge RPC call: `set_motors(left_pwm, right_pwm)`
  - Values in range [-255, +255]
  - Called on every received `/cmd_vel` message

**Kinematics**:
```
left_speed  = linear.x  -  angular.z * wheel_separation / 2
right_speed = linear.x  +  angular.z * wheel_separation / 2
left_pwm    = clamp(left_speed  / max_speed * max_pwm,  -255, 255)
right_pwm   = clamp(right_speed / max_speed * max_pwm,  -255, 255)
```

**Parameters in `sancho_params.yaml`**:

| Parameter | Default | Description |
|---|---|---|
| `wheel_separation` | `0.2` | Distance between tracks in meters — **must be measured** |
| `max_speed` | `1.0` | Speed in m/s that maps to PWM 255 — **must be tuned** |
| `max_pwm` | `255` | Maximum PWM value (matches L298N hardware limit) |

**If this node crashes**: No more Bridge calls are made. The MCU retains the last PWM state — the rover may continue moving. A future MCU-side watchdog timer (TBD) will zero the PWMs if no `set_motors()` call arrives within a timeout.

**If the Bridge call fails**: The node logs an error and does not retry. The motors are not updated for that cycle.

---

### sensor_node

**Purpose**: Hardware interface for the sensors. Receives raw MCU data via Bridge notifications and republishes it as standard ROS 2 messages. Does not filter or interpret the data.

**Source file**: `ros2_ws/src/sancho_rover/sancho_rover/sensor_node.py`

**Inputs**:
- Arduino Bridge notification: `sensor_data` (pushed by MCU every 100 ms)
  - `data["distance"]` — HC-SR04 reading in **cm**
  - `data["gyro_z"]` — MPU-6050 Z-axis gyro in **deg/s**

**Outputs**:
- `/scan` (`sensor_msgs/Range`) — distance in **meters** (converted from cm)
- `/imu/data` (`sensor_msgs/Imu`) — `angular_velocity.z` in **rad/s** (converted from deg/s)

**Conversions**:
```
range.range             = data["distance"] / 100.0     # cm → m
imu.angular_velocity.z  = data["gyro_z"] * π / 180.0   # deg/s → rad/s
```

Orientation and linear acceleration covariance are set to -1 (unavailable) because the MPU-6050 integration at this stage only provides gyro data.

**Parameters in `sancho_params.yaml`**: None currently. The Bridge notification rate is fixed by the MCU sketch (100 ms interval) and cannot be changed from the Python side.

**If this node crashes**: `/scan` stops being published. After a short delay, `controller_node` has no fresh distance data. Its `_last_scan_distance` retains the last known value; if that value was above the obstacle threshold the rover continues moving, which is a known risk. A watchdog on the `/scan` subscription is a future improvement.

---

## 6. HOW TO MODIFY THE SYSTEM

### Changing the trail detection logic (camera_node)

All visual processing is in `_timer_callback()` in `camera_node.py`. The most common changes are:

**Changing the trail colour**: Update `hsv_lower` and `hsv_upper` in `sancho_params.yaml`. No code change needed. Use a colour picker tool (e.g. `cv2.inRange` previewed with a test script) to find the HSV range for the actual fluorescent paint.

**Changing the crop region**: Adjust `crop_fraction` in `sancho_params.yaml`. A value of `0.5` means the bottom 50% of the frame is used.

**Changing the detection algorithm**: Modify `_timer_callback()`. The current approach (largest contour centroid) is simple and robust. If false positives become a problem, add a minimum area threshold: `if cv2.contourArea(largest) < MIN_AREA: publish NaN`.

**One rule that must not change**: Always publish `NaN` when no trail is detected — never publish `0.0`. The `controller_node` distinguishes between "trail centered" (`0.0`) and "trail absent" (`NaN`).

### Tuning PID and state machine behaviour (controller_node + sancho_params.yaml)

All PID gains and state machine thresholds are in `sancho_params.yaml` under `controller_node`. Editing the file and restarting the node is sufficient — no recompilation needed. Changes take effect on the next node start (or can be applied at runtime with `ros2 param set`).

Tuning sequence:
1. Set `pid_ki` and `pid_kd` to `0.0`. Increase `pid_kp` until the rover oscillates, then back off.
2. Add `pid_kd` (small value) to reduce oscillation.
3. Add `pid_ki` (very small value) to correct steady-state lateral drift.
4. Adjust `base_speed` to the desired operating speed.
5. Set `obstacle_distance_m` to a safe stopping distance for the environment.

The state machine logic itself is in `_control_loop()` in `controller_node.py`. To add a new state (e.g. `REVERSING`), add it to the `State` enum and add transition conditions to `_control_loop()`.

### Changing motor control and kinematics (motor_bridge_node)

The differential drive kinematics are in `_cmd_vel_callback()` in `motor_bridge_node.py`. The formula assumes a symmetric two-track drive. If the chassis is asymmetric or a different drive model is needed, modify this function.

`wheel_separation` (in `sancho_params.yaml`) is the distance between the left and right tracks in meters. This must be measured from the physical chassis. Until measured, the default value of `0.2 m` is a placeholder.

`max_speed` is the m/s value that corresponds to full PWM (255). This must be determined empirically by measuring how fast the rover travels at full throttle.

### Adding new sensors (sensor_node + Arduino sketch)

**On the MCU side** (`sancho_bridge.ino`): Read the sensor in `loop()` and add the value to the `ArduinoBridgeMessage` before calling `Bridge.notify()`. Example: `msg.put("temperature", readTemp())`.

**On the Python side** (`sensor_node.py`): In `_on_sensor_data()`, read the new key from `data` (e.g. `data["temperature"]`) and publish it to a new topic. Create a new publisher in `__init__()`.

**In the launch and params files**: If the new sensor has tunable parameters, add them to `sancho_params.yaml` under `sensor_node`, declare them with `self.declare_parameter()` in `sensor_node.py`, and add the topic to the architecture documentation.

### Adding a new ROS 2 node

1. Create `ros2_ws/src/sancho_rover/sancho_rover/new_node.py` with a class inheriting from `rclpy.node.Node`.
2. Add a `main()` function and a `console_scripts` entry in `setup.py`:
   ```python
   'new_node = sancho_rover.new_node:main',
   ```
3. Add the node to `launch/sancho.launch.py`.
4. Add parameters to `config/sancho_params.yaml` if needed.
5. Rebuild: `colcon build --packages-select sancho_rover`.

### Files that must always be updated together

| When you change... | Also update... |
|---|---|
| A topic name or type | Every node that publishes or subscribes to it; `docs/architecture.md` |
| A parameter name | `sancho_params.yaml`; the `declare_parameter()` call in the node |
| A Bridge function name | The Arduino sketch (`Bridge.expose()`); the Python node (`bridge.call()`) |
| A Bridge notification field | The Arduino sketch (`msg.put()`); `sensor_node.py` (`data[key]`) |
| A new node | `setup.py` (entry point); `sancho.launch.py`; `sancho_params.yaml` |
| The package name | `package.xml`, `setup.py`, `launch/*.py`, `resource/`, `CLAUDE.md` |

---

## 7. SETUP AND USAGE GUIDE

### Development setup on a new machine

These instructions target Ubuntu 24.04 (native or WSL2 on Windows). ROS 2 Jazzy is the required distribution.

**Step 1 — Install ROS 2 Jazzy**

```bash
# Add ROS 2 apt repository
sudo apt install software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
```

**Step 2 — Install Python dependencies**

```bash
sudo apt install python3-opencv
```

**Step 3 — Clone the repository**

```bash
git clone <repository-url> sancho-rover
cd sancho-rover
```

### Building the workspace

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sancho_rover
source install/setup.bash
```

Add the source line to your shell profile to avoid repeating it:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/sancho-rover/ros2_ws/install/setup.bash" >> ~/.bashrc
```

After any code change, rebuild with `colcon build --packages-select sancho_rover` and re-source.

### Running the full system with the launch file

```bash
ros2 launch sancho_rover sancho.launch.py
```

This starts all four nodes simultaneously, each with parameters loaded from `config/sancho_params.yaml`. Stop with `Ctrl+C`.

### Running a single node for testing

To test `camera_node` in isolation (useful for tuning HSV values):
```bash
ros2 run sancho_rover camera_node
```

Monitor its output:
```bash
ros2 topic echo /trail_error
```

To test `controller_node` without hardware, you can publish fake inputs:
```bash
# Simulate a trail error
ros2 topic pub /trail_error std_msgs/Float32 "data: 0.5"
# Simulate a distance reading
ros2 topic pub /scan sensor_msgs/Range "{range: 1.0}"
# Watch the output
ros2 topic echo /cmd_vel
```

### Tuning parameters without recompiling

Edit `config/sancho_params.yaml` and restart the affected node (or the full launch). No rebuild is needed.

For live tuning during a running session:
```bash
ros2 param set /controller_node pid_kp 1.5
ros2 param set /controller_node base_speed 0.3
```

These changes are not persisted — update `sancho_params.yaml` to make them permanent.

To inspect current parameter values:
```bash
ros2 param list /controller_node
ros2 param get /controller_node pid_kp
```

### Deploying on the Arduino UNO Q using Docker

The Dockerfile is in `docker/`. It builds an arm64 image that includes ROS 2, the workspace, and all dependencies. This step is only performed on the physical robot.

```bash
# Build the image (run on the QRB2210 or cross-compile for arm64)
docker build -f docker/Dockerfile -t sancho_rover .

# Run — pass the camera device and grant hardware access
docker run --device /dev/video0 --privileged sancho_rover
```

The container's entry point runs `ros2 launch sancho_rover sancho.launch.py` automatically.

### Flashing the Arduino sketch

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (version 2.x recommended).
2. Install the Arduino Bridge library via the Library Manager (search "ArduinoBridge").
3. Install the STM32 board support package.
4. Open `arduino/sancho_bridge/sancho_bridge.ino`.
5. Select the correct board (STM32U585 / Arduino UNO Q MCU target) and port.
6. Click Upload.

The sketch does not need to be re-flashed unless motor pin assignments, sensor wiring, or the emergency stop threshold changes. ROS 2 code changes do not require re-flashing.

### Useful debugging commands

```bash
# See all active topics
ros2 topic list

# Watch topic data in real time
ros2 topic echo /trail_error
ros2 topic echo /scan
ros2 topic echo /cmd_vel

# Check topic publish rate
ros2 topic hz /trail_error

# See node graph
ros2 node list
ros2 run rqt_graph rqt_graph

# Check if a node is alive
ros2 node info /camera_node
```
