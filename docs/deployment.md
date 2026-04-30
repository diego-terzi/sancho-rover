# SANCHO — Deployment Guide

How to take SANCHO from a clean repository to a moving rover. This document
describes the as-of-today *operational* sequence; the *why* behind each piece
lives in [`implementation_report.md`](implementation_report.md).

---

## 1. The runtime architecture, in one diagram

```
                        ┌─ Arduino UNO Q ──────────────────────────────────────┐
                        │                                                       │
   USB                  │   ┌────────────────────────────────┐                  │
   C270  ──────────►    │   │ ROS 2 container                │                  │
                        │   │ (sancho_rover image)           │                  │
                        │   │                                │                  │
                        │   │   camera_node                  │                  │
                        │   │     │ /trail_error             │                  │
                        │   │     ▼                          │                  │
                        │   │   controller_node              │                  │
                        │   │     │ /cmd_vel                 │                  │
                        │   │     ▼                          │                  │
                        │   │   motor_bridge_node            │                  │
                        │   │     │ UDP [int16 L, int16 R]   │                  │
                        │   │     ▼                          │                  │
                        │   └─────┼──────────────────────────┘                  │
                        │         │ rover1-main-1:9001                          │
                        │         │ (rover1_default Docker network)             │
                        │         ▼                                              │
                        │   ┌────────────────────────────────┐                  │
                        │   │ App Lab Python container       │                  │
                        │   │ (rover1-main-1)                │                  │
                        │   │                                │                  │
                        │   │   UDP listener                 │                  │
                        │   │     │                          │                  │
                        │   │     ▼                          │                  │
                        │   │   Bridge.notify("set_motors")  │                  │
                        │   │     │                          │                  │
                        │   └─────┼──────────────────────────┘                  │
                        │         │ /var/run/arduino-router.sock                │
                        │         ▼                                              │
                        │   ┌────────────────────────────────┐                  │
                        │   │ arduino-router service         │                  │
                        │   │  (Linux daemon)                │                  │
                        │   │                                │                  │
                        │   │   serial 115200 on /dev/ttyHS1 │                  │
                        │   └─────┼──────────────────────────┘                  │
                        │         │                                              │
                        │         │ internal UART                               │
                        │         ▼                                              │
                        │   ┌────────────────────────────────┐                  │
                        │   │ STM32U585 MCU                  │                  │
                        │   │ (sketch/sketch.ino)            │                  │
                        │   │                                │                  │
                        │   │   Bridge.provide_safe(         │                  │
                        │   │     "set_motors", setMotors)   │                  │
                        │   │       │                        │                  │
                        │   │       ▼                        │                  │
                        │   │   PWM out on D5/D6/D9/D10      │                  │
                        │   └─────┼──────────────────────────┘                  │
                        │         │                                              │
                        └─────────┼──────────────────────────────────────────────┘
                                  │
                                  ▼ x 2
                            ┌──────────┐
                            │ BTS7960  │
                            │  driver  │
                            └────┬─────┘
                                 │
                                 ▼
                              motore
```

Three independent processes, three communication hops:
1. ROS 2 container → App Lab container: **UDP datagram** on a private Docker bridge
2. App Lab container → arduino-router: **Unix socket + MessagePack RPC** (`Bridge.notify`)
3. arduino-router → MCU: **UART 115200 baud** on `/dev/ttyHS1`

---

## 2. Prerequisites

You need:
- **Arduino UNO Q** with the BTS7960 modules and motors physically wired (per the pin map in [`arduino/sancho_bridge/sketch/sketch.ino`](../arduino/sancho_bridge/sketch/sketch.ino))
- **Logitech C270** USB camera plugged into the UNO Q
- **A development PC** with **Arduino App Lab** installed (one-time, just for flashing the firmware)
- **SSH access** from the dev PC to the UNO Q (or a keyboard/monitor on the UNO Q itself)

---

## 3. One-time setup

### 3.1 Flash the MCU firmware (from the dev PC)

The firmware lives at `arduino/sancho_bridge/sketch/sketch.ino`. The folder is
already laid out as an Arduino App Lab "app" so you can open it directly.

1. On the dev PC, `git clone https://github.com/diego-terzi/sancho-rover.git`
2. Open Arduino App Lab.
3. **Create a new App** named `rover1` (or any name — but if you change it, the
   container hostname will change too; see §5.4). `My Apps → Create new app`.
4. **Copy our two source files into the new App's directory:**
   - `arduino/sancho_bridge/sketch/sketch.ino` → `<app>/sketch/sketch.ino`
   - `arduino/sancho_bridge/python/main.py` → `<app>/python/main.py`

   (App Lab apps live under `/var/lib/arduino-app-cli/` on Linux or
   `~/ArduinoApps/` depending on the install — App Lab itself shows the path in
   its UI when you create the app.)
5. In App Lab, the **Sketch Library Manager** should auto-detect and install
   `Arduino_RouterBridge`, `MsgPack`, `ArxContainer` etc. on first build.
6. **Click Run.** App Lab will:
   - Compile the sketch and flash the STM32U585 over JTAG (you'll see OpenOCD
     output: "Examination succeed", "halted due to breakpoint")
   - Build a per-app Python venv at `<app>/.cache/.venv/` and install
     `arduino-app-utils` plus deps
   - Start the Python container `rover1-main-1` on Docker network `rover1_default`
   - Bind the UDP listener on `0.0.0.0:9001`
7. Confirmation messages you should see in the App Lab console:
   ```
   [sancho_bridge] UDP listener up on 0.0.0.0:9001, forwarding to MCU via Bridge.notify('set_motors', ...)
   ```
   And in the **Serial Monitor** tab:
   ```
   [sancho_bridge] MCU ready, waiting for set_motors()
   ```

After this step the firmware is permanent on the MCU — it survives reboots.
The App Lab Python container, on the other hand, **must be running every time
you operate the rover** (it's our UDP→Bridge bridge).

### 3.2 Build the ROS 2 Docker image (on the UNO Q)

```bash
ssh arduino@<unoq-ip>
cd ~
git clone https://github.com/diego-terzi/sancho-rover.git
cd sancho-rover
docker build -f docker/Dockerfile -t sancho_rover:latest .
```

First build takes ~5 min (most of the time is `colcon build` of the four
ROS 2 packages). Subsequent builds use Docker layer cache and finish in
seconds when only Python sources change.

---

## 4. Running the rover

Two containers must be up.

### 4.1 Start App Lab (on dev PC, against the UNO Q)

If App Lab can already see the UNO Q (it should, when both are on the same
LAN), **click Run** on the `rover1` app. The Python container `rover1-main-1`
comes up on the UNO Q automatically. Verify with:

```bash
ssh arduino@<unoq-ip>
docker ps         # rover1-main-1 should be in the list, status "Up"
```

### 4.2 Start the ROS 2 stack (on the UNO Q)

```bash
cd ~/sancho-rover
docker run --rm -it \
  --device /dev/video0 \
  --network rover1_default \
  sancho_rover:latest
```

Three flags matter:

| Flag | Why |
|---|---|
| `--device /dev/video0` | Pass the C270 camera into the container. **Verify the index first** with `v4l2-ctl --list-devices` — Linux can re-number USB devices between sessions. The C270 has been seen on `/dev/video0` and `/dev/video2` on the same hardware on different boots. |
| `--network rover1_default` | Join the App Lab Python container's Docker network so we can resolve `rover1-main-1` by name. Note: this **replaces** `--network host`, so external machines can no longer `ros2 topic echo` on this stack — see §5.3 for debug access. |
| `--rm -it` | Auto-remove on exit, keep stdin open for `Ctrl+C` to clean shutdown. |

### 4.3 Expected logs

```
[motor_bridge_node]: motor_bridge_node ready | d=0.265 m, diam=0.060 m, rpm=333.0
                       -> v_max=1.046 m/s | watchdog=0.50 s |
                       udp -> rover1-main-1:9001 | dry_run=False
[controller_node]: Controller started @ 20.0 Hz | Kp=1.0 Ki=0.0 Kd=0.0 |
                       base=0.3 m/s | lost_timeout=2.0 s | obstacle<0.3 m
[camera_node]: No DISPLAY environment variable — running headless,
                       disabling show_debug
[camera_node]: Camera node started: 640x480 @ 30 Hz | strips=3 EMA alpha=0.3 patience=5
```

When the trail enters the camera FOV:
```
[controller_node]: state: TRAIL_LOST -> FOLLOWING
[controller_node]: [FOLLOWING] err=-0.023 cmd.lin=0.30 cmd.ang=+0.023
```

The motors should turn proportionally. `Ctrl+C` to stop.

---

## 5. Operational details and gotchas

### 5.1 Camera index can change between boots

Linux assigns `/dev/videoN` numbers in the order USB devices enumerate. The
internal Qualcomm Venus video decoder claims its share too. We've seen the
C270 on `/dev/video0` (with Venus on 2/3) and on `/dev/video2` (with Venus
on 0/1) on the same hardware. **Always `v4l2-ctl --list-devices` first** and
adjust the `--device` flag accordingly.

If the C270 ends up on something other than `/dev/video0`, remap it:
```
--device /dev/video4:/dev/video0          # forces it to appear as /dev/video0 in the container
```
This way the YAML's `camera_index: 0` keeps working.

### 5.2 HSV bounds are sensitive to lighting

The HSV thresholds in [`sancho_params.yaml`](../ros2_ws/src/sancho_bringup/config/sancho_params.yaml)
were calibrated for a specific lighting condition with a specific trail color.
If the trail isn't being detected (`controller_node` stuck in `TRAIL_LOST`),
re-run the calibration:

```bash
# On a machine with a display (your dev PC, NOT inside Docker on the UNO Q)
python3 tools/calibrate_hsv.py 2     # or whatever index has the C270
```

Drag a rectangle over the trail in the live feed, press `s` to dump the new
values, paste into `sancho_params.yaml`, push, `git pull` on the UNO Q,
`docker build` again.

### 5.3 Debugging from another machine

Because we use `--network rover1_default` (not `--network host`), the ROS 2
DDS traffic is private to the bridge network. To peek at topics:

```bash
# On the UNO Q, attach a debug shell to the running container:
docker exec -it $(docker ps -q --filter "ancestor=sancho_rover:latest") /bin/bash

# Inside the container:
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /motor_pwm
ros2 topic hz /trail_error
```

For visualizing the camera feed, see §5.5.

### 5.4 If you rename the App Lab app

The container name follows the App Lab app name: `<appname>-main-1`. If you
rename `rover1` → `sancho`, the container becomes `sancho-main-1` and the
network becomes `sancho_default`. Update both:

```bash
# In sancho_params.yaml:
udp_target_host: sancho-main-1

# In the docker run command:
--network sancho_default
```

Or override at runtime with `--network <newname>_default` and pass
`-p udp_target_host:=<newname>-main-1` to ROS 2.

### 5.5 Headless Docker — no `cv2.imshow` windows

The ROS 2 container runs without an X server, so OpenCV debug windows would
crash the camera_node (Qt's xcb plugin calls `abort()` on a missing display).
The node detects this and logs:
```
No DISPLAY environment variable — running headless, disabling show_debug
```

Even with debug rendering disabled, you can still see the perception output
on the published topics:
```bash
# Inside an exec'd shell of the ROS 2 container:
ros2 topic echo /trail_error      # the normalised error
ros2 topic echo /trail_heading    # the fitted-line slope (rad)
```

If you really need to see the camera frames, mount your X socket:
```bash
docker run ... \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  ...
```
and re-enable `show_debug: true` in the YAML. Easiest done on a developer
laptop, not on the UNO Q.

### 5.6 The watchdog stops the motors when the controller stalls

There are three independent layers:
1. **motor_bridge_node** (Python in Docker): if no `/cmd_vel` arrives within
   500 ms it sends `(0, 0)` over UDP. Visible in the log as
   `watchdog: no /cmd_vel for X.XXs`.
2. **MCU firmware**: if no `set_motors()` RPC arrives within 500 ms the MCU
   zeroes the PWM regardless of what the MPU is doing.
3. **BTS7960**: PWM = 0 → both half-bridges off → motors coast (no braking).

You will see the rover stop quickly if you Ctrl+C the Docker container, or if
camera_node loses the trail for > `trail_lost_timeout` seconds (controller
goes to TRAIL_LOST → publishes zero Twist → motor_bridge sends 0,0).

---

## 6. The full command list, copy-pasteable

For a clean session from scratch:

```bash
# -- Dev PC --
# (One time only — flash the firmware)
# Open Arduino App Lab, copy sketch.ino + main.py into a new App, click Run.

# -- UNO Q (every operating session) --
ssh arduino@<unoq-ip>

# Make sure App Lab Python container is up
docker ps                # expect rover1-main-1 running

# Verify camera index
v4l2-ctl --list-devices  # find which /dev/videoN is the C270

# (First time on this UNO Q, or after a code change:)
cd ~/sancho-rover
git pull
docker build -f docker/Dockerfile -t sancho_rover:latest .

# Run the stack
docker run --rm -it \
  --device /dev/video0 \
  --network rover1_default \
  sancho_rover:latest

# In another shell, monitor topics:
docker exec -it $(docker ps -q --filter "ancestor=sancho_rover:latest") \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /ros2_ws/install/setup.bash && \
           ros2 topic echo /motor_pwm"
```

---

## 7. Troubleshooting matrix

| Symptom | Likely cause | Fix |
|---|---|---|
| `Webcam 0 not accessible` and node dies | Camera index changed | `v4l2-ctl --list-devices`, update `--device` |
| `controller_node` stuck in `TRAIL_LOST`, never detects | HSV bounds don't match current lighting / trail color | Re-run `tools/calibrate_hsv.py`, update YAML, rebuild |
| Camera detects, controller follows, but motors silent | App Lab Python container down, or wrong network | `docker ps` for `rover1-main-1`; ensure `--network rover1_default` |
| Motors run continuously at full speed after Ctrl+C | (Should never happen — three watchdogs guard against this) | Confirm firmware was flashed correctly; press App Lab "Stop" to fire the App Lab watchdog as a fourth safety |
| `udp -> rover1-main-1:9001` shown but no movement | Mismatch between App Lab app name and `udp_target_host` | Either rename App Lab app to `rover1` or update `udp_target_host` in YAML |
| `process has died, exit code -6` for camera_node | Qt xcb abort on missing DISPLAY (should not happen since the headless guard, but possible if YAML override forces show_debug=true) | Either set `show_debug: false`, or mount X socket — see §5.5 |
| `numpy` import slow (~5 s on first run inside container) | ARM64 numpy is slow on the QRB2210 — known, not a bug | Wait. Subsequent runs use cached imports |

---

## 8. References

- [`README.md`](../README.md) — quick reference and current status
- [`implementation_report.md`](implementation_report.md) — architectural rationale and per-component design notes
- [`architecture.md`](architecture.md) — original target architecture document (some parts now stale; see §7.6 of the implementation report)
- [`arduino/sancho_bridge/sketch/sketch.ino`](../arduino/sancho_bridge/sketch/sketch.ino) — MCU firmware
- [`arduino/sancho_bridge/python/main.py`](../arduino/sancho_bridge/python/main.py) — UDP-to-Bridge shim
- [Arduino App Lab Bridge docs](https://docs.arduino.cc/) — official RPC reference
