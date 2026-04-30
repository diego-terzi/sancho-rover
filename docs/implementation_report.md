# SANCHO — Implementation Report

This document is a detailed technical record of what has been implemented in the
SANCHO ROS 2 stack, the design decisions behind each component, the values currently
configured, and the test scenarios that validate the work to date. It complements
[`architecture.md`](architecture.md), which describes the *target* system; this file
describes the **as-built state**.

Last updated against commit `ebdbb65` (with subsequent in-flight tuning of HSV bounds,
ROI, and wheel separation reflected below).

---

## 1. Executive summary

| Component | Status | Lines | Source |
|---|---|---|---|
| `camera_node` | ✅ working | 184 | [camera_node.py](../ros2_ws/src/sancho_perception/sancho_perception/camera_node.py) |
| `controller_node` | ✅ working | 191 | [controller_node.py](../ros2_ws/src/sancho_control/sancho_control/controller_node.py) |
| `motor_bridge_node` | ✅ working (dry-run on dev machine) | 213 | [motor_bridge_node.py](../ros2_ws/src/sancho_bridge/sancho_bridge/motor_bridge_node.py) |
| `sim_node` (SIL simulator) | ✅ working | 360+ | [sim_node.py](../ros2_ws/src/sancho_perception/sancho_perception/sim_node.py) |
| `tools/calibrate_hsv.py` | ✅ working | 218 | [calibrate_hsv.py](../tools/calibrate_hsv.py) |
| MCU firmware (IBT-2 rewrite) | ❌ pending | — | [sancho_bridge.ino](../arduino/sancho_bridge/sancho_bridge.ino) (still L298N) |
| `sensor_node` | ❌ pending | — | — |

**Pipeline validated software-in-the-loop**: virtual trail → controller → motor PWM
→ visualised rover motion. The full chain camera → controller → motor_bridge has
also been exercised on the real C270 against a fluorescent test trail, with the
simulator providing the visual feedback in place of physical motors.

---

## 2. ROS 2 workspace layout

The codebase was migrated from a single-package layout to a four-package layout
(commit `a3b6914`). Each package has one architectural responsibility; this is
deliberate so that hardware-specific code (`sancho_bridge`) is isolated from
behaviour code (`sancho_control`) and from perception code (`sancho_perception`).

```
ros2_ws/src/
├── sancho_perception/   camera_node, sim_node
├── sancho_control/      controller_node
├── sancho_bridge/       motor_bridge_node (talks to MCU)
└── sancho_bringup/      sancho_launch.py + sancho_params.yaml
```

The `bringup` package owns:
- the launch file that wires the three runtime nodes together,
- the single shared parameter YAML — every node reads its tunables from here, so
  there is exactly one place to change a value.

---

## 3. Component-by-component implementation

### 3.1 `camera_node` — robust trail detection

**Purpose.** Convert a USB-camera frame into a single normalised number that tells
the controller where the fluorescent trail is in the image plane.

**Topics published**

| Topic | Type | Rate | Meaning |
|---|---|---|---|
| `/trail_error` | `std_msgs/Float32` | 30 Hz | `-1` = trail at far left, `+1` = far right, `NaN` = trail lost |
| `/trail_heading` | `std_msgs/Float32` | 30 Hz | trail line slope (rad). Currently unused; reserved for future feed-forward |

**Algorithm pipeline.**

1. **Capture** at 640×480, 30 Hz.
2. **ROI crop** — keep the bottom `roi_height_percent` of the frame. Configured
   value: `0.85`. With the C270 mounted at 17 cm height and tilted ~53° downward,
   the camera's effective ground coverage is ~5–26 cm in front of the rover; an
   ROI of 0.85 captures the full useful look-ahead without including the
   degenerate top edge of the frame.
3. **HSV mask** with bounds calibrated via [`tools/calibrate_hsv.py`](../tools/calibrate_hsv.py).
   Calibrated values: `lower=[18, 34, 0]`, `upper=[61, 255, 221]`.
4. **Morphological cleanup**:
   - `MORPH_OPEN` (erode → dilate) removes isolated noise pixels (stray paint
     reflections, motion-blur fragments).
   - `MORPH_CLOSE` (dilate → erode) fills small gaps inside the trail blob.
   - Kernel: 5×5 ellipse — small enough to keep edges sharp, large enough to
     close typical 2–3 pixel gaps.
5. **Multi-strip line fit**. The ROI is split into `num_roi_strips=3` horizontal
   bands. In each strip, the largest contour above `min_contour_area=500 px` is
   kept and its centroid recorded. If ≥ 2 strips have valid centroids, a line
   `x = a·y + b` is fitted through them (`np.polyfit`, degree 1). The trail's
   *lateral position* at the bottom of the ROI is `x_bottom = a·H + b`; the
   *heading* is `atan(a)` rad. With only 1 valid strip, we fall back to its
   centroid alone (no heading). With 0, the error is reported as missing.
6. **Normalisation**. `error_raw = clip((x_bottom − W/2) / (W/2), −1, +1)`.
7. **Exponential moving average** on the published error:
   `smoothed = α · raw + (1 − α) · prev`, with `α = 0.3`. This filters
   single-frame glitches (motion blur, glare spikes).
8. **Lost-trail debouncing**. A per-frame counter `consecutive_lost` is
   incremented when no strip detects the trail. For up to
   `lost_trail_patience=5` frames the node *coasts* on the last valid smoothed
   value; only after the patience window is exceeded is `NaN` published. On
   re-acquisition after a confirmed full loss, the EMA snaps directly to the
   new sample to avoid a slow drift back from a stale value.

**Why the multi-strip line fit and not a single ROI centroid?** A single
horizontal band gives only the lateral position; under perspective and pitch
variation this jitters significantly. Three strips at increasing depth give a
poor man's optical-flow reading: their fitted line yields both lateral position
*and* heading angle, and is robust to one strip momentarily losing the trail.

**Calibration tool.** A standalone Python script
[`tools/calibrate_hsv.py`](../tools/calibrate_hsv.py) — no ROS required — opens
the camera, lets the user drag a rectangle over the trail, and auto-computes
the HSV bounds from the 5th–95th percentiles of the sampled pixels with margins
(`±10 H`, `±30 S`, `±40 V`). Trackbars allow fine-tuning. Pressing `s` prints
the values ready to paste into [`sancho_params.yaml`](../ros2_ws/src/sancho_bringup/config/sancho_params.yaml).

---

### 3.2 `controller_node` — PID + three-state machine

**Purpose.** Convert the dimensionless `trail_error` (and, eventually, the
obstacle distance) into a physical velocity command for the rover body.

**Topics**

| Topic | Type | Direction | Rate |
|---|---|---|---|
| `/trail_error` | `std_msgs/Float32` | in | 30 Hz |
| `/scan` | `sensor_msgs/Range` | in | (when `sensor_node` exists) |
| `/cmd_vel` | `geometry_msgs/Twist` | out | **20 Hz, timer-driven** |

**Why timer-driven and not callback-driven.** Every subscriber callback only
*records* the latest value. A 20 Hz timer (`control_rate_hz`) does the actual
work: state decision → PID → publish. Three reasons:

1. The PID's derivative and integral are only meaningful if Δt is constant.
   Reacting to incoming messages couples Δt to the camera rate and breaks the
   gains' physical units.
2. The controller keeps publishing zero Twist even if the camera dies — so
   `motor_bridge_node`'s watchdog sees the stop, and `TRAIL_LOST` detection
   itself works correctly (it depends on time, not on incoming messages).
3. The node decouples the 30 Hz vision rate from the 20 Hz control rate
   cleanly.

**State machine.**

```
                        (no_trail > timeout)
       FOLLOWING ───────────────────────────► TRAIL_LOST
           ▲   │                                   │
           │   │ (obstacle <  threshold)           │  (trail back)
           │   ▼                                   │
           │  OBSTACLE_STOP                        │
           └────────────────(any priority change)──┘
```

- **FOLLOWING**: PID active, `linear.x = base_speed`, `angular.z` = clamped PID
  output.
- **TRAIL_LOST**: no valid (non-NaN) `trail_error` for more than
  `trail_lost_timeout=2.0 s`. Twist is zeroed.
- **OBSTACLE_STOP**: front distance below `obstacle_distance_m=0.3`. Highest
  priority. Twist is zeroed. (Currently inert — no one publishes `/scan` yet.)

**Transition logic.** Order of evaluation in `_next_state()`:
`OBSTACLE_STOP → FOLLOWING → TRAIL_LOST` (first match wins). When `/scan` is
not being published, `last_distance_time is None` and the obstacle check
returns `False`, so the system gracefully degrades to `FOLLOWING ↔ TRAIL_LOST`.

**PID details.** Working on the latest *valid* error:

```
ε(t) = trail_error
I    += ε · Δt                             (Δt = 1/control_rate_hz)
D    = (ε − ε_prev) / Δt
u    = K_p ε + K_i I + K_d D
cmd.angular.z = clamp(−u, −max_angular_z, +max_angular_z)
cmd.linear.x  = base_speed
```

Sign convention (REP-103): `+angular.z` is CCW. A trail error `ε > 0` means
the trail is to the *right* of centre, so the rover must turn right →
`angular.z < 0`. Hence the explicit negation.

**Two PID-implementation gotchas handled**:

1. **Integral windup**: if the rover sat in `TRAIL_LOST` for several seconds,
   the integral term — even though it stops accumulating in this implementation
   — would still hold a value from the previous `FOLLOWING` session, causing a
   wild kick on re-entry. Solution: `integral = 0` *on entering* `FOLLOWING`.
2. **Derivative kick**: if the trail was at `ε = +0.5` when lost and re-appears
   at `ε = −0.3`, a naive derivative is `(−0.3 − 0.5) / 0.05 = −16 rad/s²` of
   instantaneous angular acceleration command. Solution: `prev_error =
   current_error` *on entering* `FOLLOWING`, so the first derivative sample
   after re-acquisition is exactly 0.

**Latest valid-vs-current error.** The controller distinguishes
`last_valid_error` (most recent non-NaN sample) from the current message.
Within `trail_lost_timeout`, NaN bursts are *coasted* on the last valid value,
matching the perception-side coasting in `camera_node.lost_trail_patience`.
Only when no valid sample has arrived for the full timeout window does the
state machine transition to `TRAIL_LOST`.

---

### 3.3 `motor_bridge_node` — kinematics + safety + telemetry

**Purpose.** Translate `/cmd_vel` (m/s, rad/s) into per-track signed PWM and
send to the MCU. Single-point hardware abstraction: this is the only node in
the stack that talks to the MCU.

**Topics**

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | in |
| `/motor_pwm` | `std_msgs/Int16MultiArray` | out (telemetry, `[left, right]`) |

**MCU call.** `bridge.call("set_motors", left, right)` over the Arduino Bridge
RPC. Both arguments are signed integers in `[-255, +255]`.

**Differential-drive kinematics.**

```
v_left  = linear.x − angular.z · d / 2
v_right = linear.x + angular.z · d / 2
```

with `d = wheel_separation = 0.265 m` (measured, centre-to-centre between tracks).
Sign convention as above: `angular.z > 0` ⇒ left turn ⇒ right track faster.

**Velocity-to-PWM mapping.** Defined by:

```
v_max  = (RPM / 60) · π · D
       = (333 / 60) · π · 0.06
       ≈ 1.046 m/s

pwm_k  = clamp( v_k / v_max · max_pwm · scale_k, −max_pwm, +max_pwm )
```

with `RPM = 333` (measured: shaft RPM at PWM 255 with the 5 kg cardboard prototype
under load — *not* the datasheet no-load value; this absorbs gear losses and
loaded-condition slip), `D = 0.06 m` (sprocket diameter), `max_pwm = 255`. Scale
and invert flags are applied per channel:

```
if 0 < |pwm_k| < deadband_pwm:  pwm_k → ±deadband_pwm    # stiction compensation
if invert_k:                     pwm_k → −pwm_k          # wiring correction
```

**Watchdog.** A 20 Hz timer monitors `last_cmd_time`. If `> watchdog_timeout =
0.5 s` has elapsed without a new `/cmd_vel`, the node sends `set_motors(0, 0)`
and logs a warning. This protects against:
- the controller crashing mid-motion,
- ROS-graph disconnects (e.g. a flaky network bridge),
- the launch file partial failure.

**Dry-run mode.** When `arduino_bridge` cannot be imported (development
machines, no MCU connected), the node logs every PWM call instead of executing
it but still publishes telemetry on `/motor_pwm`. This lets the entire pipeline
be tested end-to-end without hardware (see §6, *Test scenarios*).

**Telemetry on `/motor_pwm`.** Every PWM update — including the startup
`(0, 0)`, watchdog stops, and shutdown stops — is published as
`[left_pwm, right_pwm]`. This single line is what makes the SIL simulator able
to drive a virtual rover from the *real* output of the bridge node, validating
the kinematics end-to-end.

**Shutdown safety.** `destroy_node()` always sends `set_motors(0, 0)` followed
by `emergency_stop()` — so even a Ctrl-C leaves the rover stopped.

---

### 3.4 `sim_node` — software-in-the-loop simulator

**Purpose.** Validate the full perception → control → motor-bridge pipeline
without any motors moving. Two distinct modes are offered, selected with the
`publish_trail_error` parameter.

#### 3.4.1 Closed-loop mode (`publish_trail_error: true`)

The node *replaces* `camera_node`. It defines a virtual trail in world
coordinates (a sine wave `y = A · sin(2π·x / λ)` with `A = 1 m, λ = 4 m`),
computes what a real camera would see given the rover's pose, publishes
`/trail_error`, integrates the rover under the resulting `/cmd_vel`, and
visualises everything top-down.

**Trail-error model.** A look-ahead point is projected along the rover's
heading at distance `lookahead_m = 0.4 m`:

```
probe_x = x + lookahead · cos θ
probe_y = y + lookahead · sin θ
```

The trail's `y` at `probe_x` is `y_t = A·sin(2π·probe_x/λ)`. The lateral
offset between the probe and the trail point, rotated into the rover frame
(`y_r = −Δx_w sin θ + Δy_w cos θ`), is normalised against `fov_half_width_m =
0.4 m` to produce the simulated `trail_error`. If `|y_r|` exceeds the FOV the
trail is reported lost (`NaN`), which exercises the controller's
`TRAIL_LOST` branch.

**Pose integration** uses unicycle kinematics:

```
x  += v · cos θ · Δt
y  += v · sin θ · Δt
θ  += ω · Δt          (wrapped to (−π, π])
```

with `(v, ω)` taken either from `/cmd_vel` directly (`motion_source =
'cmd_vel'`) or back-computed from `/motor_pwm` using the inverse of
`motor_bridge_node`'s mapping (`motion_source = 'motor_pwm'`):

```
v_left  = (pwm_l / max_pwm) · v_max
v_right = (pwm_r / max_pwm) · v_max
v       = (v_left + v_right) / 2
ω       = (v_right − v_left) / d
```

This second mode is what makes the SIL test exercise *the actual output of
`motor_bridge_node`*, not just the controller — a wrong sign or kinematics bug
in the bridge would visibly show up as the rover turning the wrong way.

#### 3.4.2 Open-loop mode (`publish_trail_error: false`)

The node **does not** publish `/trail_error`. It only subscribes to `/cmd_vel`
(and optionally `/motor_pwm`) and visualises the resulting motion. This mode
is used when `camera_node` is running on the real C270 and the goal is to see
the live brain output translated to rover motion.

#### 3.4.3 Wheel dashboard

A 300×170 px panel in the lower-right corner draws two wheels as discs with
two perpendicular spokes that rotate at the actual angular velocity of each
track:

```
ω_wheel = v_track / wheel_radius   [rad/s]
```

The PWM value commanded for that track is printed under each wheel. In
`motion_source = 'motor_pwm'` mode the displayed value is the **live PWM from
the topic**; in `'cmd_vel'` mode it shows the *predicted* PWM that
`motor_bridge_node` would produce, allowing the kinematics to be sanity-checked
visually before deploying.

The view is centred on `(rover.x, 0)` in closed-loop mode (so the trail stays
in frame) and on `(rover.x, rover.y)` in open-loop mode (so the rover stays
centred). Press `r` to reset the rover to start.

---

### 3.5 `tools/calibrate_hsv.py`

Stand-alone OpenCV script (no ROS dependency). Opens a camera, lets the user
drag a rectangle over the trail, and auto-fits HSV bounds.

**Sampling algorithm.** For the pixels inside the rectangle:

```
H_lo = max(0,   percentile(H,  5)  − 10)        H_hi = min(180, percentile(H, 95) + 10)
S_lo = max(0,   percentile(S,  5)  − 30)        S_hi = min(255, percentile(S, 95) + 30)
V_lo = max(0,   percentile(V,  5)  − 40)        V_hi = min(255, percentile(V, 95) + 40)
```

The 5th–95th percentile (instead of min/max) rejects outlier pixels —
specular highlights, edge anti-aliasing — that would otherwise blow the
mask up. The asymmetric margins reflect the physics: hue is stable
(±10), saturation drifts moderately with distance and exposure (±30),
and value is the most volatile under motion and shadow (±40).

Two implementation notes from real-world debugging:
- The OpenCV Qt backend on Wayland requires `imshow` to have run at least
  once before `setMouseCallback` will succeed. The callback is now
  registered after the first iteration of the main loop.
- Window titles are kept ASCII-only (`'Camera'` not `'Camera — drag …'`)
  because the Qt backend's name-lookup didn't always handle non-ASCII
  cleanly.

---

## 4. Data flow

```
                     virtual mode             real-camera mode
                     (sim closed-loop)        (sim open-loop)
                     ─────────────────        ─────────────────
                     sim_node                 C270 USB camera
                          │                        │
                          │ /trail_error           │
                          ▼                        ▼
                     ┌────────────┐          camera_node
                     │            │                │
                     │ controller │◄───────────────┘ /trail_error
                     │   _node    │
                     │            │
                     └─────┬──────┘
                           │ /cmd_vel  (Twist, 20 Hz)
                           ▼
                     ┌────────────┐
                     │   motor_   │
                     │  bridge_   │   bridge.call("set_motors", L, R)
                     │   node     │ ─────────────────────────────► MCU (when deployed)
                     └─────┬──────┘
                           │ /motor_pwm  (Int16MultiArray, 20 Hz, telemetry)
                           ▼
                       sim_node (visualisation: wheel dashboard + rover pose)
```

The same set of nodes serves both test setups; only `sim_node`'s parameters
change between scenarios.

---

## 5. Hardware values currently configured

All values live in
[`ros2_ws/src/sancho_bringup/config/sancho_params.yaml`](../ros2_ws/src/sancho_bringup/config/sancho_params.yaml).

### Camera (`camera_node`)

| Param | Value | Source |
|---|---|---|
| `frame_width × frame_height` | 640 × 480 | C270 working resolution |
| `publish_rate_hz` | 30.0 | matches C270 max FPS |
| `hsv_lower` | `[18, 34, 0]` | calibrated via `tools/calibrate_hsv.py` |
| `hsv_upper` | `[61, 255, 221]` | calibrated via `tools/calibrate_hsv.py` |
| `roi_height_percent` | 0.85 | extended for look-ahead — see §5.1 below |
| `num_roi_strips` | 3 | balance: ≥ 2 for line fit, ≤ 4 for compute budget |
| `min_contour_area` | 500 px | rejects noise blobs |
| `morph_kernel_size` | 5 | small enough to preserve trail edges |
| `ema_alpha` | 0.3 | 30 % weight on each new sample |
| `lost_trail_patience` | 5 frames | ≈ 167 ms coast time at 30 Hz |

### Controller (`controller_node`)

| Param | Value | Notes |
|---|---|---|
| `pid_kp` | 1.0 | TODO: tune on rover |
| `pid_ki` | 0.0 | start with pure P controller |
| `pid_kd` | 0.0 | add after Kp is set |
| `base_speed` | 0.3 m/s | ≈ 28 % of `v_max` |
| `trail_lost_timeout` | 2.0 s | |
| `obstacle_distance_m` | 0.3 m | inert until `sensor_node` exists |
| `control_rate_hz` | 20.0 | timer rate, also PID Δt |
| `max_angular_z` | 2.0 rad/s | output clamp |

### Motor bridge (`motor_bridge_node`)

| Param | Value | Source |
|---|---|---|
| `wheel_separation` | 0.265 m | measured, track centre-to-centre |
| `wheel_diameter` | 0.06 m | measured |
| `motor_rpm` | 333.0 | calibrated: shaft RPM at PWM 255 under 5 kg prototype load |
| **derived** `v_max` | ≈ 1.046 m/s | `(333/60)·π·0.06` |
| `max_pwm` | 255 | 8-bit PWM |
| `deadband_pwm` | 0 | TODO: bench-tune (expect 40–80) |
| `invert_left` / `invert_right` | false / false | TODO: confirm on first power-up |
| `left_scale` / `right_scale` | 1.0 / 1.0 | TODO: calibrate if asymmetric |
| `watchdog_timeout` | 0.5 s | software watchdog at the bridge level |

### Chassis & mass (current prototype)

| Quantity | Value | Notes |
|---|---|---|
| Mass (cardboard 1:1 prototype) | ≤ 5 kg | dimensioned spec is 24 kg with the DJI Power 2000; the kinematic & motor sizing was done for the dimensioned spec, not the lighter prototype |
| Track separation `d` | 26.5 cm | |
| Sprocket diameter | 6 cm | |
| Camera mounting height | 17 cm | from ground to lens |
| Camera tilt | ~53° below horizontal | inferred from "trail visible from 5 cm" boundary (see §5.1) |

### 5.1 Camera geometry & look-ahead — derived

The camera (Logitech C270, vertical FOV ≈ 41°, half-FOV ≈ 20.5°) is mounted at
height **h = 17 cm** above the ground and tilted downward such that the
**closest visible point on the ground is 5 cm in front** of the rover.

**Camera tilt below horizontal** (angle of the optical axis):

```
θ_tilt = atan(h / d_near) − FOV_v/2
       = atan(0.17 / 0.05) − 20.5°
       ≈ 73.6° − 20.5°
       ≈ 53.1°  below horizontal
```

**Far visible point** (top of the frame on the ground):

```
d_far = h / tan(θ_tilt − FOV_v/2)
      = 0.17 / tan(32.6°)
      ≈ 0.265 m  (~26.5 cm in front of the rover)
```

**Total visible ground strip:** ~5 cm to ~26 cm in front of the rover —
**roughly 21 cm of usable look-ahead**.

**Implication for control.** With `roi_height_percent = 0.40` (the previous
calibration), the controller would only see ~5 cm to ~13 cm — at
`base_speed = 0.3 m/s` that corresponds to **0.43 s of reaction time**,
far too short for a reactive PID to anticipate curves. The current value
`0.85` extends the controller's effective look-ahead to nearly the full
geometric maximum.

This short look-ahead is also the reason `camera_node` already publishes
`/trail_heading` (the slope of the fitted multi-strip line). Wiring this
into `controller_node` as a feed-forward term is identified as the most
valuable next improvement for curve handling — see §7.4.

---

## 6. Test scenarios validated

### 6.1 Vision alone

- Run `camera_node` with the C270 over the fluorescent test trail.
- Verify the debug windows show the trail isolated cleanly, the green centroids
  are placed on the trail in each strip, and the yellow fitted line follows
  the trail's curvature.
- `ros2 topic echo /trail_error` shows values in `[-1, +1]` that move with the
  trail's lateral position; goes to `NaN` when the camera is covered.

### 6.2 Controller in isolation

- Run `controller_node`. In a second terminal:
  ```bash
  ros2 topic pub -r 30 /trail_error std_msgs/Float32 "data: 0.5"
  ros2 topic echo /cmd_vel
  ```
- Confirm `angular.z` is **negative** (turn right for trail-on-the-right).
- Inject `data: .nan` for > 2 s → state log shows `FOLLOWING -> TRAIL_LOST`,
  `cmd_vel` becomes zero. Inject a valid value → returns to `FOLLOWING`.

### 6.3 Motor bridge in isolation

- Run `motor_bridge_node` (dry-run on dev machine).
- `ros2 topic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"`
- Expected log: `[dry_run] set_motors(L=  +73, R=  +73)`. Verifying: `0.3 / 1.046
  · 255 ≈ 73`. ✓
- `linear.x: 0.0, angular.z: 1.0` → `L ≈ -32, R ≈ +32`. Verifying: `1.0 · 0.265
  / 2 / 1.046 · 255 ≈ 32.3`. ✓
- Stop the publisher → after ~ 0.5 s, log shows `watchdog: no /cmd_vel for X s
  — stopping motors`. ✓

### 6.4 Closed-loop SIL (no hardware at all)

- Terminal 1: `ros2 launch sancho_bringup sancho_launch.py` (camera + controller +
  bridge — but use Scenario A in the README, which stops camera_node and uses
  the simulator instead).
- Terminal 2: `sim_node` with `publish_trail_error:=true motion_source:=motor_pwm`.
- In the sim window: the orange rover triangle starts at `y = +0.3 m` (off the
  trail) and steers itself onto the yellow sine wave; the wheel dashboard shows
  the live PWM differences as it corrects.

### 6.5 Open-loop SIL with the real camera

- `ros2 launch sancho_bringup sancho_launch.py` (default — runs `camera_node`).
- `sim_node` with `publish_trail_error:=false motion_source:=motor_pwm`.
- Move the C270 over the trail: the sim's rover responds in real time to the
  vision-derived commands. The wheel dashboard shows exactly the PWM values
  that *would* be sent to the MCU.

This last scenario is the strongest available validation without hardware:
*every* runtime node is exercised, and only the motors themselves are virtual.

---

## 7. Deployment architecture (live on the rover)

The development sections above describe the software-in-the-loop story. The
following describes how the same code runs on real hardware on the Arduino
UNO Q. The step-by-step operational sequence lives in
[`deployment.md`](deployment.md); this section explains *why* the runtime is
shaped the way it is.

### 7.1 The three-process layout

Once on the rover, three processes co-operate:

```
┌─ ROS 2 container (sancho_rover image) ──────────────────────────────┐
│   camera_node  →  controller_node  →  motor_bridge_node             │
│                                            │                        │
│                                            │ UDP /16-bit L,R/        │
│                                            ▼                        │
└────────────────────────────────────────────┼────────────────────────┘
                                             │ rover1-main-1:9001
                                             │ (rover1_default bridge net)
┌─ App Lab Python container (rover1-main-1) ─┼───────────────────────┐
│                                            ▼                       │
│   socket.recv(...) → Bridge.notify("set_motors", L, R)              │
└────────────────────────────────────────────┼───────────────────────┘
                                             │ /var/run/arduino-router.sock
                                             ▼
┌─ arduino-router (Linux daemon, /dev/ttyHS1 owner) ─────────────────┐
│   MessagePack-RPC over UART, 115200 baud                            │
└────────────────────────────────────────────┼───────────────────────┘
                                             │
                                             ▼
                                       STM32U585 MCU
                                       (sketch.ino)
```

### 7.2 Why the UDP shim exists

The Arduino UNO Q exposes its inter-processor communication through the
`arduino-router` Linux service that owns `/dev/ttyHS1` (115 200 baud, MsgPack
RPC framing). The Python client lives in `arduino.app_utils.Bridge`, which is
**only installed in the per-app virtualenv created by App Lab** under
`/var/lib/arduino-app-cli/<app>/.cache/.venv/`. There is no
system-wide install path and no published PyPI package we could depend on
inside our Docker image.

Two options were considered:

1. **Install `arduino.app_utils` into the ROS 2 Docker image directly.** This
   would require copying the per-app venv contents (and its native deps) into
   the image, and mounting `/var/run/arduino-router.sock` at runtime. Tightly
   couples our build to App Lab internals that aren't part of any stable API,
   and would break the moment Arduino updates the SDK.
2. **Run a small App Lab Python app whose only job is to forward UDP to
   `Bridge.notify`.** Decouples our build from App Lab entirely.

We chose option 2. The shim is 30 lines of code
([`arduino/sancho_bridge/python/main.py`](../arduino/sancho_bridge/python/main.py))
and lets each side speak its native language: the ROS 2 stack ships standard
ROS 2 message types, App Lab ships an App Lab app, neither knows about the
other.

### 7.3 Why `--network rover1_default` (not `--network host`)

App Lab runs each App Lab app inside its own Docker container on a private
bridge network whose name is derived from the app name (`rover1` →
`rover1_default`). The Python listener inside that container sees only its
own loopback (`127.0.0.1` of the bridge network namespace), so a
`--network host` ROS 2 container would not be able to reach it on `127.0.0.1`.

Two fixes:

- **Make the ROS 2 container join `rover1_default`** — what we do.
  Docker's embedded DNS then resolves the App Lab container by name
  (`rover1-main-1`) without hard-coding an IP.
- **Make App Lab's container use `--network host`** — would require editing
  App Lab's auto-generated docker-compose, which we don't own.

The cost of joining the bridge net is that ROS 2 DDS traffic stays inside the
bridge — external machines can no longer `ros2 topic echo` on the rover. For
debugging we `docker exec` into the container instead.

### 7.4 Three-layer motor safety

| Layer | Where | Trigger | Time to stop |
|---|---|---|---|
| 1. Python watchdog | `motor_bridge_node` in Docker | No `/cmd_vel` for `watchdog_timeout` (0.5 s) | within 1 cycle of the 20 Hz watchdog timer |
| 2. MCU watchdog | `sketch.ino` `loop()` | No `set_motors()` for `MOTOR_WATCHDOG_MS` (500 ms) | every iteration of the MCU loop, ~µs |
| 3. PWM = 0 = coast | BTS7960 hardware | Both PWM inputs at zero | instantaneous: half-bridges off |

Layer 1 fires when the controller dies. Layer 2 fires when the entire MPU
side dies (Docker, App Lab, or both). Layer 3 is the natural electrical
behaviour at PWM 0 — no braking, motors simply drift to a stop.

### 7.5 Headless-aware perception

Inside the Docker container there is no X server. Calling `cv2.imshow` would
abort the camera_node process via Qt's xcb plugin (a hard `abort()`, not a
catchable exception). The node detects the missing `DISPLAY` env var during
init and disables debug rendering before the timer fires. On the developer
laptop the env var is set, so the debug windows appear normally.

### 7.6 Pending work

The MCU firmware (originally in this list as the highest-priority task) is
**done** — see [`arduino/sancho_bridge/sketch/sketch.ino`](../arduino/sancho_bridge/sketch/sketch.ino).
Remaining work:

- **`sensor_node`**: a fourth runtime node that listens for Bridge
  notifications from the MCU (HC-SR04 distance, MPU-6050 gyro Z) and
  republishes them as `/scan` and `/imu/data`. Once it exists,
  `controller_node`'s `OBSTACLE_STOP` branch becomes active. Firmware-side
  the sensors have to be re-added to `sketch.ino` (currently motor-only).
- **On-rover motor calibration**: bench-tune `deadband_pwm` (expect 40–80
  for this chassis), verify `invert_left/right`, optionally calibrate
  `left_scale/right_scale` for asymmetric tracks, optionally measure real
  loaded `v_max` and update `motor_rpm` accordingly.
- **Feed-forward on `/trail_heading`** (see §5.1): the camera already
  publishes the trail's slope; wiring it into the PID as
  `u += K_ff · heading` would meaningfully improve curve anticipation given
  the camera's ~21 cm look-ahead.
- **Live parameter updates in `controller_node`**: a
  `set_parameters_callback` would let `ros2 param set` re-tune PID gains
  without restarting the node.
- **Documentation cleanup**: [`docs/architecture.md`](architecture.md) still
  references the original single-package layout and L298N driver.

---

## 8. References

| Source | Path |
|---|---|
| Camera node | [`camera_node.py`](../ros2_ws/src/sancho_perception/sancho_perception/camera_node.py) |
| Controller node | [`controller_node.py`](../ros2_ws/src/sancho_control/sancho_control/controller_node.py) |
| Motor bridge node | [`motor_bridge_node.py`](../ros2_ws/src/sancho_bridge/sancho_bridge/motor_bridge_node.py) |
| Simulator node | [`sim_node.py`](../ros2_ws/src/sancho_perception/sancho_perception/sim_node.py) |
| MCU firmware (BTS7960) | [`sketch.ino`](../arduino/sancho_bridge/sketch/sketch.ino) |
| App Lab UDP→Bridge shim | [`main.py`](../arduino/sancho_bridge/python/main.py) |
| HSV calibration tool | [`calibrate_hsv.py`](../tools/calibrate_hsv.py) |
| Launch file | [`sancho_launch.py`](../ros2_ws/src/sancho_bringup/launch/sancho_launch.py) |
| Parameter YAML | [`sancho_params.yaml`](../ros2_ws/src/sancho_bringup/config/sancho_params.yaml) |
| Dockerfile | [`Dockerfile`](../docker/Dockerfile) |
| Deployment guide | [`deployment.md`](deployment.md) |
| Target architecture | [`architecture.md`](architecture.md) |
| User-facing quickstart | [`README.md`](../README.md) |

---

*End of report.*
