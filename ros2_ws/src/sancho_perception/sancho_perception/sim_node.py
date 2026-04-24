"""
sim_node — rover motion visualizer for testing the controller without hardware.

Two modes, selected with the `publish_trail_error` parameter:

  CLOSED LOOP (publish_trail_error: true, the default)
    Replaces camera_node. Defines a virtual sine-wave trail, computes what a
    camera would see, publishes /trail_error, subscribes to /cmd_vel, and
    integrates pose. Useful for validating the PID end-to-end with no hardware.
    DO NOT run camera_node at the same time.

  OPEN LOOP (publish_trail_error: false)
    Does NOT publish /trail_error. Just subscribes to /cmd_vel and integrates
    pose so you can watch the rover's motion driven by a real vision source.
    Use this when camera_node is running on the real camera feed: the full
    pipeline camera -> controller -> sim is exercised visually.

Coordinate frame:
    World: x forward, y left (ROS REP-103). Heading theta=0 => along world +x.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray
from geometry_msgs.msg import Twist

import cv2
import numpy as np


class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')

        # --- parameters ---
        self.declare_parameter('sim_rate_hz', 30.0)
        self.declare_parameter('publish_trail_error', True)   # false = open-loop: use real camera_node instead
        self.declare_parameter('trail_amplitude_m', 1.0)      # sine wave amplitude (closed-loop only)
        self.declare_parameter('trail_wavelength_m', 4.0)     # one full wave every N m (closed-loop only)
        self.declare_parameter('lookahead_m', 0.4)            # virtual camera look-ahead (closed-loop only)
        self.declare_parameter('fov_half_width_m', 0.4)       # lateral half-FOV at look-ahead (closed-loop only)
        self.declare_parameter('canvas_px', 800)
        self.declare_parameter('pixels_per_meter', 80.0)
        self.declare_parameter('start_y_m', 0.3)              # initial lateral offset from trail (closed-loop only)
        self.declare_parameter('start_theta_rad', 0.0)
        self.declare_parameter('lose_trail_outside_fov', True)

        # Motion source: where the sim gets its velocity from.
        #   'cmd_vel'   — integrate Twist directly (default, tests just the controller)
        #   'motor_pwm' — integrate from motor_bridge_node's PWM output
        #                (tests the full brain -> motor_bridge path)
        self.declare_parameter('motion_source', 'cmd_vel')

        # Physical params needed to convert motor_pwm -> wheel velocity -> body motion.
        # These must match motor_bridge_node's values for the loop to behave correctly.
        self.declare_parameter('wheel_separation_m', 0.30)
        self.declare_parameter('wheel_diameter_m',   0.06)
        self.declare_parameter('motor_rpm',          333.0)
        self.declare_parameter('max_pwm',            255)

        self.dt = 1.0 / float(self.get_parameter('sim_rate_hz').value)
        self.publish_error = bool(self.get_parameter('publish_trail_error').value)
        self.motion_source = str(self.get_parameter('motion_source').value).lower()
        if self.motion_source not in ('cmd_vel', 'motor_pwm'):
            self.get_logger().warn(
                f"motion_source='{self.motion_source}' invalid; falling back to 'cmd_vel'"
            )
            self.motion_source = 'cmd_vel'

        self.wheel_sep    = float(self.get_parameter('wheel_separation_m').value)
        self.wheel_diam   = float(self.get_parameter('wheel_diameter_m').value)
        self.wheel_radius = self.wheel_diam / 2.0
        self.motor_rpm_p  = float(self.get_parameter('motor_rpm').value)
        self.max_pwm_p    = int(self.get_parameter('max_pwm').value)
        # v_max used to invert PWM -> m/s (must match motor_bridge_node)
        self.v_max_inv = (self.motor_rpm_p / 60.0) * math.pi * self.wheel_diam
        self.A = float(self.get_parameter('trail_amplitude_m').value)
        self.L = float(self.get_parameter('trail_wavelength_m').value)
        self.lookahead = float(self.get_parameter('lookahead_m').value)
        self.fov_half = float(self.get_parameter('fov_half_width_m').value)
        self.canvas_px = int(self.get_parameter('canvas_px').value)
        self.ppm = float(self.get_parameter('pixels_per_meter').value)
        self.lose_outside_fov = bool(self.get_parameter('lose_trail_outside_fov').value)

        # --- pose state (world frame) ---
        # In open-loop the starting offset is meaningless — just start at origin.
        self.x = 0.0
        self.y = float(self.get_parameter('start_y_m').value) if self.publish_error else 0.0
        self.theta = float(self.get_parameter('start_theta_rad').value)

        # --- velocity state (what actually drives the simulation) ---
        self.v = 0.0   # body linear.x (m/s)
        self.w = 0.0   # body angular.z (rad/s)
        self.v_left = 0.0   # left track velocity (m/s), for wheel animation
        self.v_right = 0.0  # right track velocity (m/s), for wheel animation
        self.last_pwm_left = 0
        self.last_pwm_right = 0
        # For the HUD when motion_source='motor_pwm': track latest raw cmd_vel too
        self._last_cmd_v = 0.0
        self._last_cmd_w = 0.0
        # Wheel rotation angles for the visualisation (radians, modulo 2pi)
        self.wheel_angle_left = 0.0
        self.wheel_angle_right = 0.0
        self.last_cmd_time = self.get_clock().now()

        # --- trajectory history for rendering ---
        self.history = deque(maxlen=1500)
        self.history.append((self.x, self.y))

        # --- I/O ---
        # We always subscribe to /cmd_vel so the HUD can show it even when motion
        # is driven from motor_pwm. Only one source actually updates v/w though.
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        if self.motion_source == 'motor_pwm':
            self.create_subscription(Int16MultiArray, 'motor_pwm', self._on_motor_pwm, 10)
        # Error publisher exists only in closed-loop mode; otherwise the real
        # camera_node owns /trail_error.
        self.error_pub = (
            self.create_publisher(Float32, 'trail_error', 10)
            if self.publish_error else None
        )
        # In open-loop, also listen to /trail_error so we can show it on the HUD.
        self.latest_external_error = float('nan')
        if not self.publish_error:
            self.create_subscription(
                Float32, 'trail_error', self._on_external_error, 10
            )

        # OpenCV window must exist before the timer starts pushing frames
        cv2.namedWindow('SANCHO sim (top-down)', cv2.WINDOW_AUTOSIZE)

        self.timer = self.create_timer(self.dt, self._step)

        mode = 'CLOSED-LOOP (publishes /trail_error)' if self.publish_error \
            else 'OPEN-LOOP (visualization only)'
        self.get_logger().info(
            f'sim_node started @ {1.0/self.dt:.0f} Hz  mode: {mode}  '
            f'motion_source: {self.motion_source}  '
            f'v_max_inv={self.v_max_inv:.3f} m/s  '
            f'start=({self.x:.2f},{self.y:.2f},{math.degrees(self.theta):.1f}°)'
        )

    # ------------------------------------------------------------------ I/O

    def _on_cmd_vel(self, msg: Twist):
        # cmd_vel is always recorded for the HUD, but only drives motion when
        # motion_source == 'cmd_vel'.
        if self.motion_source == 'cmd_vel':
            self.v = float(msg.linear.x)
            self.w = float(msg.angular.z)
            # Back out per-track velocities for wheel animation consistency
            self.v_left = self.v - self.w * self.wheel_sep / 2.0
            self.v_right = self.v + self.w * self.wheel_sep / 2.0
        self._last_cmd_v = float(msg.linear.x)
        self._last_cmd_w = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _on_motor_pwm(self, msg: Int16MultiArray):
        if len(msg.data) < 2:
            return
        pwm_l = int(msg.data[0])
        pwm_r = int(msg.data[1])
        self.last_pwm_left = pwm_l
        self.last_pwm_right = pwm_r
        # Inverse of motor_bridge_node's mapping: pwm -> track velocity
        self.v_left = (pwm_l / self.max_pwm_p) * self.v_max_inv
        self.v_right = (pwm_r / self.max_pwm_p) * self.v_max_inv
        # Diff-drive inverse kinematics: (v_l, v_r) -> (vx, wz)
        self.v = 0.5 * (self.v_left + self.v_right)
        self.w = (self.v_right - self.v_left) / self.wheel_sep
        self.last_cmd_time = self.get_clock().now()

    def _on_external_error(self, msg: Float32):
        self.latest_external_error = float(msg.data)

    # ------------------------------------------------------------------ model

    def _trail_y(self, x: float) -> float:
        return self.A * math.sin(2.0 * math.pi * x / self.L)

    def _compute_trail_error(self):
        """Return normalized trail_error in [-1, +1], or NaN if trail is outside FOV.

        Convention: positive error => trail is to the RIGHT of rover center.
        In the rover frame (x forward, y left), trail on the right => y_r < 0,
        so error = -y_r / fov_half.
        """
        # look-ahead point along rover's forward axis (world frame)
        probe_x = self.x + self.lookahead * math.cos(self.theta)
        probe_y = self.y + self.lookahead * math.sin(self.theta)

        # trail point directly above that x in world
        trail_px = probe_x
        trail_py = self._trail_y(trail_px)

        # vector from probe to trail point, rotated into rover frame
        dx_w = trail_px - probe_x            # 0 by construction
        dy_w = trail_py - probe_y
        # rover-frame: y_r = -dx_w sin(th) + dy_w cos(th)
        y_r = -dx_w * math.sin(self.theta) + dy_w * math.cos(self.theta)

        if self.lose_outside_fov and abs(y_r) > self.fov_half:
            return float('nan')
        return float(max(-1.0, min(1.0, -y_r / self.fov_half)))

    def _integrate_pose(self):
        # Unicycle kinematics (differential drive at center)
        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt
        self.theta += self.w * self.dt
        # normalize theta to (-pi, pi] to avoid unbounded growth
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi
        self.history.append((self.x, self.y))

        # Integrate wheel rotation angles for the visualisation.
        # omega_wheel = v_track / radius
        if self.wheel_radius > 0:
            self.wheel_angle_left = (
                self.wheel_angle_left + (self.v_left / self.wheel_radius) * self.dt
            ) % (2.0 * math.pi)
            self.wheel_angle_right = (
                self.wheel_angle_right + (self.v_right / self.wheel_radius) * self.dt
            ) % (2.0 * math.pi)

    # ------------------------------------------------------------------ main tick

    def _step(self):
        self._integrate_pose()

        if self.publish_error:
            err = self._compute_trail_error()
            msg = Float32()
            msg.data = err
            self.error_pub.publish(msg)
        else:
            # Open-loop: the real camera_node owns /trail_error. Only display it.
            err = self.latest_external_error

        self._render(err)

    # ------------------------------------------------------------------ render

    def _world_to_px(self, x_w: float, y_w: float, origin_x: float, origin_y: float):
        """Convert world coords to pixel coords. View is centered on (origin_x, origin_y).
        World +x -> screen right, world +y -> screen up."""
        cx = self.canvas_px // 2
        cy = self.canvas_px // 2
        u = int(cx + (x_w - origin_x) * self.ppm)
        v = int(cy - (y_w - origin_y) * self.ppm)
        return u, v

    def _render(self, err: float):
        canvas = np.full((self.canvas_px, self.canvas_px, 3), 30, dtype=np.uint8)

        # Closed-loop: center y=0 so the trail stays visible. Open-loop: follow the rover.
        origin_x = self.x
        origin_y = 0.0 if self.publish_error else self.y

        # --- grid (every meter) ---
        for m in range(-10, 11):
            # vertical lines (constant x)
            u, _ = self._world_to_px(origin_x + m, 0, origin_x, origin_y)
            cv2.line(canvas, (u, 0), (u, self.canvas_px), (55, 55, 55), 1)
            # horizontal lines (constant y)
            _, v = self._world_to_px(0, m, origin_x, origin_y)
            cv2.line(canvas, (0, v), (self.canvas_px, v), (55, 55, 55), 1)

        # --- x-axis highlight ---
        _, v0 = self._world_to_px(0, 0, origin_x, origin_y)
        cv2.line(canvas, (0, v0), (self.canvas_px, v0), (90, 90, 90), 1)

        # --- virtual trail (only meaningful in closed-loop) ---
        if self.publish_error:
            x_min = origin_x - self.canvas_px / (2.0 * self.ppm)
            x_max = origin_x + self.canvas_px / (2.0 * self.ppm)
            pts = []
            steps = 300
            for i in range(steps + 1):
                xw = x_min + (x_max - x_min) * i / steps
                yw = self._trail_y(xw)
                pts.append(self._world_to_px(xw, yw, origin_x, origin_y))
            pts_np = np.array(pts, dtype=np.int32)
            cv2.polylines(canvas, [pts_np], False, (0, 220, 220), 3)  # fluorescent yellow

        # --- rover history trail (fading blue) ---
        hist = list(self.history)
        for i in range(1, len(hist)):
            p1 = self._world_to_px(hist[i - 1][0], hist[i - 1][1], origin_x, origin_y)
            p2 = self._world_to_px(hist[i][0], hist[i][1], origin_x, origin_y)
            alpha = i / len(hist)
            color = (int(255 * alpha), int(100 * alpha), int(50 * alpha))
            cv2.line(canvas, p1, p2, color, 2)

        # --- rover body (triangle showing heading) ---
        L_r = 0.25  # rover length (meters, for drawing)
        W_r = 0.18
        # triangle: tip forward, two rear corners
        corners_body = [(L_r, 0.0), (-L_r * 0.6, W_r / 2), (-L_r * 0.6, -W_r / 2)]
        cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        body_px = []
        for (bx, by) in corners_body:
            wx = self.x + cos_t * bx - sin_t * by
            wy = self.y + sin_t * bx + cos_t * by
            body_px.append(self._world_to_px(wx, wy, origin_x, origin_y))
        cv2.fillPoly(canvas, [np.array(body_px, dtype=np.int32)], (80, 180, 255))
        cv2.polylines(canvas, [np.array(body_px, dtype=np.int32)], True, (20, 90, 160), 2)

        # --- look-ahead ray + probe point + FOV bounds (closed-loop only) ---
        if self.publish_error:
            probe_x = self.x + self.lookahead * math.cos(self.theta)
            probe_y = self.y + self.lookahead * math.sin(self.theta)
            p_rover = self._world_to_px(self.x, self.y, origin_x, origin_y)
            p_probe = self._world_to_px(probe_x, probe_y, origin_x, origin_y)
            cv2.line(canvas, p_rover, p_probe, (200, 200, 200), 1)
            cv2.circle(canvas, p_probe, 4, (255, 255, 255), -1)

            nx, ny = -math.sin(self.theta), math.cos(self.theta)  # left normal
            left_w = (probe_x + self.fov_half * nx, probe_y + self.fov_half * ny)
            right_w = (probe_x - self.fov_half * nx, probe_y - self.fov_half * ny)
            cv2.line(canvas,
                     self._world_to_px(left_w[0], left_w[1], origin_x, origin_y),
                     self._world_to_px(right_w[0], right_w[1], origin_x, origin_y),
                     (120, 120, 120), 1)

        # --- wheel dashboard (lower-right corner) ---
        self._draw_wheel_panel(canvas)

        # --- HUD ---
        mode_tag = 'closed-loop' if self.publish_error else 'open-loop (from camera)'
        err_line = (
            f"error {err:+.3f}  [{mode_tag}]"
            if not math.isnan(err)
            else f"error  NaN  [{mode_tag}]"
        )
        txt = [
            f"pose  x={self.x:+.2f}m y={self.y:+.2f}m th={math.degrees(self.theta):+.1f}°",
            f"cmd   v={self.v:+.3f} m/s   w={self.w:+.3f} rad/s",
            err_line,
        ]
        for i, line in enumerate(txt):
            cv2.putText(canvas, line, (10, 24 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)

        cv2.imshow('SANCHO sim (top-down)', canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            # press 'r' in the window to reset the rover to start
            self.x = 0.0
            self.y = float(self.get_parameter('start_y_m').value)
            self.theta = float(self.get_parameter('start_theta_rad').value)
            self.history.clear()
            self.history.append((self.x, self.y))
            self.get_logger().info('rover reset')

    def _draw_wheel_panel(self, canvas):
        """Draw the 'motor dashboard' in the lower-right corner: two wheels
        rotating at the commanded angular velocity, plus their PWM values."""
        panel_w = 300
        panel_h = 170
        x0 = self.canvas_px - panel_w - 10
        y0 = self.canvas_px - panel_h - 10
        # panel background
        cv2.rectangle(canvas, (x0, y0), (x0 + panel_w, y0 + panel_h), (45, 45, 45), -1)
        cv2.rectangle(canvas, (x0, y0), (x0 + panel_w, y0 + panel_h), (120, 120, 120), 1)

        title = 'motor_pwm (live)' if self.motion_source == 'motor_pwm' else 'predicted PWM (from cmd_vel)'
        cv2.putText(canvas, title, (x0 + 10, y0 + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)

        # Choose values to display: actual PWMs if we're reading them, otherwise
        # what motor_bridge_node *would* compute from cmd_vel.
        if self.motion_source == 'motor_pwm':
            pwm_l_disp = self.last_pwm_left
            pwm_r_disp = self.last_pwm_right
        else:
            pwm_l_disp = self._predict_pwm(self.v_left)
            pwm_r_disp = self._predict_pwm(self.v_right)

        wheel_r = 45
        cy = y0 + 88
        left_cx = x0 + 75
        right_cx = x0 + panel_w - 75

        for (cx, angle, pwm, label) in [
            (left_cx,  self.wheel_angle_left,  pwm_l_disp, 'L'),
            (right_cx, self.wheel_angle_right, pwm_r_disp, 'R'),
        ]:
            cv2.circle(canvas, (cx, cy), wheel_r, (70, 70, 70), -1)
            cv2.circle(canvas, (cx, cy), wheel_r, (180, 180, 180), 2)
            # two perpendicular spokes to make rotation easy to see
            for k in (0.0, math.pi / 2):
                ex = int(cx + wheel_r * math.cos(angle + k))
                ey = int(cy - wheel_r * math.sin(angle + k))
                cv2.line(canvas, (cx, cy), (ex, ey),
                         (0, 255, 255) if k == 0.0 else (0, 180, 180),
                         3 if k == 0.0 else 1)
            cv2.circle(canvas, (cx, cy), 4, (30, 30, 30), -1)
            cv2.putText(canvas, label, (cx - 8, y0 + 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2)
            cv2.putText(canvas, f'{pwm:+d}',
                        (cx - 28, cy + wheel_r + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 2)

    def _predict_pwm(self, v_track: float) -> int:
        if self.v_max_inv <= 0:
            return 0
        pwm = int(round((v_track / self.v_max_inv) * self.max_pwm_p))
        return max(-self.max_pwm_p, min(self.max_pwm_p, pwm))

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
