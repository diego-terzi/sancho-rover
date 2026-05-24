"""
Follow controller node: PID controller for human-follow mode.

State machine:
    MARKER_LOST  No valid marker for > marker_lost_timeout seconds.
    FOLLOWING    PID on lateral error + longitudinal distance error.
    TOO_CLOSE    Distance < min_safe_distance_m: stop (back off not allowed).
    WAITING      Stable at target for waiting_frames frames: stop and wait.

Control law (FOLLOWING only):
    angular.z = clamp(-( Kp_lat*err_lat + Ki_lat*∫err_lat + Kd_lat*derr_lat ), ±max_angular_z)
    linear.x  = clamp(   Kp_lon*err_lon + Ki_lon*∫err_lon + Kd_lon*derr_lon,   0, max_linear_speed)
    err_lon   = current_distance - target_distance_m  (positive = too far → drive forward)
    Integrals and prev-errors reset to zero on every re-entry to FOLLOWING.

Publishes /follow_cmd_vel (NOT /cmd_vel). The mode_manager muxes it onto /cmd_vel.
"""

import math
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class State(Enum):
    MARKER_LOST = auto()
    FOLLOWING   = auto()
    TOO_CLOSE   = auto()
    WAITING     = auto()


class FollowControllerNode(Node):
    def __init__(self):
        super().__init__('follow_controller_node')

        self._target_dist    = float(self.declare_parameter('target_distance_m',     1.2).value)
        self._min_safe_dist  = float(self.declare_parameter('min_safe_distance_m',   0.5).value)
        self._max_follow_dist = float(self.declare_parameter('max_follow_distance_m', 3.0).value)
        self._kp_lat         = float(self.declare_parameter('pid_lat_kp',            1.5).value)
        self._ki_lat         = float(self.declare_parameter('pid_lat_ki',            0.0).value)
        self._kd_lat         = float(self.declare_parameter('pid_lat_kd',            0.1).value)
        self._kp_lon         = float(self.declare_parameter('pid_lon_kp',            0.8).value)
        self._ki_lon         = float(self.declare_parameter('pid_lon_ki',            0.0).value)
        self._kd_lon         = float(self.declare_parameter('pid_lon_kd',            0.05).value)
        self._max_linear     = float(self.declare_parameter('max_linear_speed',      0.8).value)
        self._max_angular    = float(self.declare_parameter('max_angular_z',         2.0).value)
        self._wait_entry_err = float(self.declare_parameter('waiting_entry_error_m', 0.15).value)
        self._wait_exit_err  = float(self.declare_parameter('waiting_exit_error_m',  0.25).value)
        self._wait_frames    = int(self.declare_parameter('waiting_frames',           12).value)
        self._lost_timeout   = float(self.declare_parameter('marker_lost_timeout',   1.5).value)
        rate_hz              = float(self.declare_parameter('control_rate_hz',        20.0).value)
        self._dt             = 1.0 / rate_hz

        self.create_subscription(Float32, 'follow_error',    self._on_error,    1)
        self.create_subscription(Float32, 'follow_distance', self._on_distance, 1)
        self._cmd_pub = self.create_publisher(Twist, 'follow_cmd_vel', 1)

        self._last_error = None
        self._last_dist  = None
        self._last_marker_time = None

        self._state        = State.MARKER_LOST
        self._integral_lat = 0.0
        self._integral_lon = 0.0
        self._prev_err_lat = 0.0
        self._prev_err_lon = 0.0
        self._stable_count = 0

        self.create_timer(self._dt, self._control_step)
        self.get_logger().info('follow_controller_node started')

    def _on_error(self, msg: Float32):
        self._last_error = float(msg.data)

    def _on_distance(self, msg: Float32):
        self._last_dist = float(msg.data)

    def _control_step(self):
        now = self.get_clock().now()
        error_valid = (self._last_error is not None) and not math.isnan(self._last_error)
        dist_valid  = (self._last_dist  is not None) and not math.isnan(self._last_dist)
        marker_visible = (
            error_valid and dist_valid
            and self._last_dist <= self._max_follow_dist
        )

        if marker_visible:
            self._last_marker_time = now

        if self._last_marker_time is None:
            lost_elapsed = float('inf')
        else:
            lost_elapsed = (now - self._last_marker_time).nanoseconds * 1e-9
        marker_lost = lost_elapsed > self._lost_timeout

        prev_state = self._state
        if self._state == State.MARKER_LOST:
            if marker_visible:
                self._state = State.FOLLOWING
        elif self._state == State.FOLLOWING:
            if marker_lost:
                self._state = State.MARKER_LOST
            elif dist_valid and self._last_dist < self._min_safe_dist:
                self._state = State.TOO_CLOSE
            else:
                if dist_valid and abs(self._last_dist - self._target_dist) < self._wait_entry_err:
                    self._stable_count += 1
                else:
                    self._stable_count = 0
                if self._stable_count >= self._wait_frames:
                    self._stable_count = 0
                    self._state = State.WAITING
        elif self._state == State.TOO_CLOSE:
            if marker_lost:
                self._state = State.MARKER_LOST
            elif dist_valid and self._last_dist >= self._min_safe_dist:
                self._state = State.FOLLOWING
        elif self._state == State.WAITING:
            if marker_lost:
                self._state = State.MARKER_LOST
            elif dist_valid and abs(self._last_dist - self._target_dist) > self._wait_exit_err:
                self._state = State.FOLLOWING

        if self._state != prev_state:
            if self._state == State.FOLLOWING:
                self._integral_lat = 0.0
                self._integral_lon = 0.0
                self._prev_err_lat = self._last_error if error_valid else 0.0
                self._prev_err_lon = (self._last_dist - self._target_dist) if dist_valid else 0.0
            self.get_logger().info(f'state: {prev_state.name} -> {self._state.name}')

        cmd = Twist()
        if self._state == State.FOLLOWING:
            err_lat = self._last_error if error_valid else 0.0
            err_lon = (self._last_dist - self._target_dist) if dist_valid else 0.0

            self._integral_lat += err_lat * self._dt
            self._integral_lon += err_lon * self._dt
            d_lat = (err_lat - self._prev_err_lat) / self._dt
            d_lon = (err_lon - self._prev_err_lon) / self._dt
            self._prev_err_lat = err_lat
            self._prev_err_lon = err_lon

            angular_raw = (
                self._kp_lat * err_lat
                + self._ki_lat * self._integral_lat
                + self._kd_lat * d_lat
            )
            linear_raw = (
                self._kp_lon * err_lon
                + self._ki_lon * self._integral_lon
                + self._kd_lon * d_lon
            )

            # Positive lat error = marker right → turn right → negative angular.z (REP-103)
            cmd.angular.z = float(max(-self._max_angular, min(self._max_angular, -angular_raw)))
            # Never reverse: clamp linear to [0, max_linear_speed]
            cmd.linear.x  = float(max(0.0, min(self._max_linear, linear_raw)))

        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
