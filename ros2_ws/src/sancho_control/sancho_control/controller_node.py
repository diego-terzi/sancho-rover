"""
Controller node: converts trail_error + trail_heading + obstacle distance into /cmd_vel.

State machine (see docs/architecture.md §5):
    FOLLOWING       PID + heading feed-forward, adaptive linear speed.
    TRAIL_LOST      No valid trail for > trail_lost_timeout: stop.
    OBSTACLE_STOP   Front distance < obstacle_distance_m: stop (top priority).

Control law on FOLLOWING:
    angular.z = -(Kp*err + Ki*∫err + Kd*ḋerr + heading_ff_gain * heading)
    linear.x  = max_speed - (max_speed - min_speed) * curvature
                where curvature = clamp(max(|err|, |heading|) * curve_slowdown_gain, 0, 1)

Inputs:
    /trail_error   (std_msgs/Float32)   -1..+1, NaN when no trail.
    /trail_heading (std_msgs/Float32)   rad, NaN when single-strip detection.
    /scan          (sensor_msgs/Range)  meters; optional (sensor_node TBD).

Output:
    /cmd_vel       (geometry_msgs/Twist) published at control_rate_hz.
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class State(Enum):
    FOLLOWING = auto()
    TRAIL_LOST = auto()
    OBSTACLE_STOP = auto()


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Parameters
        self.declare_parameter('pid_kp', 1.5)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.1)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_speed', 0.9)
        self.declare_parameter('heading_ff_gain', 1.5)
        self.declare_parameter('curve_slowdown_gain', 1.5)
        self.declare_parameter('trail_lost_timeout', 2.0)
        self.declare_parameter('obstacle_distance_m', 0.3)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('max_angular_z', 2.0)

        self.kp = float(self.get_parameter('pid_kp').value)
        self.ki = float(self.get_parameter('pid_ki').value)
        self.kd = float(self.get_parameter('pid_kd').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.heading_ff_gain = float(self.get_parameter('heading_ff_gain').value)
        self.curve_slowdown_gain = float(self.get_parameter('curve_slowdown_gain').value)
        self.trail_lost_timeout = float(self.get_parameter('trail_lost_timeout').value)
        self.obstacle_distance_m = float(self.get_parameter('obstacle_distance_m').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)

        # I/O
        self.create_subscription(Float32, 'trail_error', self._on_trail_error, 10)
        self.create_subscription(Float32, 'trail_heading', self._on_trail_heading, 10)
        self.create_subscription(Range, 'scan', self._on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Latest data — stashed by callbacks, consumed by the control loop
        self.last_valid_error = 0.0
        self.last_valid_time = None
        self.last_heading = 0.0
        self.last_distance = float('inf')
        self.last_distance_time = None

        # PID internal state
        self.prev_error = 0.0
        self.integral = 0.0

        # State machine
        self.current_state = State.TRAIL_LOST

        self.dt = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(self.dt, self._control_step)

        self._tick = 0

        self.get_logger().info(
            f'Controller started @ {self.control_rate_hz} Hz | '
            f'Kp={self.kp} Ki={self.ki} Kd={self.kd} | '
            f'speed={self.min_speed}..{self.max_speed} m/s | '
            f'ff={self.heading_ff_gain} slow={self.curve_slowdown_gain} | '
            f'lost_timeout={self.trail_lost_timeout} s | '
            f'obstacle<{self.obstacle_distance_m} m'
        )

    # --- Subscribers just stash; the timer does all the work ---

    def _on_trail_error(self, msg: Float32):
        if not math.isnan(msg.data):
            self.last_valid_error = float(msg.data)
            self.last_valid_time = self.get_clock().now()

    def _on_trail_heading(self, msg: Float32):
        # NaN = single-strip detection (no line fit) → fall back to no feed-forward
        self.last_heading = 0.0 if math.isnan(msg.data) else float(msg.data)

    def _on_scan(self, msg: Range):
        self.last_distance = float(msg.range)
        self.last_distance_time = self.get_clock().now()

    # --- State decision ---

    def _trail_is_fresh(self) -> bool:
        if self.last_valid_time is None:
            return False
        age_s = (self.get_clock().now() - self.last_valid_time).nanoseconds * 1e-9
        return age_s <= self.trail_lost_timeout

    def _obstacle_is_close(self) -> bool:
        if self.last_distance_time is None:
            return False
        return self.last_distance < self.obstacle_distance_m

    def _next_state(self) -> State:
        if self._obstacle_is_close():
            return State.OBSTACLE_STOP
        if self._trail_is_fresh():
            return State.FOLLOWING
        return State.TRAIL_LOST

    # --- Control loop ---

    def _control_step(self):
        next_state = self._next_state()

        if next_state != self.current_state:
            self.get_logger().info(
                f'state: {self.current_state.name} -> {next_state.name}'
            )
            if next_state == State.FOLLOWING:
                # Fresh start on re-entry: prevents integral windup from a long
                # stop, and avoids a derivative kick when the trail re-appears
                # at a very different position than it was lost at.
                self.integral = 0.0
                self.prev_error = self.last_valid_error
            self.current_state = next_state

        cmd = Twist()

        if self.current_state == State.FOLLOWING:
            error = self.last_valid_error
            heading = self.last_heading

            self.integral += error * self.dt
            derivative = (error - self.prev_error) / self.dt
            self.prev_error = error

            angular_correction = (
                self.kp * error
                + self.ki * self.integral
                + self.kd * derivative
                + self.heading_ff_gain * heading
            )
            # Positive error/heading = trail to the right → turn right →
            # angular.z negative (ROS REP-103: +z is CCW / left turn).
            angular_z = -angular_correction
            angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))

            curvature = min(
                1.0,
                max(abs(error), abs(heading)) * self.curve_slowdown_gain,
            )
            speed = self.max_speed - (self.max_speed - self.min_speed) * curvature

            cmd.linear.x = speed
            cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)

        self._tick += 1
        if self._tick % int(self.control_rate_hz) == 0:
            self.get_logger().info(
                f'[{self.current_state.name}] '
                f'err={self.last_valid_error:+.3f} '
                f'hdg={self.last_heading:+.3f} '
                f'cmd.lin={cmd.linear.x:.2f} cmd.ang={cmd.angular.z:+.3f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
