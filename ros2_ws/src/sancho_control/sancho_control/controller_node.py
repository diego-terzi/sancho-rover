"""
Controller node: converts trail_error + obstacle distance into /cmd_vel.

State machine (see docs/architecture.md §5):
    FOLLOWING       PID runs, Twist with linear.x + angular.z correction.
    TRAIL_LOST      No valid trail for > trail_lost_timeout: stop.
    OBSTACLE_STOP   Front distance < obstacle_distance_m: stop (top priority).

Inputs:
    /trail_error (std_msgs/Float32)     -1..+1, NaN when no trail.
    /scan        (sensor_msgs/Range)    meters; optional (sensor_node TBD).

Output:
    /cmd_vel     (geometry_msgs/Twist)  published at control_rate_hz.
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
        self.declare_parameter('pid_kp', 1.0)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.0)
        self.declare_parameter('base_speed', 0.3)
        self.declare_parameter('trail_lost_timeout', 2.0)
        self.declare_parameter('obstacle_distance_m', 0.3)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('max_angular_z', 2.0)

        self.kp = float(self.get_parameter('pid_kp').value)
        self.ki = float(self.get_parameter('pid_ki').value)
        self.kd = float(self.get_parameter('pid_kd').value)
        self.base_speed = float(self.get_parameter('base_speed').value)
        self.trail_lost_timeout = float(self.get_parameter('trail_lost_timeout').value)
        self.obstacle_distance_m = float(self.get_parameter('obstacle_distance_m').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)

        # I/O
        self.create_subscription(Float32, 'trail_error', self._on_trail_error, 10)
        self.create_subscription(Range, 'scan', self._on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Latest data — stashed by callbacks, consumed by the control loop
        self.last_valid_error = 0.0
        self.last_valid_time = None
        self.last_distance = float('inf')
        self.last_distance_time = None

        # PID internal state
        self.prev_error = 0.0
        self.integral = 0.0

        # State machine
        self.current_state = State.TRAIL_LOST  # start stopped until we see a trail

        self.dt = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(self.dt, self._control_step)

        self._tick = 0  # for throttled logging

        self.get_logger().info(
            f'Controller started @ {self.control_rate_hz} Hz | '
            f'Kp={self.kp} Ki={self.ki} Kd={self.kd} | '
            f'base={self.base_speed} m/s | '
            f'lost_timeout={self.trail_lost_timeout} s | '
            f'obstacle<{self.obstacle_distance_m} m'
        )

    # --- Subscribers just stash; the timer does all the work ---

    def _on_trail_error(self, msg: Float32):
        if not math.isnan(msg.data):
            self.last_valid_error = float(msg.data)
            self.last_valid_time = self.get_clock().now()

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
        # No scan yet (sensor_node not running) — don't block on phantom obstacles
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

        cmd = Twist()  # defaults to all-zero

        if self.current_state == State.FOLLOWING:
            error = self.last_valid_error
            self.integral += error * self.dt
            derivative = (error - self.prev_error) / self.dt
            self.prev_error = error

            angular_correction = (
                self.kp * error
                + self.ki * self.integral
                + self.kd * derivative
            )
            # Positive error = trail is right of center → rover must turn right →
            # angular.z must be negative (ROS REP-103: +z is CCW / left turn).
            angular_z = -angular_correction
            angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))

            cmd.linear.x = self.base_speed
            cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)

        # Throttled status log — once per second at 20 Hz
        self._tick += 1
        if self._tick % int(self.control_rate_hz) == 0:
            self.get_logger().info(
                f'[{self.current_state.name}] '
                f'err={self.last_valid_error:+.3f} '
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
