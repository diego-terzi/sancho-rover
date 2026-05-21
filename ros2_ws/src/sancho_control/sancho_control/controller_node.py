"""
Controller node: converts trail_error + obstacle distance into /cmd_vel.

State machine:
    FOLLOWING       Pure PID on lateral error, full speed by default.
    TRAIL_LOST      No valid trail for > trail_lost_timeout: stop.
    OBSTACLE_STOP   Front distance < obstacle_distance_m: stop (top priority).

Control law on FOLLOWING:
    angular.z = -(Kp*err + Ki*∫err + Kd*ḋerr)
    curve_intensity = clip(curve_sensitivity * max(|err|, |lookahead|), 0, 1)
              # curve_sensitivity > 1 anticipates braking: a moderate lookahead
              # already saturates curve_intensity to 1, so we slow down BEFORE
              # the curve gets sharp instead of mid-bend.
    linear.x  = min(max_linear_speed,
                    v_motor_max * (1 - (1 - slow_speed_ratio) * curve_intensity))
              # max_linear_speed caps top speed BELOW v_motor_max so we keep
              # PWM headroom for the differential — otherwise the outer track
              # saturates at v_max and the rover can't actually turn sharply.
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

        motor_rpm                = float(self.declare_parameter('motor_rpm', 300.0).value)
        wheel_diameter           = float(self.declare_parameter('wheel_diameter', 0.09).value)
        self.v_motor_max         = math.pi * wheel_diameter * motor_rpm / 60.0
        self.max_linear_speed    = float(self.declare_parameter('max_linear_speed', 0.7).value)
        self.kp                  = float(self.declare_parameter('pid_kp', 1.5).value)
        self.ki                  = float(self.declare_parameter('pid_ki', 0.0).value)
        self.kd                  = float(self.declare_parameter('pid_kd', 0.1).value)
        self.slow_speed_ratio    = float(self.declare_parameter('slow_speed_ratio', 0.25).value)
        self.curve_sensitivity   = float(self.declare_parameter('curve_sensitivity', 2.0).value)
        self.trail_lost_timeout  = float(self.declare_parameter('trail_lost_timeout', 2.0).value)
        self.obstacle_distance_m = float(self.declare_parameter('obstacle_distance_m', 0.3).value)
        self.control_rate_hz     = float(self.declare_parameter('control_rate_hz', 20.0).value)
        self.max_angular_z       = float(self.declare_parameter('max_angular_z', 2.0).value)

        self.create_subscription(Float32, 'trail_error',           self._on_trail_error, 1)
        self.create_subscription(Float32, 'trail_lookahead_error', self._on_lookahead,   1)
        self.create_subscription(Range,   'scan',                  self._on_scan,        1)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.last_valid_error   = 0.0
        self.last_valid_time    = None
        self.last_lookahead_err = 0.0
        self.last_distance      = float('inf')
        self.last_distance_time = None
        self.prev_error = 0.0
        self.integral = 0.0
        self.current_state = State.TRAIL_LOST
        self.dt = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(self.dt, self._control_step)

        self.get_logger().info('Controller node started')

    def _on_trail_error(self, msg: Float32):
        if not math.isnan(msg.data):
            self.last_valid_error = float(msg.data)
            self.last_valid_time = self.get_clock().now()

    def _on_lookahead(self, msg: Float32):
        if not math.isnan(msg.data):
            self.last_lookahead_err = float(msg.data)

    def _on_scan(self, msg: Range):
        self.last_distance = float(msg.range)
        self.last_distance_time = self.get_clock().now()

    def _control_step(self):
        now = self.get_clock().now()
        obstacle_close = (
            self.last_distance_time is not None
            and self.last_distance < self.obstacle_distance_m
        )
        trail_fresh = (
            self.last_valid_time is not None
            and (now - self.last_valid_time).nanoseconds * 1e-9 <= self.trail_lost_timeout
        )
        if obstacle_close:
            next_state = State.OBSTACLE_STOP
        elif trail_fresh:
            next_state = State.FOLLOWING
        else:
            next_state = State.TRAIL_LOST

        if next_state != self.current_state:
            self.get_logger().info(
                f'state: {self.current_state.name} -> {next_state.name}'
            )
            if next_state == State.FOLLOWING:
                # Fresh start on re-entry: prevents integral windup from a long
                # stop, and avoids a derivative kick when the trail re-appears
                # at a very different position than it was lost at.
                self.integral           = 0.0
                self.prev_error         = self.last_valid_error
                # Drop any stale anticipation from before the trail was lost.
                self.last_lookahead_err = 0.0
            self.current_state = next_state

        cmd = Twist()

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
            # Positive error = trail to the right → turn right →
            # angular.z negative (ROS REP-103: +z is CCW / left turn).
            angular_z = -angular_correction
            angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))

            # Slowdown driven by the worst of {current lateral error, lookahead},
            # amplified by curve_sensitivity so braking anticipates the curve
            # rather than reacting mid-bend.
            raw_intensity   = max(abs(error), abs(self.last_lookahead_err))
            curve_intensity = min(1.0, self.curve_sensitivity * raw_intensity)
            speed = self.v_motor_max * (1.0 - (1.0 - self.slow_speed_ratio) * curve_intensity)
            # Hard cap — keeps PWM headroom for the steering differential.
            speed = min(speed, self.max_linear_speed)

            cmd.linear.x  = speed
            cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)


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
