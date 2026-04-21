import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

KP_DEFAULT              = 1.0   # TODO: tune this value
KI_DEFAULT              = 0.0   # TODO: tune this value
KD_DEFAULT              = 0.0   # TODO: tune this value
TRAIL_LOST_TIMEOUT_DEFAULT  = 2.0   # seconds — TODO: tune this value
OBSTACLE_DISTANCE_DEFAULT   = 0.3   # meters  — TODO: tune this value
BASE_SPEED_DEFAULT          = 0.2   # m/s     — TODO: tune this value
CONTROL_HZ = 20.0


class State(Enum):
    FOLLOWING     = 'FOLLOWING'
    TRAIL_LOST    = 'TRAIL_LOST'
    OBSTACLE_STOP = 'OBSTACLE_STOP'


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('pid_kp',              KP_DEFAULT)
        self.declare_parameter('pid_ki',              KI_DEFAULT)
        self.declare_parameter('pid_kd',              KD_DEFAULT)
        self.declare_parameter('trail_lost_timeout',  TRAIL_LOST_TIMEOUT_DEFAULT)
        self.declare_parameter('obstacle_distance_m', OBSTACLE_DISTANCE_DEFAULT)
        self.declare_parameter('base_speed',          BASE_SPEED_DEFAULT)

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32, '/trail_error', self._trail_callback, 10)
        self.create_subscription(Range,   '/scan',        self._scan_callback,  10)

        self._state              = State.FOLLOWING
        self._last_trail_time    = self.get_clock().now()
        self._last_scan_distance = float('inf')
        self._last_error_value   = float('nan')

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = self.get_clock().now()

        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)

    def _trail_callback(self, msg: Float32):
        self._last_error_value = msg.data
        if not math.isnan(msg.data):
            self._last_trail_time = self.get_clock().now()

    def _scan_callback(self, msg: Range):
        self._last_scan_distance = msg.range

    def _control_loop(self):
        kp              = self.get_parameter('pid_kp').value
        ki              = self.get_parameter('pid_ki').value
        kd              = self.get_parameter('pid_kd').value
        timeout         = self.get_parameter('trail_lost_timeout').value
        obstacle_thresh = self.get_parameter('obstacle_distance_m').value
        base_speed      = self.get_parameter('base_speed').value

        now              = self.get_clock().now()
        elapsed_no_trail = (now - self._last_trail_time).nanoseconds * 1e-9

        # Obstacle check has highest priority
        if self._last_scan_distance < obstacle_thresh:
            self._state = State.OBSTACLE_STOP
        elif self._state == State.OBSTACLE_STOP and self._last_scan_distance >= obstacle_thresh:
            self._state = State.FOLLOWING
        elif not math.isnan(self._last_error_value):
            self._state = State.FOLLOWING
        elif elapsed_no_trail > timeout:
            self._state = State.TRAIL_LOST

        if self._state != State.FOLLOWING:
            self._integral   = 0.0
            self._prev_error = 0.0
            self._prev_time  = now
            self._pub.publish(Twist())
            self.get_logger().warn(f'State: {self._state.value}', throttle_duration_sec=1.0)
            return

        error = self._last_error_value
        if math.isnan(error):
            error = 0.0

        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now

        if dt > 0:
            self._integral += error * dt
            derivative      = (error - self._prev_error) / dt
        else:
            derivative = 0.0
        self._prev_error = error

        angular = kp * error + ki * self._integral + kd * derivative

        cmd = Twist()
        cmd.linear.x  = base_speed
        cmd.angular.z = -angular  # positive error = trail right → turn right (negative angular.z)
        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
