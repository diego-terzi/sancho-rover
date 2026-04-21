import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from arduino_bridge import ArduinoBridge

WHEEL_SEPARATION_DEFAULT = 0.2   # meters — TODO: measure actual chassis
MAX_SPEED_DEFAULT        = 1.0   # m/s    — TODO: tune once motors are tested
MAX_PWM_DEFAULT          = 255


class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        self.declare_parameter('wheel_separation', WHEEL_SEPARATION_DEFAULT)
        self.declare_parameter('max_speed',        MAX_SPEED_DEFAULT)
        self.declare_parameter('max_pwm',          MAX_PWM_DEFAULT)

        self._bridge = ArduinoBridge()

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

    def _cmd_vel_callback(self, msg: Twist):
        wheel_sep = self.get_parameter('wheel_separation').value
        max_speed = self.get_parameter('max_speed').value
        max_pwm   = int(self.get_parameter('max_pwm').value)

        left_speed  = msg.linear.x - msg.angular.z * wheel_sep / 2.0
        right_speed = msg.linear.x + msg.angular.z * wheel_sep / 2.0

        left_pwm  = int(left_speed  / max_speed * max_pwm)
        right_pwm = int(right_speed / max_speed * max_pwm)

        left_pwm  = max(-max_pwm, min(max_pwm, left_pwm))
        right_pwm = max(-max_pwm, min(max_pwm, right_pwm))

        try:
            self._bridge.call('set_motors', left_pwm, right_pwm)
        except Exception as exc:
            self.get_logger().error(f'Bridge call failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
