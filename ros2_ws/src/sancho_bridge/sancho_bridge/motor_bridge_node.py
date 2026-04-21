# motor_bridge_node.py
# TODO: implement differential drive kinematics and Arduino Bridge RPC calls
# Subscribes to: /cmd_vel (geometry_msgs/Twist)
# Calls on MCU:  set_motors(left, right) via Arduino Bridge RPC
# Parameters:    wheel_separation, max_pwm, base_speed — from sancho_params.yaml

import rclpy
from rclpy.node import Node


class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')
        self.get_logger().info('MotorBridgeNode started — not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
