# controller_node.py
# TODO: implement PID controller and state machine
# Subscribes to: /trail_error (std_msgs/Float32)
# Subscribes to: /obstacle_distance (sensor_msgs/Range) — optional
# Publishes to:  /cmd_vel (geometry_msgs/Twist)
# States: FOLLOWING / TRAIL_LOST / OBSTACLE_STOP

import rclpy
from rclpy.node import Node


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('ControllerNode started — not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
