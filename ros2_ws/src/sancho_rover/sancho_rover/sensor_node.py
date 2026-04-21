import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from arduino_bridge import ArduinoBridge

DEG_TO_RAD = math.pi / 180.0

# HC-SR04 physical characteristics
ULTRASONIC_FOV       = 0.26  # beam angle in radians (~15 degrees)
ULTRASONIC_MIN_RANGE = 0.02  # meters
ULTRASONIC_MAX_RANGE = 4.0   # meters


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        self._range_pub = self.create_publisher(Range, '/scan',     10)
        self._imu_pub   = self.create_publisher(Imu,   '/imu/data', 10)

        self._bridge = ArduinoBridge()
        self._bridge.provide('sensor_data', self._on_sensor_data)

        # bridge.run() blocks — run in a daemon thread so ROS 2 spin stays free
        self._bridge_thread = threading.Thread(target=self._bridge.run, daemon=True)
        self._bridge_thread.start()

    def _on_sensor_data(self, data: dict):
        stamp = self.get_clock().now().to_msg()

        range_msg = Range()
        range_msg.header.stamp      = stamp
        range_msg.header.frame_id   = 'ultrasonic'
        range_msg.radiation_type    = Range.ULTRASOUND
        range_msg.field_of_view     = ULTRASONIC_FOV
        range_msg.min_range         = ULTRASONIC_MIN_RANGE
        range_msg.max_range         = ULTRASONIC_MAX_RANGE
        range_msg.range             = data['distance'] / 100.0  # cm → meters
        self._range_pub.publish(range_msg)

        imu_msg = Imu()
        imu_msg.header.stamp    = stamp
        imu_msg.header.frame_id = 'imu'
        imu_msg.angular_velocity.z = data['gyro_z'] * DEG_TO_RAD  # deg/s → rad/s
        # Orientation and linear acceleration not available from MPU-6050 at this stage
        imu_msg.orientation_covariance[0]         = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self._imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
