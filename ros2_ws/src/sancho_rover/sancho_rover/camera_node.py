import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

CAMERA_INDEX_DEFAULT = 0
HSV_LOWER_DEFAULT    = [40, 80, 80]    # TODO: tune for fluorescent trail color
HSV_UPPER_DEFAULT    = [80, 255, 255]  # TODO: tune for fluorescent trail color
CROP_FRACTION_DEFAULT = 0.5            # TODO: tune during hardware testing
TIMER_HZ = 30.0


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index',  CAMERA_INDEX_DEFAULT)
        self.declare_parameter('hsv_lower',     HSV_LOWER_DEFAULT)
        self.declare_parameter('hsv_upper',     HSV_UPPER_DEFAULT)
        self.declare_parameter('crop_fraction', CROP_FRACTION_DEFAULT)

        self._pub = self.create_publisher(Float32, '/trail_error', 10)

        camera_index = self.get_parameter('camera_index').value
        self._cap = cv2.VideoCapture(camera_index)
        if not self._cap.isOpened():
            self.get_logger().error(f'Cannot open camera at index {camera_index}')

        self.create_timer(1.0 / TIMER_HZ, self._timer_callback)

    def _timer_callback(self):
        ret, frame = self._cap.read()
        if not ret:
            self._publish(float('nan'))
            return

        crop_fraction = self.get_parameter('crop_fraction').value
        h = frame.shape[0]
        cropped = frame[int(h * (1.0 - crop_fraction)):, :]

        hsv   = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        mask  = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._publish(float('nan'))
            return

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            self._publish(float('nan'))
            return

        cx    = M['m10'] / M['m00']
        width = cropped.shape[1]
        # 0.0 = centered, -1.0 = far left, +1.0 = far right
        error = (cx - width / 2.0) / (width / 2.0)
        self._publish(float(error))

    def _publish(self, value: float):
        msg = Float32()
        msg.data = value
        self._pub.publish(msg)

    def destroy_node(self):
        self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
