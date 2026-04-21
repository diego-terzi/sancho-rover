import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('hsv_lower', [6, 63, 41])
        self.declare_parameter('hsv_upper', [28, 225, 172])
        self.declare_parameter('roi_height_percent', 0.3)
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('show_debug', True)

        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value)
        self.roi_height_percent = self.get_parameter('roi_height_percent').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.show_debug = self.get_parameter('show_debug').value

        self.publisher_ = self.create_publisher(Float32, 'trail_error', 10)

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Webcam {self.camera_index} not accessible')
            raise RuntimeError('Camera open failed')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)
        self.frame_count = 0

        self.get_logger().info(
            f'Camera node started: {self.frame_width}x{self.frame_height} '
            f'@ {self.publish_rate_hz} Hz, '
            f'HSV lower={self.hsv_lower.tolist()}, upper={self.hsv_upper.tolist()}'
        )

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame not read')
            return

        height, width = frame.shape[:2]
        roi_start = int(height * (1 - self.roi_height_percent))
        roi = frame[roi_start:height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        error = float('nan')

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area >= self.min_contour_area:
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    error = (cx - width / 2) / (width / 2)

                    if self.show_debug:
                        cy = int(M['m01'] / M['m00'])
                        cv2.drawContours(roi, [largest], -1, (0, 255, 0), 2)
                        cv2.circle(roi, (int(cx), cy), 8, (0, 0, 255), -1)

        msg = Float32()
        msg.data = error
        self.publisher_.publish(msg)

        if self.show_debug:
            cv2.line(roi, (width // 2, 0), (width // 2, roi.shape[0]), (255, 255, 255), 1)
            cv2.imshow('ROI with detection', roi)
            cv2.imshow('Mask', mask)
            cv2.waitKey(1)

        self.frame_count += 1
        if self.frame_count % int(self.publish_rate_hz) == 0:
            self.get_logger().info(f'Frame {self.frame_count} | error: {error:.3f}')

    def destroy_node(self):
        self.cap.release()
        if self.show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
