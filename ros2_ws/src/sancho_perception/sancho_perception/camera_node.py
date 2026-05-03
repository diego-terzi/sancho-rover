import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.hsv_lower           = np.array(self.declare_parameter('hsv_lower', [0, 0, 0]).value)
        self.hsv_upper           = np.array(self.declare_parameter('hsv_upper', [68, 255, 255]).value)
        self.roi_height_percent  = self.declare_parameter('roi_height_percent', 0.40).value
        
        self.publish_rate_hz     = 30.0
        self.num_roi_strips      = 3
        self.min_contour_area    = 500
        self.ema_alpha           = 0.3
        self.lost_trail_patience = 5

        self.smoothed_error      = 0.0
        self.consecutive_lost    = 0

        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        self.error_pub = self.create_publisher(Float32, 'trail_error', 1)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Webcam not accessible')
            raise RuntimeError('Camera open failed')

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)

        self.get_logger().info('Camera node started')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame not read')
            return

        height, width = frame.shape[:2]
        roi = frame[int(height * (1.0 - self.roi_height_percent)):, :]
        roi_h = roi.shape[0]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        strip_h = roi_h // self.num_roi_strips
        strip_points = []
        for i in range(self.num_roi_strips):
            y0 = roi_h - (i + 1) * strip_h
            y1 = roi_h - i * strip_h
            contours, _ = cv2.findContours(
                mask[y0:y1, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) < self.min_contour_area:
                continue
            M = cv2.moments(largest)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            strip_points.append((cx, y0 + cy))

        half_w = width / 2.0
        if len(strip_points) >= 2:
            pts = np.array(strip_points)
            a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)
            x_bottom = a * roi_h + b
            error_raw = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))
        elif len(strip_points) == 1:
            error_raw = float(np.clip((strip_points[0][0] - half_w) / half_w, -1.0, 1.0))
        else:
            error_raw = None
        if error_raw is not None:
            if self.consecutive_lost > self.lost_trail_patience:
                self.smoothed_error = error_raw
            else:
                self.smoothed_error = self.ema_alpha * error_raw + (1.0 - self.ema_alpha) * self.smoothed_error
            self.consecutive_lost = 0
            error = self.smoothed_error
        else:
            self.consecutive_lost += 1
            error = float('nan') if self.consecutive_lost > self.lost_trail_patience else self.smoothed_error

        msg = Float32()
        msg.data = error
        self.error_pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
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
