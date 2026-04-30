import os

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
        self.declare_parameter('hsv_lower', [0, 0, 0])
        self.declare_parameter('hsv_upper', [68, 255, 255])
        self.declare_parameter('roi_height_percent', 0.40)
        self.declare_parameter('num_roi_strips', 3)
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('lost_trail_patience', 5)
        self.declare_parameter('show_debug', True)

        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.hsv_lower = np.array(self.get_parameter('hsv_lower').value)
        self.hsv_upper = np.array(self.get_parameter('hsv_upper').value)
        self.roi_height_percent = self.get_parameter('roi_height_percent').value
        self.num_roi_strips = self.get_parameter('num_roi_strips').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        morph_k = self.get_parameter('morph_kernel_size').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.lost_trail_patience = self.get_parameter('lost_trail_patience').value
        self.show_debug = self.get_parameter('show_debug').value
        # Without an X11 DISPLAY, Qt's xcb plugin aborts the whole process when
        # cv2.imshow is invoked (this is a hard abort, not a catchable Python
        # exception). Detect that up-front and disable debug rendering.
        if self.show_debug and not os.environ.get('DISPLAY'):
            self.get_logger().warn(
                'No DISPLAY environment variable — running headless, disabling show_debug'
            )
            self.show_debug = False

        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_k, morph_k)
        )

        self.error_pub = self.create_publisher(Float32, 'trail_error', 10)
        self.heading_pub = self.create_publisher(Float32, 'trail_heading', 10)

        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Webcam {self.camera_index} not accessible')
            raise RuntimeError('Camera open failed')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        self.smoothed_error = 0.0
        self.consecutive_lost = 0
        self.frame_count = 0

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)

        self.get_logger().info(
            f'Camera node started: {self.frame_width}x{self.frame_height} '
            f'@ {self.publish_rate_hz} Hz | strips={self.num_roi_strips} '
            f'EMA alpha={self.ema_alpha} patience={self.lost_trail_patience}'
        )

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame not read')
            return

        height, width = frame.shape[:2]
        roi_start = int(height * (1.0 - self.roi_height_percent))
        roi = frame[roi_start:, :]
        roi_h = roi.shape[0]

        # Build cleaned mask once over the full ROI
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        # OPEN removes isolated noise pixels; CLOSE fills small gaps inside the trail blob
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        strip_h = roi_h // self.num_roi_strips
        strip_points = []  # (cx, y_in_roi) for each strip with a valid blob

        for i in range(self.num_roi_strips):
            y0 = roi_h - (i + 1) * strip_h
            y1 = roi_h - i * strip_h
            strip_mask = mask[y0:y1, :]

            contours, _ = cv2.findContours(
                strip_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
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

            if self.show_debug:
                cv2.drawContours(roi[y0:y1], [largest], -1, (0, 255, 0), 2)
                cv2.circle(roi[y0:y1], (int(cx), int(cy)), 8, (0, 0, 255), -1)

        # Compute lateral error and heading from a line fit across strips
        heading_raw = float('nan')
        if len(strip_points) >= 2:
            xs = np.array([p[0] for p in strip_points])
            ys = np.array([p[1] for p in strip_points])
            # Fit x = a*y + b; project to the bottom row to get lateral error
            a, b = np.polyfit(ys, xs, 1)
            x_bottom = a * roi_h + b
            error_raw = float(np.clip((x_bottom - width / 2.0) / (width / 2.0), -1.0, 1.0))
            heading_raw = float(np.arctan(a))  # rad; positive = trail leans right
            if self.show_debug:
                cv2.line(roi, (int(a * 0 + b), 0), (int(x_bottom), roi_h), (0, 255, 255), 2)
        elif len(strip_points) == 1:
            cx = strip_points[0][0]
            error_raw = float(np.clip((cx - width / 2.0) / (width / 2.0), -1.0, 1.0))
        else:
            error_raw = None

        # EMA smoothing + lost-trail debouncing
        if error_raw is not None:
            if self.consecutive_lost > self.lost_trail_patience:
                # Snap immediately on re-acquire after a confirmed full loss
                self.smoothed_error = error_raw
            else:
                self.smoothed_error = (
                    self.ema_alpha * error_raw
                    + (1.0 - self.ema_alpha) * self.smoothed_error
                )
            self.consecutive_lost = 0
            error = self.smoothed_error
        else:
            self.consecutive_lost += 1
            # Coast on last known value during the patience window, then go NaN
            error = (
                float('nan')
                if self.consecutive_lost > self.lost_trail_patience
                else self.smoothed_error
            )

        msg = Float32()
        msg.data = error
        self.error_pub.publish(msg)

        heading_msg = Float32()
        heading_msg.data = heading_raw
        self.heading_pub.publish(heading_msg)

        if self.show_debug:
            cv2.line(roi, (width // 2, 0), (width // 2, roi_h), (255, 255, 255), 1)
            for i in range(1, self.num_roi_strips):
                y = roi_h - i * strip_h
                cv2.line(roi, (0, y), (width, y), (80, 80, 80), 1)
            cv2.imshow('ROI + detection', roi)
            cv2.imshow('Mask', mask)
            cv2.waitKey(1)

        self.frame_count += 1
        if self.frame_count % max(1, int(self.publish_rate_hz)) == 0:
            self.get_logger().info(
                f'[{self.frame_count}] error={error:.3f} heading={heading_raw:.3f} '
                f'strips={len(strip_points)}/{self.num_roi_strips} '
                f'lost={self.consecutive_lost}'
            )

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
