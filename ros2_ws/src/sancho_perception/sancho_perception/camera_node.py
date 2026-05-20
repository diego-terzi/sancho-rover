import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        lab_a_min  = self.declare_parameter('lab_a_min',  100).value
        lab_a_max  = self.declare_parameter('lab_a_max',  145).value
        lab_b_min  = self.declare_parameter('lab_b_min',  150).value
        lab_b_max  = self.declare_parameter('lab_b_max',  255).value
        clahe_clip = self.declare_parameter('clahe_clip',  2.0).value
        clahe_tile = self.declare_parameter('clahe_tile',    8).value
        self.clahe      = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))
        self.lab_lower  = np.array([0,   lab_a_min, lab_b_min])
        self.lab_upper  = np.array([255, lab_a_max, lab_b_max])
        self.roi_height_percent  = float(self.declare_parameter('roi_height_percent', 0.60).value)
        self.publish_rate_hz     = float(self.declare_parameter('publish_rate_hz',    30.0).value)
        self.num_roi_strips      = int(self.declare_parameter('num_roi_strips',          3).value)
        self.min_contour_area    = int(self.declare_parameter('min_contour_area',      500).value)
        self.min_solidity        = float(self.declare_parameter('min_solidity',        0.60).value)
        self.min_tape_width_px   = int(self.declare_parameter('min_tape_width_px',      15).value)
        self.min_total_mask_area = int(self.declare_parameter('min_total_mask_area', 3000).value)
        self.max_fit_residual_px = float(self.declare_parameter('max_fit_residual_px', 30.0).value)
        self.ema_alpha           = float(self.declare_parameter('ema_alpha',            0.3).value)
        self.lost_trail_patience = int(self.declare_parameter('lost_trail_patience',      5).value)
        morph_k                  = int(self.declare_parameter('morph_kernel_size',        5).value)

        self.smoothed_error      = 0.0
        self.consecutive_lost    = 0

        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))

        self.error_pub = self.create_publisher(Float32, 'trail_error', 1)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Webcam not accessible')
            raise RuntimeError('Camera open failed')

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)

        self.get_logger().info('Camera node started')

    def _is_tape_like(self, cnt) -> bool:
        area = cv2.contourArea(cnt)
        if area < self.min_contour_area:
            return False
        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        if hull_area == 0 or area / hull_area < self.min_solidity:
            return False
        _, _, w, _ = cv2.boundingRect(cnt)
        return w >= self.min_tape_width_px

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame not read')
            return

        height, width = frame.shape[:2]
        roi = frame[int(height * (1.0 - self.roi_height_percent)):, :]
        roi_h = roi.shape[0]
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab)
        l_eq = self.clahe.apply(l_ch)
        lab_eq = cv2.merge([l_eq, a_ch, b_ch])
        mask = cv2.inRange(lab_eq, self.lab_lower, self.lab_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        strip_points = []
        if cv2.countNonZero(mask) >= self.min_total_mask_area:
            strip_h = roi_h // self.num_roi_strips
            for i in range(self.num_roi_strips):
                y0 = roi_h - (i + 1) * strip_h
                y1 = roi_h - i * strip_h
                contours, _ = cv2.findContours(
                    mask[y0:y1, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                candidates = [c for c in contours if self._is_tape_like(c)]
                if not candidates:
                    continue
                largest = max(candidates, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M['m00'] == 0:
                    continue
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                strip_points.append((cx, y0 + cy))

        half_w = width / 2.0
        # Require >=2 strip detections AND a low fit residual: a single blob, or
        # three unrelated noise blobs that don't actually lie on a line, are
        # treated as "no trail". Residual = mean |x_pred - x_actual| in pixels.
        if len(strip_points) >= 2:
            pts = np.array(strip_points)
            a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)
            residual = float(np.mean(np.abs(a * pts[:, 1] + b - pts[:, 0])))
            if residual <= self.max_fit_residual_px:
                x_bottom = a * roi_h + b
                error_raw = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))
            else:
                error_raw = None
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