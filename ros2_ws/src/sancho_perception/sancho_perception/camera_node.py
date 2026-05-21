"""
Camera node: detect a tape trail by LAB colour + CLAHE and publish two topics:

  /trail_error            Float32 in [-1, 1]  — lateral error at the bottom of ROI
                          (drives PID steering)
  /trail_lookahead_error  Float32 in [-1, 1]  — lateral error projected further ahead
                          (drives speed reduction before curves)

Two pure functions do the work; CameraNode wires them to the camera.

  lab_mask(roi_bgr, ...) -> uint8 binary mask
      BGR -> LAB, CLAHE on L channel, inRange on A+B, morphology cleanup,
      blob quality filter (area, solidity, width).

  mask_to_error(mask, ...) -> (error, lookahead_err)
      Splits the mask into horizontal strips, picks each strip's largest blob
      centroid, fits a line through >= 2 centroids, projects it to:
        - bottom of ROI  -> error
        - lookahead_row_fraction from top -> lookahead_err (clamped to topmost centroid)
      Returns (None, None) if fit is rejected or too few strips.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np


def lab_mask(roi_bgr, *,
             lab_lower, lab_upper,
             clahe_clip, clahe_tile,
             morph_k,
             min_contour_area, min_solidity, min_tape_width_px,
             min_total_mask_area):
    """LAB+CLAHE colour mask with blob quality filtering."""
    lab              = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    clahe            = cv2.createCLAHE(clipLimit=clahe_clip,
                                       tileGridSize=(clahe_tile, clahe_tile))
    l_eq             = clahe.apply(l_ch)
    lab_eq           = cv2.merge([l_eq, a_ch, b_ch])

    mask   = cv2.inRange(lab_eq, lab_lower, lab_upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    if cv2.countNonZero(mask) < min_total_mask_area:
        return np.zeros_like(mask)

    # Keep only tape-like blobs: sufficient area, solidity and width.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = np.zeros_like(mask)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_contour_area:
            continue
        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        if hull_area == 0 or area / hull_area < min_solidity:
            continue
        _, _, w, _ = cv2.boundingRect(cnt)
        if w < min_tape_width_px:
            continue
        cv2.drawContours(out, [cnt], -1, 255, -1)
    return out


def mask_to_error(mask, *,
                  num_strips,
                  min_strip_area,
                  max_fit_residual_px,
                  lookahead_row_fraction):
    """Reduce a binary mask to (error, lookahead_err); both in [-1,1] or both None."""
    h, w    = mask.shape
    half_w  = w / 2.0
    strip_h = h // num_strips
    centroids = []

    for i in range(num_strips):
        y0 = h - (i + 1) * strip_h
        y1 = h - i       * strip_h
        contours, _ = cv2.findContours(
            mask[y0:y1, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            continue
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_strip_area:
            continue
        M = cv2.moments(largest)
        if M['m00'] == 0:
            continue
        centroids.append((M['m10'] / M['m00'], y0 + M['m01'] / M['m00']))

    if len(centroids) < 2:
        return None, None

    pts      = np.array(centroids)
    a, b     = np.polyfit(pts[:, 1], pts[:, 0], 1)
    residual = float(np.mean(np.abs(a * pts[:, 1] + b - pts[:, 0])))
    if residual > max_fit_residual_px:
        return None, None

    x_bottom = a * h + b
    error    = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))

    # Project line further ahead; clamped to topmost detected centroid
    # so we never extrapolate beyond what was actually seen.
    top_detected_y = float(pts[:, 1].min())
    lookahead_y    = max(h * lookahead_row_fraction, top_detected_y)
    x_lookahead    = a * lookahead_y + b
    lookahead_err  = float(np.clip((x_lookahead - half_w) / half_w, -1.0, 1.0))

    return error, lookahead_err


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ROI / frame
        self.roi_height_percent = float(self.declare_parameter('roi_height_percent', 0.70).value)
        self.publish_rate_hz    = float(self.declare_parameter('publish_rate_hz',   30.0).value)

        # LAB colour params
        lab_a_min = int(self.declare_parameter('lab_a_min', 100).value)
        lab_a_max = int(self.declare_parameter('lab_a_max', 145).value)
        lab_b_min = int(self.declare_parameter('lab_b_min',  60).value)
        lab_b_max = int(self.declare_parameter('lab_b_max', 115).value)
        self.lab_lower = np.array([0,   lab_a_min, lab_b_min], dtype=np.uint8)
        self.lab_upper = np.array([255, lab_a_max, lab_b_max], dtype=np.uint8)

        # CLAHE + morphology
        self.clahe_clip  = float(self.declare_parameter('clahe_clip',      2.0).value)
        self.clahe_tile  = int(  self.declare_parameter('clahe_tile',        8).value)
        self.morph_k     = int(  self.declare_parameter('morph_kernel_size', 5).value)

        # Blob quality gate
        self.min_contour_area    = int(  self.declare_parameter('min_contour_area',      500).value)
        self.min_solidity        = float(self.declare_parameter('min_solidity',         0.60).value)
        self.min_tape_width_px   = int(  self.declare_parameter('min_tape_width_px',      15).value)
        self.min_total_mask_area = int(  self.declare_parameter('min_total_mask_area', 3000).value)

        # Strip → fit params
        self.num_strips             = int(  self.declare_parameter('num_roi_strips',            3).value)
        self.min_strip_area         = int(  self.declare_parameter('min_strip_area',          300).value)
        self.max_fit_residual_px    = float(self.declare_parameter('max_fit_residual_px',    30.0).value)
        self.lookahead_row_fraction = float(self.declare_parameter('lookahead_row_fraction',  0.0).value)

        # Smoothing / debounce
        self.ema_alpha           = float(self.declare_parameter('ema_alpha',           0.3).value)
        self.lost_trail_patience = int(  self.declare_parameter('lost_trail_patience',   5).value)

        self.smoothed_error         = 0.0
        self.smoothed_lookahead_err = 0.0
        self.consecutive_lost       = 0

        self.error_pub     = self.create_publisher(Float32, 'trail_error',           1)
        self.lookahead_pub = self.create_publisher(Float32, 'trail_lookahead_error', 1)

        cam_idx = int(self.declare_parameter('camera_index', 0).value)
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,
                     int(self.declare_parameter('frame_width',  640).value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,
                     int(self.declare_parameter('frame_height', 480).value))
        if not self.cap.isOpened():
            self.get_logger().error('Webcam not accessible')
            raise RuntimeError('Camera open failed')

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)
        self.get_logger().info('Camera node started (LAB+CLAHE detector with lookahead)')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame not read')
            return

        h   = frame.shape[0]
        roi = frame[int(h * (1.0 - self.roi_height_percent)):, :]

        mask = lab_mask(
            roi,
            lab_lower          = self.lab_lower,
            lab_upper          = self.lab_upper,
            clahe_clip         = self.clahe_clip,
            clahe_tile         = self.clahe_tile,
            morph_k            = self.morph_k,
            min_contour_area   = self.min_contour_area,
            min_solidity       = self.min_solidity,
            min_tape_width_px  = self.min_tape_width_px,
            min_total_mask_area= self.min_total_mask_area,
        )

        error_raw, lookahead_err_raw = mask_to_error(
            mask,
            num_strips             = self.num_strips,
            min_strip_area         = self.min_strip_area,
            max_fit_residual_px    = self.max_fit_residual_px,
            lookahead_row_fraction = self.lookahead_row_fraction,
        )

        if error_raw is not None:
            if self.consecutive_lost > self.lost_trail_patience:
                self.smoothed_error         = error_raw
                self.smoothed_lookahead_err = lookahead_err_raw
            else:
                self.smoothed_error = (
                    self.ema_alpha * error_raw
                    + (1.0 - self.ema_alpha) * self.smoothed_error
                )
                self.smoothed_lookahead_err = (
                    self.ema_alpha * lookahead_err_raw
                    + (1.0 - self.ema_alpha) * self.smoothed_lookahead_err
                )
            self.consecutive_lost = 0
            error         = self.smoothed_error
            lookahead_err = self.smoothed_lookahead_err
        else:
            self.consecutive_lost += 1
            if self.consecutive_lost > self.lost_trail_patience:
                error         = float('nan')
                lookahead_err = float('nan')
            else:
                error         = self.smoothed_error
                lookahead_err = self.smoothed_lookahead_err

        err_msg      = Float32(); err_msg.data  = error
        look_msg     = Float32(); look_msg.data = lookahead_err
        self.error_pub.publish(err_msg)
        self.lookahead_pub.publish(look_msg)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
