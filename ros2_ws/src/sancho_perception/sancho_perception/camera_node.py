"""
Camera node: detect a tape trail by its *shape* (not colour) and publish a
normalised lateral error in [-1, 1] on /trail_error.

Two pure functions do the work; the ROS node just wires them to a video
stream and a publisher.

  shape_mask(roi_bgr, ...) -> uint8 binary mask
      Pipeline: grayscale + CLAHE → morphological top-hat AND black-hat
      with a horizontal kernel ~2× the expected tape width (matched filter
      for narrow vertical stripes of *either* polarity, light-on-dark or
      dark-on-light) → threshold → cleanup → connected-component filter
      by area and elongation.

  mask_to_error(mask, ...) -> (error, lookahead_err), each float in [-1, 1] or None
      Splits the mask into horizontal strips, picks each strip's largest
      blob centroid, fits a line through ≥ 2 centroids, then projects it to
      two rows:
        - the bottom of the ROI → `error` (drives PID steering)
        - a row higher up (controlled by `lookahead_row_fraction`) → `lookahead_err`
          (drives speed reduction in the controller_node)
      `lookahead_err` is clamped so it never extrapolates above the topmost
      detected centroid. Returns (None, None) if the fit is rejected.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np

def shape_mask(roi_bgr, *,
               tape_width_px,
               stripe_threshold,
               min_blob_area,
               min_elongation,
               clahe_clip,
               clahe_tile):
    """Binary mask of narrow vertical stripes ~tape_width_px wide. No colour cues."""
    gray  = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=clahe_clip,
                            tileGridSize=(clahe_tile, clahe_tile))
    gray  = clahe.apply(gray)

    # Kernel wider than the tape → opening removes the tape → top-hat recovers it.
    # Wide horizontally so vertical stripes are the ones suppressed by the opening.
    kw     = max(3, int(tape_width_px * 2) | 1)  # force odd width
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kw, 3))
    tophat   = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT,   kernel)  # bright stripes
    blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)  # dark stripes
    stripe_response = np.maximum(tophat, blackhat)

    _, mask = cv2.threshold(stripe_response, stripe_threshold, 255, cv2.THRESH_BINARY)

    # Open to kill speckle; close vertically to bridge gaps along the stripe.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 11)))

    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    out = np.zeros_like(mask)
    for i in range(1, n_labels):
        _, _, w_box, h_box, area = stats[i]
        if area < min_blob_area:
            continue
        if max(w_box, h_box) / max(1, min(w_box, h_box)) < min_elongation:
            continue
        out[labels == i] = 255
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
        if M["m00"] == 0:
            continue
        centroids.append((M["m10"] / M["m00"], y0 + M["m01"] / M["m00"]))

    if len(centroids) < 2:
        return None, None

    pts      = np.array(centroids)
    a, b     = np.polyfit(pts[:, 1], pts[:, 0], 1)
    residual = float(np.mean(np.abs(a * pts[:, 1] + b - pts[:, 0])))
    if residual > max_fit_residual_px:
        return None, None

    x_bottom = a * h + b
    error    = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))

    # Lookahead: project the same line to a row higher in the ROI (= further
    # ahead in space). Clamped so we never extrapolate above the topmost
    # detected centroid, which would amplify fit noise on short trails.
    top_detected_y = float(pts[:, 1].min())
    lookahead_y    = max(h * lookahead_row_fraction, top_detected_y)
    x_lookahead    = a * lookahead_y + b
    lookahead_err  = float(np.clip((x_lookahead - half_w) / half_w, -1.0, 1.0))

    return error, lookahead_err


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Frame / ROI
        self.roi_height_percent  = float(self.declare_parameter('roi_height_percent',  0.60).value)
        self.publish_rate_hz     = float(self.declare_parameter('publish_rate_hz',     30.0).value)
        # shape_mask params
        self.tape_width_px       = int(self.declare_parameter('tape_width_px',           41).value)
        self.stripe_threshold    = int(self.declare_parameter('stripe_threshold',        25).value)
        self.min_blob_area       = int(self.declare_parameter('min_blob_area',          800).value)
        self.min_elongation      = float(self.declare_parameter('min_elongation',        3.0).value)
        self.clahe_clip          = float(self.declare_parameter('clahe_clip',            2.0).value)
        self.clahe_tile          = int(self.declare_parameter('clahe_tile',                8).value)
        # mask_to_error params
        self.num_strips          = int(self.declare_parameter('num_roi_strips',            3).value)
        self.min_strip_area      = int(self.declare_parameter('min_strip_area',          500).value)
        self.max_fit_residual_px    = float(self.declare_parameter('max_fit_residual_px',    30.0).value)
        self.lookahead_row_fraction = float(self.declare_parameter('lookahead_row_fraction', 0.0).value)
        # Smoothing / debounce
        self.ema_alpha           = float(self.declare_parameter('ema_alpha',             0.3).value)
        self.lost_trail_patience = int(self.declare_parameter('lost_trail_patience',       5).value)

        self.smoothed_error         = 0.0
        self.smoothed_lookahead_err = 0.0
        self.consecutive_lost       = 0

        self.error_pub     = self.create_publisher(Float32, 'trail_error',           1)
        self.lookahead_pub = self.create_publisher(Float32, 'trail_lookahead_error', 1)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Webcam not accessible')
            raise RuntimeError('Camera open failed')

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)
        self.get_logger().info('Camera node started (shape-based detector)')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame not read')
            return

        h   = frame.shape[0]
        roi = frame[int(h * (1.0 - self.roi_height_percent)):, :]

        mask = shape_mask(
            roi,
            tape_width_px    = self.tape_width_px,
            stripe_threshold = self.stripe_threshold,
            min_blob_area    = self.min_blob_area,
            min_elongation   = self.min_elongation,
            clahe_clip       = self.clahe_clip,
            clahe_tile       = self.clahe_tile,
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

        err_msg  = Float32(); err_msg.data  = error
        look_msg = Float32(); look_msg.data = lookahead_err
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


if __name__ == '__main__':
    main()
