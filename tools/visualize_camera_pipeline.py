#!/usr/bin/env python3
"""
Local debug viewer for the camera_node pipeline.

Mirrors what sancho_perception/camera_node.py does, with live OpenCV
visualisation: ROI box, HSV mask, per-strip detected blobs, line fit, and
the resulting trail_error value. No ROS required — just OpenCV + NumPy.

Reads the same params from sancho_params.yaml as the node, so what you see
here is exactly what the node would process. Useful for tuning HSV/ROI on a
laptop with a webcam attached, before flashing the values to the rover.

Usage:
    python3 tools/visualize_camera_pipeline.py [camera_index]   (default: 0)

Press 'q' or Esc to quit.
"""

import os
import sys
import cv2
import numpy as np

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML not installed.  sudo apt install python3-yaml")
    sys.exit(1)


YAML_PATH = os.path.join(
    os.path.dirname(__file__),
    "..", "ros2_ws", "src", "sancho_bringup", "config", "sancho_params.yaml",
)

CAM_INDEX = int(sys.argv[1]) if len(sys.argv) > 1 else 0


def load_params(path):
    """Read camera_node's ros__parameters block from sancho_params.yaml."""
    with open(path) as f:
        data = yaml.safe_load(f)
    return data["camera_node"]["ros__parameters"]


def main():
    p = load_params(YAML_PATH)
    hsv_lower            = np.array(p["hsv_lower"])
    hsv_upper            = np.array(p["hsv_upper"])
    roi_pct              = float(p["roi_height_percent"])
    num_strips           = int(p.get("num_roi_strips", 3))
    min_contour_area     = int(p.get("min_contour_area", 500))
    morph_k              = int(p.get("morph_kernel_size", 5))
    ema_alpha            = float(p.get("ema_alpha", 0.3))
    lost_patience        = int(p.get("lost_trail_patience", 5))

    print(f"[viz] params from YAML:")
    print(f"      hsv_lower={hsv_lower.tolist()}  hsv_upper={hsv_upper.tolist()}")
    print(f"      roi={roi_pct}  strips={num_strips}  min_area={min_contour_area}")
    print(f"      morph_kernel={morph_k}  ema_alpha={ema_alpha}  patience={lost_patience}")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera index {CAM_INDEX}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))

    smoothed_error = 0.0
    consecutive_lost = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        h, w = frame.shape[:2]
        roi_y0 = int(h * (1.0 - roi_pct))
        roi = frame[roi_y0:, :].copy()
        roi_h = roi.shape[0]
        half_w = w / 2.0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)

        # Per-strip blob detection (same as camera_node)
        strip_h = roi_h // num_strips
        strip_points = []
        for i in range(num_strips):
            y0 = roi_h - (i + 1) * strip_h
            y1 = roi_h - i * strip_h
            contours, _ = cv2.findContours(
                mask[y0:y1, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) < min_contour_area:
                continue
            M = cv2.moments(largest)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            strip_points.append((cx, y0 + cy))

            # Draw the detected blob in the ROI
            cv2.drawContours(roi[y0:y1], [largest], -1, (0, 255, 0), 2)
            cv2.circle(roi[y0:y1], (int(cx), int(cy)), 8, (0, 0, 255), -1)

        # Line fit + error
        if len(strip_points) >= 2:
            pts = np.array(strip_points)
            a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)
            x_bottom = a * roi_h + b
            error_raw = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))
            cv2.line(
                roi,
                (int(b), 0),
                (int(x_bottom), roi_h),
                (0, 255, 255), 2,
            )
        elif len(strip_points) == 1:
            error_raw = float(
                np.clip((strip_points[0][0] - half_w) / half_w, -1.0, 1.0)
            )
        else:
            error_raw = None

        # EMA + lost-trail debounce
        if error_raw is not None:
            if consecutive_lost > lost_patience:
                smoothed_error = error_raw
            else:
                smoothed_error = ema_alpha * error_raw + (1.0 - ema_alpha) * smoothed_error
            consecutive_lost = 0
            error = smoothed_error
        else:
            consecutive_lost += 1
            error = float("nan") if consecutive_lost > lost_patience else smoothed_error

        # ── Overlay HUD on the full frame ──
        # ROI rectangle
        cv2.rectangle(frame, (0, roi_y0), (w - 1, h - 1), (0, 165, 255), 2)
        # Centre vertical line
        cv2.line(frame, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1)
        # Strip dividers within ROI
        for i in range(1, num_strips):
            y = roi_y0 + roi_h - i * strip_h
            cv2.line(frame, (0, y), (w, y), (80, 80, 80), 1)
        # Paste ROI back so the overlays we drew on it appear
        frame[roi_y0:, :] = roi
        # Status text
        err_str = "nan" if (error != error) else f"{error:+.3f}"
        cv2.putText(
            frame,
            f"err={err_str}  strips={len(strip_points)}/{num_strips}  lost={consecutive_lost}",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3,
        )
        cv2.putText(
            frame,
            f"err={err_str}  strips={len(strip_points)}/{num_strips}  lost={consecutive_lost}",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1,
        )

        cv2.imshow("camera_node pipeline (full frame + ROI overlay)", frame)
        cv2.imshow("mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
