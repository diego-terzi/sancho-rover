#!/usr/bin/env python3
"""
Local debug viewer for the camera_node pipeline.

Mirrors what sancho_perception/camera_node.py does, with live OpenCV
visualisation: ROI box, LAB mask, per-strip detected blobs, line fit, and
the resulting trail_error value. No ROS required — just OpenCV + NumPy.

Reads the same params from sancho_params.yaml as the node. Interactive
trackbars let you tune the LAB+CLAHE thresholds live. Press 's' to write
the current trackbar values back to sancho_params.yaml.

Usage:
    python3 tools/visualize_camera_pipeline.py [camera_index]   (default: 0)

Press 's' to save parameters to sancho_params.yaml.
Press 'q' or Esc to quit.
"""

import os
import re
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

CTRL_WIN = "controls  [s]=save  [q]=quit"
VIZ_WIN  = "camera_node pipeline (full frame + ROI overlay)"
MASK_WIN = "mask"


def load_params(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return data["camera_node"]["ros__parameters"]


def save_params(path, lab_a_min, lab_a_max, lab_b_min, lab_b_max, clahe_clip, clahe_tile):
    """Update only the LAB+CLAHE lines in the YAML, preserving all comments."""
    with open(path, "r") as f:
        text = f.read()

    updates = [
        ("lab_a_min",  str(lab_a_min)),
        ("lab_a_max",  str(lab_a_max)),
        ("lab_b_min",  str(lab_b_min)),
        ("lab_b_max",  str(lab_b_max)),
        ("clahe_clip", f"{clahe_clip:.1f}"),
        ("clahe_tile", str(clahe_tile)),
    ]
    for key, val in updates:
        # Replace only the numeric value; preserve any trailing inline comment.
        text = re.sub(
            rf"^(\s*{re.escape(key)}:\s*)[\d.]+(\s*(?:#[^\n]*)?)",
            rf"\g<1>{val}\g<2>",
            text,
            flags=re.MULTILINE,
        )

    with open(path, "w") as f:
        f.write(text)
    print(f"[viz] saved → lab_a=[{lab_a_min},{lab_a_max}]  "
          f"lab_b=[{lab_b_min},{lab_b_max}]  clahe_clip={clahe_clip:.1f}  clahe_tile={clahe_tile}")


def nothing(_):
    pass


def main():
    p = load_params(YAML_PATH)
    lab_a_min        = int(p.get("lab_a_min",  100))
    lab_a_max        = int(p.get("lab_a_max",  145))
    lab_b_min        = int(p.get("lab_b_min",  150))
    lab_b_max        = int(p.get("lab_b_max",  255))
    clahe_clip       = float(p.get("clahe_clip", 2.0))
    clahe_tile       = int(p.get("clahe_tile",    8))
    roi_pct          = float(p["roi_height_percent"])
    num_strips       = int(p.get("num_roi_strips", 3))
    min_contour_area = int(p.get("min_contour_area", 500))
    morph_k          = int(p.get("morph_kernel_size", 5))
    ema_alpha        = float(p.get("ema_alpha", 0.3))
    lost_patience    = int(p.get("lost_trail_patience", 5))

    print(f"[viz] params from YAML:")
    print(f"      lab_a=[{lab_a_min},{lab_a_max}]  lab_b=[{lab_b_min},{lab_b_max}]")
    print(f"      clahe_clip={clahe_clip}  clahe_tile={clahe_tile}")
    print(f"      roi={roi_pct}  strips={num_strips}  min_area={min_contour_area}")
    print(f"      morph_kernel={morph_k}  ema_alpha={ema_alpha}  patience={lost_patience}")
    print(f"[viz] press 's' to save, 'q'/Esc to quit")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera index {CAM_INDEX}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))

    cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WIN, 420, 200)
    cv2.createTrackbar("A min",          CTRL_WIN, lab_a_min,              255, nothing)
    cv2.createTrackbar("A max",          CTRL_WIN, lab_a_max,              255, nothing)
    cv2.createTrackbar("B min",          CTRL_WIN, lab_b_min,              255, nothing)
    cv2.createTrackbar("B max",          CTRL_WIN, lab_b_max,              255, nothing)
    cv2.createTrackbar("CLAHE clip x10", CTRL_WIN, int(clahe_clip * 10),    80, nothing)
    cv2.createTrackbar("CLAHE tile",     CTRL_WIN, clahe_tile,              32, nothing)

    clahe      = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))
    prev_clip  = clahe_clip
    prev_tile  = clahe_tile

    smoothed_error   = 0.0
    consecutive_lost = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        # Read live trackbar values
        a_min    = cv2.getTrackbarPos("A min",          CTRL_WIN)
        a_max    = cv2.getTrackbarPos("A max",          CTRL_WIN)
        b_min    = cv2.getTrackbarPos("B min",          CTRL_WIN)
        b_max    = cv2.getTrackbarPos("B max",          CTRL_WIN)
        clip_int = cv2.getTrackbarPos("CLAHE clip x10", CTRL_WIN)
        tile     = max(1, cv2.getTrackbarPos("CLAHE tile", CTRL_WIN))
        clip     = clip_int / 10.0

        # Recreate CLAHE only when its params actually change
        if clip != prev_clip or tile != prev_tile:
            clahe     = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
            prev_clip = clip
            prev_tile = tile

        h, w = frame.shape[:2]
        roi_y0 = int(h * (1.0 - roi_pct))
        roi    = frame[roi_y0:, :].copy()
        roi_h  = roi.shape[0]
        half_w = w / 2.0

        # ── LAB + CLAHE colour detection ──────────────────────────────────────
        lab              = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab)
        l_eq             = clahe.apply(l_ch)
        lab_eq           = cv2.merge([l_eq, a_ch, b_ch])
        lab_lower        = np.array([0,   a_min, b_min])
        lab_upper        = np.array([255, a_max, b_max])
        mask             = cv2.inRange(lab_eq, lab_lower, lab_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)

        # Per-strip blob detection (same as camera_node)
        strip_h      = roi_h // num_strips
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

            cv2.drawContours(roi[y0:y1], [largest], -1, (0, 255, 0), 2)
            cv2.circle(roi[y0:y1], (int(cx), int(cy)), 8, (0, 0, 255), -1)

        # Line fit + error
        if len(strip_points) >= 2:
            pts = np.array(strip_points)
            coef_a, coef_b = np.polyfit(pts[:, 1], pts[:, 0], 1)
            x_bottom  = coef_a * roi_h + coef_b
            error_raw = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))
            cv2.line(roi, (int(coef_b), 0), (int(x_bottom), roi_h), (0, 255, 255), 2)
        elif len(strip_points) == 1:
            error_raw = float(np.clip((strip_points[0][0] - half_w) / half_w, -1.0, 1.0))
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
        cv2.rectangle(frame, (0, roi_y0), (w - 1, h - 1), (0, 165, 255), 2)
        cv2.line(frame, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1)
        for i in range(1, num_strips):
            y = roi_y0 + roi_h - i * strip_h
            cv2.line(frame, (0, y), (w, y), (80, 80, 80), 1)
        frame[roi_y0:, :] = roi

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
        param_str = f"A=[{a_min},{a_max}]  B=[{b_min},{b_max}]  CLAHE={clip:.1f}/{tile}"
        cv2.putText(frame, param_str, (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(frame, param_str, (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

        cv2.imshow(VIZ_WIN, frame)
        cv2.imshow(MASK_WIN, mask)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("s"):
            save_params(YAML_PATH, a_min, a_max, b_min, b_max, clip, tile)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
