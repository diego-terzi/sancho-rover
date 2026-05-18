#!/usr/bin/env python3
"""
Local debug viewer for the shape-based camera_node pipeline.

Mirrors what sancho_perception/camera_node.py does (shape_mask + mask_to_error),
with live OpenCV visualisation: the binary mask, the per-strip centroids, the
line fit, and the resulting trail_error value. No ROS required — just OpenCV.

Reads the same params from sancho_params.yaml as the node. Trackbars tune the
detector live. Press 's' to write the current trackbar values back to the YAML
(only the keys that have a trackbar are touched; comments are preserved).

Usage:
    python3 tools/visualize_shape_pipeline.py [camera_index]   (default: 0)

Keys:
    s        save current trackbar values to sancho_params.yaml
    r        toggle stripe_response (top-hat output) window on/off
    q / Esc  quit
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

CTRL_WIN = "controls  [s]=save  [r]=response  [q]=quit"
VIZ_WIN  = "shape pipeline (frame + ROI overlay)"
MASK_WIN = "mask"
RESP_WIN = "stripe_response (top-hat)"


# ── kept in sync with camera_node.py — copy on every change ───────────────────

def shape_mask_debug(roi_bgr, *,
                    tape_width_px, stripe_threshold, min_blob_area,
                    min_elongation, clahe_clip, clahe_tile):
    """Same as camera_node.shape_mask, but also returns the intermediate top-hat response."""
    gray  = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=clahe_clip,
                            tileGridSize=(clahe_tile, clahe_tile))
    gray  = clahe.apply(gray)

    kw     = max(3, int(tape_width_px * 2) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kw, 3))
    tophat   = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT,   kernel)
    blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)
    stripe_response = np.maximum(tophat, blackhat)

    _, mask = cv2.threshold(stripe_response, stripe_threshold, 255, cv2.THRESH_BINARY)
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
    return out, stripe_response


def mask_to_error_debug(mask, *,
                        num_strips, min_strip_area, max_fit_residual_px):
    """Same as camera_node.mask_to_error, plus centroids/fit/status for visualisation."""
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
        return None, centroids, None, f"need>=2 strips (got {len(centroids)})"

    pts      = np.array(centroids)
    a, b     = np.polyfit(pts[:, 1], pts[:, 0], 1)
    residual = float(np.mean(np.abs(a * pts[:, 1] + b - pts[:, 0])))
    if residual > max_fit_residual_px:
        return None, centroids, (a, b, residual), f"fit_rejected r={residual:.1f}>{max_fit_residual_px:.0f}"

    x_bottom = a * h + b
    err = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))
    return err, centroids, (a, b, residual), f"fit_ok r={residual:.1f}"


# ── YAML I/O ──────────────────────────────────────────────────────────────────

def load_params(path):
    with open(path) as f:
        return yaml.safe_load(f)["camera_node"]["ros__parameters"]


def save_params(path, updates):
    """Replace only the numeric value of each YAML key, preserving trailing comments."""
    with open(path, "r") as f:
        text = f.read()
    for key, val in updates:
        text = re.sub(
            rf"^(\s*{re.escape(key)}:\s*)[\d.]+(\s*(?:#[^\n]*)?)",
            rf"\g<1>{val}\g<2>",
            text,
            flags=re.MULTILINE,
        )
    with open(path, "w") as f:
        f.write(text)
    print(f"[viz] saved {len(updates)} params to {os.path.basename(path)}")


def nothing(_):
    pass


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    p = load_params(YAML_PATH)
    roi_pct          = float(p.get("roi_height_percent",   1.0))
    tape_width_px    = int(  p.get("tape_width_px",         15))
    stripe_threshold = int(  p.get("stripe_threshold",      25))
    min_blob_area    = int(  p.get("min_blob_area",        800))
    min_elongation   = float(p.get("min_elongation",        3.0))
    clahe_clip       = float(p.get("clahe_clip",            2.0))
    clahe_tile       = int(  p.get("clahe_tile",              8))
    num_strips       = int(  p.get("num_roi_strips",          3))
    min_strip_area   = int(  p.get("min_strip_area",        500))
    max_fit_residual = float(p.get("max_fit_residual_px",  30.0))

    print(f"[viz] params from YAML:")
    print(f"      tape_width_px={tape_width_px}  stripe_threshold={stripe_threshold}")
    print(f"      min_blob_area={min_blob_area}  min_elongation={min_elongation}")
    print(f"      CLAHE={clahe_clip}/{clahe_tile}  roi={roi_pct}")
    print(f"      num_strips={num_strips}  min_strip_area={min_strip_area}")
    print(f"      max_fit_residual={max_fit_residual}px")
    print(f"[viz] keys: s=save  r=toggle response  q=quit")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera index {CAM_INDEX}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WIN, 500, 480)
    cv2.createTrackbar("tape_width_px",    CTRL_WIN, tape_width_px,             100, nothing)
    cv2.createTrackbar("stripe_threshold", CTRL_WIN, stripe_threshold,          255, nothing)
    cv2.createTrackbar("min_blob_area",    CTRL_WIN, min_blob_area,           10000, nothing)
    cv2.createTrackbar("elongation x10",   CTRL_WIN, int(min_elongation * 10),  100, nothing)
    cv2.createTrackbar("CLAHE clip x10",   CTRL_WIN, int(clahe_clip * 10),       80, nothing)
    cv2.createTrackbar("CLAHE tile",       CTRL_WIN, clahe_tile,                 32, nothing)
    cv2.createTrackbar("min_strip_area",   CTRL_WIN, min_strip_area,           5000, nothing)
    cv2.createTrackbar("residual_px",      CTRL_WIN, int(max_fit_residual),     100, nothing)

    show_response = False

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        twp     = max(1, cv2.getTrackbarPos("tape_width_px",    CTRL_WIN))
        thr     =        cv2.getTrackbarPos("stripe_threshold", CTRL_WIN)
        mba     =        cv2.getTrackbarPos("min_blob_area",    CTRL_WIN)
        elong   =        cv2.getTrackbarPos("elongation x10",   CTRL_WIN) / 10.0
        ccl     =        cv2.getTrackbarPos("CLAHE clip x10",   CTRL_WIN) / 10.0
        ctl     = max(1, cv2.getTrackbarPos("CLAHE tile",       CTRL_WIN))
        msa     =        cv2.getTrackbarPos("min_strip_area",   CTRL_WIN)
        res_t   = float( cv2.getTrackbarPos("residual_px",      CTRL_WIN))

        h, w   = frame.shape[:2]
        roi_y0 = int(h * (1.0 - roi_pct))
        roi    = frame[roi_y0:, :].copy()

        mask, response = shape_mask_debug(
            roi,
            tape_width_px=twp, stripe_threshold=thr,
            min_blob_area=mba, min_elongation=elong,
            clahe_clip=ccl,    clahe_tile=ctl,
        )
        error, centroids, fit, status = mask_to_error_debug(
            mask,
            num_strips=num_strips, min_strip_area=msa,
            max_fit_residual_px=res_t,
        )

        # ── Build ROI overlay ──
        roi_viz = roi.copy()
        roi_h   = roi.shape[0]
        strip_h = roi_h // num_strips
        for i in range(1, num_strips):
            y = roi_h - i * strip_h
            cv2.line(roi_viz, (0, y), (roi.shape[1], y), (80, 80, 80), 1)
        for cx, cy in centroids:
            cv2.circle(roi_viz, (int(cx), int(cy)), 6, (0, 0, 255), -1)
        if fit is not None:
            a, b, _ = fit
            colour = (0, 255, 255) if error is not None else (255, 0, 255)
            cv2.line(roi_viz, (int(b), 0), (int(a * roi_h + b), roi_h), colour, 2)
        cv2.line(roi_viz, (w // 2, 0), (w // 2, roi_h), (255, 255, 255), 1)

        # ── Compose final frame ──
        frame[roi_y0:, :] = roi_viz
        if roi_pct < 1.0:
            cv2.rectangle(frame, (0, roi_y0), (w - 1, h - 1), (0, 165, 255), 2)

        err_str = "None" if error is None else f"{error:+.3f}"
        line1   = f"err={err_str}  strips={len(centroids)}/{num_strips}  {status}"
        line2   = (f"tape_w={twp}  thr={thr}  min_blob={mba}  elong={elong:.1f}  "
                   f"CLAHE={ccl:.1f}/{ctl}")
        for y, txt in ((22, line1), (44, line2)):
            cv2.putText(frame, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(frame, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

        cv2.imshow(VIZ_WIN, frame)
        cv2.imshow(MASK_WIN, mask)
        if show_response:
            # Normalise top-hat output to 0..255 for visualisation only.
            resp_viz = cv2.normalize(response, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            cv2.imshow(RESP_WIN, resp_viz)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("r"):
            show_response = not show_response
            if not show_response:
                cv2.destroyWindow(RESP_WIN)
        if key == ord("s"):
            save_params(YAML_PATH, [
                ("tape_width_px",       str(twp)),
                ("stripe_threshold",    str(thr)),
                ("min_blob_area",       str(mba)),
                ("min_elongation",      f"{elong:.1f}"),
                ("clahe_clip",          f"{ccl:.1f}"),
                ("clahe_tile",          str(ctl)),
                ("min_strip_area",      str(msa)),
                ("max_fit_residual_px", f"{res_t:.1f}"),
            ])

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
