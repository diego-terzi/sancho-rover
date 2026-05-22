#!/usr/bin/env python3
"""
Local debug viewer for the camera_node pipeline.

Mirrors what sancho_perception/camera_node.py does (lab_mask + mask_to_error),
with live OpenCV visualisation: ROI box, LAB mask, per-strip detected blobs,
line fit, trail_error, and trail_lookahead_error. No ROS required.

Reads the same params from sancho_params.yaml as the node. Interactive
trackbars let you tune all detector params live. Press 's' to write
the current trackbar values back to sancho_params.yaml.

Usage:
    python3 tools/visualize_camera_pipeline.py [camera_index]   (default: 0)

Keys:
    s        save current trackbar values to sancho_params.yaml
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

CTRL_WIN = "controls  [s]=save  [q]=quit"
VIZ_WIN  = "camera_node pipeline  [drag]=sample colour  [s]=save  [q]=quit"
MASK_WIN = "mask"

# Margins added around the percentile-sampled LAB range.
MARGIN_A = 10
MARGIN_B = 15

_sel = {'drawing': False, 'ix': 0, 'iy': 0, 'ex': 0, 'ey': 0, 'frame': None}


def _sample_lab(frame, x0, y0, x1, y1):
    """Return (a_lo, a_hi, b_lo, b_hi) sampled from the rectangle with margins."""
    patch = frame[y0:y1, x0:x1]
    if patch.size == 0:
        return None
    lab   = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB)
    a_ch  = lab[:, :, 1].flatten()
    b_ch  = lab[:, :, 2].flatten()
    a_lo  = max(0,   int(np.percentile(a_ch,  5)) - MARGIN_A)
    a_hi  = min(255, int(np.percentile(a_ch, 95)) + MARGIN_A)
    b_lo  = max(0,   int(np.percentile(b_ch,  5)) - MARGIN_B)
    b_hi  = min(255, int(np.percentile(b_ch, 95)) + MARGIN_B)
    return a_lo, a_hi, b_lo, b_hi


def _mouse_cb(event, x, y, flags, param):
    ctrl_win = param
    s = _sel
    if event == cv2.EVENT_LBUTTONDOWN:
        s['drawing'] = True
        s['ix'] = s['ex'] = x
        s['iy'] = s['ey'] = y
    elif event == cv2.EVENT_MOUSEMOVE and s['drawing']:
        s['ex'] = x
        s['ey'] = y
    elif event == cv2.EVENT_LBUTTONUP:
        s['drawing'] = False
        s['ex'] = x
        s['ey'] = y
        if s['frame'] is None:
            return
        x0, x1 = sorted([s['ix'], s['ex']])
        y0, y1 = sorted([s['iy'], s['ey']])
        if (x1 - x0) < 5 or (y1 - y0) < 5:
            return
        result = _sample_lab(s['frame'], x0, y0, x1, y1)
        if result is None:
            return
        a_lo, a_hi, b_lo, b_hi = result
        cv2.setTrackbarPos("A min", ctrl_win, a_lo)
        cv2.setTrackbarPos("A max", ctrl_win, a_hi)
        cv2.setTrackbarPos("B min", ctrl_win, b_lo)
        cv2.setTrackbarPos("B max", ctrl_win, b_hi)
        print(f"[viz] sampled LAB → A=[{a_lo},{a_hi}]  B=[{b_lo},{b_hi}]")


def load_params(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return data["camera_node"]["ros__parameters"]


def save_params(path, updates):
    """Update YAML lines in place, preserving comments. `updates`: list of (key, value_str)."""
    with open(path, "r") as f:
        text = f.read()
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
    print(f"[viz] saved {len(updates)} params to {os.path.basename(path)}")


def nothing(_):
    pass


def is_tape_like(cnt, min_area, min_solidity, min_width_px, min_elongation):
    """Mirror of CameraNode._is_tape_like: area + solidity + width + elongation gate.

    Elongation = long_side / short_side of the *rotated* min-area rect, so it
    measures how line-like a blob is regardless of orientation (a 45° tape is
    just as elongated as a vertical one). Grass/clutter blobs are roundish
    (elongation ~1) and get rejected; a continuous straight-or-angled tape
    segment is long and thin (elongation high)."""
    area = cv2.contourArea(cnt)
    if area < min_area:
        return False
    hull_area = cv2.contourArea(cv2.convexHull(cnt))
    if hull_area == 0 or area / hull_area < min_solidity:
        return False
    _, _, w, _ = cv2.boundingRect(cnt)
    if w < min_width_px:
        return False
    (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
    short = min(rw, rh)
    if short < 1.0:
        return False
    return (max(rw, rh) / short) >= min_elongation


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
    min_solidity     = float(p.get("min_solidity", 0.60))
    min_tape_width   = int(p.get("min_tape_width_px", 15))
    min_elongation   = float(p.get("min_elongation", 2.5))
    min_total_mask       = int(p.get("min_total_mask_area", 3000))
    max_fit_residual     = float(p.get("max_fit_residual_px", 30.0))
    lookahead_row_frac   = float(p.get("lookahead_row_fraction", 0.0))
    morph_k              = int(p.get("morph_kernel_size", 5))
    ema_alpha            = float(p.get("ema_alpha", 0.3))
    lost_patience        = int(p.get("lost_trail_patience", 5))

    print(f"[viz] params from YAML:")
    print(f"      lab_a=[{lab_a_min},{lab_a_max}]  lab_b=[{lab_b_min},{lab_b_max}]")
    print(f"      clahe_clip={clahe_clip}  clahe_tile={clahe_tile}")
    print(f"      roi={roi_pct}  strips={num_strips}  min_area={min_contour_area}")
    print(f"      min_solidity={min_solidity}  min_tape_width={min_tape_width}px  min_elongation={min_elongation}")
    print(f"      min_total_mask={min_total_mask}  max_residual={max_fit_residual}px")
    print(f"      lookahead_row_frac={lookahead_row_frac}  morph_kernel={morph_k}")
    print(f"      ema_alpha={ema_alpha}  patience={lost_patience}")
    print(f"[viz] press 's' to save, 'q'/Esc to quit")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"ERROR: cannot open camera index {CAM_INDEX}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))

    cv2.namedWindow(CTRL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL_WIN, 460, 470)
    cv2.createTrackbar("ROI % x100",     CTRL_WIN, int(roi_pct * 100),         100, nothing)
    cv2.createTrackbar("A min",          CTRL_WIN, lab_a_min,                  255, nothing)
    cv2.createTrackbar("A max",          CTRL_WIN, lab_a_max,                  255, nothing)
    cv2.createTrackbar("B min",          CTRL_WIN, lab_b_min,                  255, nothing)
    cv2.createTrackbar("B max",          CTRL_WIN, lab_b_max,                  255, nothing)
    cv2.createTrackbar("CLAHE clip x10", CTRL_WIN, int(clahe_clip * 10),        80, nothing)
    cv2.createTrackbar("CLAHE tile",     CTRL_WIN, clahe_tile,                  32, nothing)
    cv2.createTrackbar("min_area",       CTRL_WIN, min_contour_area,          5000, nothing)
    cv2.createTrackbar("solidity x100",  CTRL_WIN, int(min_solidity * 100),    100, nothing)
    cv2.createTrackbar("min_width_px",   CTRL_WIN, min_tape_width,             200, nothing)
    cv2.createTrackbar("elongation x10", CTRL_WIN, int(min_elongation * 10),   100, nothing)
    cv2.createTrackbar("min_total_mask", CTRL_WIN, min_total_mask,           30000, nothing)
    cv2.createTrackbar("residual_px",    CTRL_WIN, int(max_fit_residual),      100, nothing)
    cv2.createTrackbar("lookahead x100", CTRL_WIN, int(lookahead_row_frac * 100), 100, nothing)

    clahe      = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))
    prev_clip  = clahe_clip
    prev_tile  = clahe_tile

    smoothed_error        = 0.0
    smoothed_lookahead    = 0.0
    consecutive_lost      = 0
    cb_registered         = False

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        _sel['frame'] = frame.copy()  # clean copy for colour sampling

        # Read live trackbar values
        a_min    = cv2.getTrackbarPos("A min",          CTRL_WIN)
        a_max    = cv2.getTrackbarPos("A max",          CTRL_WIN)
        b_min    = cv2.getTrackbarPos("B min",          CTRL_WIN)
        b_max    = cv2.getTrackbarPos("B max",          CTRL_WIN)
        clip_int = cv2.getTrackbarPos("CLAHE clip x10", CTRL_WIN)
        tile     = max(1, cv2.getTrackbarPos("CLAHE tile", CTRL_WIN))
        clip     = clip_int / 10.0
        roi_pct_t        = max(0.05, cv2.getTrackbarPos("ROI % x100",    CTRL_WIN) / 100.0)
        min_area_t       = cv2.getTrackbarPos("min_area",       CTRL_WIN)
        solidity_t       = cv2.getTrackbarPos("solidity x100",  CTRL_WIN) / 100.0
        min_width_t      = cv2.getTrackbarPos("min_width_px",   CTRL_WIN)
        elongation_t     = cv2.getTrackbarPos("elongation x10", CTRL_WIN) / 10.0
        min_total_mask_t = cv2.getTrackbarPos("min_total_mask", CTRL_WIN)
        max_residual_t   = float(cv2.getTrackbarPos("residual_px", CTRL_WIN))
        lookahead_frac_t = cv2.getTrackbarPos("lookahead x100", CTRL_WIN) / 100.0

        # Recreate CLAHE only when its params actually change
        if clip != prev_clip or tile != prev_tile:
            clahe     = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
            prev_clip = clip
            prev_tile = tile

        h, w = frame.shape[:2]
        roi_y0 = int(h * (1.0 - roi_pct_t))
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

        mask_area   = int(cv2.countNonZero(mask))
        gate_passed = mask_area >= min_total_mask_t

        # Clean mask shown in MASK_WIN: only the blobs that pass is_tape_like
        # (the green ones). Rejected blobs (red border in the ROI) are dropped.
        mask_clean = np.zeros_like(mask)

        strip_h      = roi_h // num_strips
        strip_points = []
        if gate_passed:
            for i in range(num_strips):
                y0 = roi_h - (i + 1) * strip_h
                y1 = roi_h - i * strip_h
                contours, _ = cv2.findContours(
                    mask[y0:y1, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                tape_like = []
                rejected  = []
                for c in contours:
                    (tape_like if is_tape_like(c, min_area_t, solidity_t, min_width_t, elongation_t) else rejected).append(c)
                for c in rejected:
                    cv2.drawContours(roi[y0:y1], [c], -1, (0, 0, 255), 1)
                # Keep accepted blobs in the clean mask.
                for c in tape_like:
                    cv2.drawContours(mask_clean[y0:y1, :], [c], -1, 255, -1)
                if not tape_like:
                    continue
                largest = max(tape_like, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] == 0:
                    continue
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                strip_points.append((cx, y0 + cy))
                cv2.drawContours(roi[y0:y1], [largest], -1, (0, 255, 0), 2)
                cv2.circle(roi[y0:y1], (int(cx), int(cy)), 8, (0, 0, 255), -1)

        if not gate_passed:
            fit_status = f"mask_too_small ({mask_area}<{min_total_mask_t})"
        else:
            fit_status = f"need>=2 strips (got {len(strip_points)})"
        error_raw        = None
        lookahead_raw    = None
        if len(strip_points) >= 2:
            pts = np.array(strip_points)
            coef_a, coef_b = np.polyfit(pts[:, 1], pts[:, 0], 1)
            residual = float(np.mean(np.abs(coef_a * pts[:, 1] + coef_b - pts[:, 0])))
            x_bottom = coef_a * roi_h + coef_b
            if residual <= max_residual_t:
                error_raw = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))
                cv2.line(roi, (int(coef_b), 0), (int(x_bottom), roi_h), (0, 255, 255), 2)
                fit_status = f"fit_ok r={residual:.1f}"

                # Lookahead: same logic as camera_node.mask_to_error
                top_detected_y = float(pts[:, 1].min())
                look_y = max(roi_h * lookahead_frac_t, top_detected_y)
                x_look = coef_a * look_y + coef_b
                lookahead_raw = float(np.clip((x_look - half_w) / half_w, -1.0, 1.0))
                # Draw lookahead point
                cv2.circle(roi, (int(x_look), int(look_y)), 8, (255, 165, 0), -1)
                cv2.line(roi, (int(x_look) - 12, int(look_y)),
                              (int(x_look) + 12, int(look_y)), (255, 165, 0), 2)
            else:
                cv2.line(roi, (int(coef_b), 0), (int(x_bottom), roi_h), (255, 0, 255), 1)
                fit_status = f"fit_rejected r={residual:.1f}>{max_residual_t:.0f}"

        # EMA + lost-trail debounce (mirrors camera_node)
        if error_raw is not None:
            if consecutive_lost > lost_patience:
                smoothed_error     = error_raw
                smoothed_lookahead = lookahead_raw
            else:
                smoothed_error     = ema_alpha * error_raw     + (1.0 - ema_alpha) * smoothed_error
                smoothed_lookahead = ema_alpha * lookahead_raw + (1.0 - ema_alpha) * smoothed_lookahead
            consecutive_lost = 0
            error     = smoothed_error
            lookahead = smoothed_lookahead
        else:
            consecutive_lost += 1
            if consecutive_lost > lost_patience:
                error     = float("nan")
                lookahead = float("nan")
            else:
                error     = smoothed_error
                lookahead = smoothed_lookahead

        # ── Overlay HUD on the full frame ──
        cv2.rectangle(frame, (0, roi_y0), (w - 1, h - 1), (0, 165, 255), 2)
        cv2.line(frame, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1)
        for i in range(1, num_strips):
            y = roi_y0 + roi_h - i * strip_h
            cv2.line(frame, (0, y), (w, y), (80, 80, 80), 1)
        frame[roi_y0:, :] = roi

        err_str  = "nan" if (error    != error)    else f"{error:+.3f}"
        look_str = "nan" if (lookahead != lookahead) else f"{lookahead:+.3f}"
        line1 = f"err={err_str}  look={look_str}  strips={len(strip_points)}/{num_strips}  lost={consecutive_lost}"
        line2 = f"mask={mask_area}  {fit_status}"
        for y, txt in ((22, line1), (44, line2)):
            cv2.putText(frame, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(frame, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

        param_str = f"A=[{a_min},{a_max}]  B=[{b_min},{b_max}]  CLAHE={clip:.1f}/{tile}  elong>={elongation_t:.1f}"
        cv2.putText(frame, param_str, (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(frame, param_str, (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

        # Draw selection rectangle while dragging
        if _sel['drawing']:
            cv2.rectangle(frame, (_sel['ix'], _sel['iy']), (_sel['ex'], _sel['ey']),
                          (0, 255, 255), 2)

        cv2.imshow(VIZ_WIN, frame)
        cv2.imshow(MASK_WIN, mask_clean)

        if not cb_registered:
            cv2.waitKey(1)
            cv2.setMouseCallback(VIZ_WIN, _mouse_cb, CTRL_WIN)
            cb_registered = True

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("s"):
            save_params(YAML_PATH, [
                ("roi_height_percent",  f"{roi_pct_t:.2f}"),
                ("lab_a_min",           str(a_min)),
                ("lab_a_max",           str(a_max)),
                ("lab_b_min",           str(b_min)),
                ("lab_b_max",           str(b_max)),
                ("clahe_clip",          f"{clip:.1f}"),
                ("clahe_tile",          str(tile)),
                ("min_contour_area",    str(min_area_t)),
                ("min_solidity",        f"{solidity_t:.2f}"),
                ("min_tape_width_px",   str(min_width_t)),
                ("min_elongation",         f"{elongation_t:.1f}"),
                ("min_total_mask_area",    str(min_total_mask_t)),
                ("max_fit_residual_px",    f"{max_residual_t:.1f}"),
                ("lookahead_row_fraction", f"{lookahead_frac_t:.2f}"),
            ])

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
