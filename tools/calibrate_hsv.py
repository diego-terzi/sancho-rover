#!/usr/bin/env python3
"""
HSV calibration tool for SANCHO trail detection.

Usage:
    python3 tools/calibrate_hsv.py [camera_index]   (default: 0)

Steps:
    1. Point the camera at the fluorescent trail.
    2. Click and drag a rectangle directly over the trail in the live feed.
    3. The mask updates immediately with auto-sampled HSV bounds.
    4. Fine-tune with the trackbars if needed.
    5. Adjust the ROI% trackbar to set how much of the bottom of the frame to use.
    6. Press 's' to print the final values — copy-paste them into sancho_params.yaml.
    7. Press 'q' or ESC to quit.

No ROS required — only OpenCV and NumPy.
"""

import sys
import cv2
import numpy as np

# Margins added around the percentile-sampled HSV range to tolerate
# small lighting shifts without requiring a re-sample.
MARGIN_H = 10   # hue — tight, hue is stable
MARGIN_S = 30   # saturation — moderate, varies with distance / exposure
MARGIN_V = 40   # value/brightness — widest, most affected by motion and shadows

# Shared mutable state for the mouse callback
_state = {
    'frame': None,   # last grabbed camera frame
    'drawing': False,
    'ix': 0, 'iy': 0,
    'ex': 0, 'ey': 0,
}

hsv_lower = np.array([0,   0,   0],   dtype=np.uint8)
hsv_upper = np.array([180, 255, 255], dtype=np.uint8)

WIN_CAM  = 'Camera'
WIN_MASK = 'Mask'
WIN_CTRL = 'Controls'


def _on_trackbar(_val):
    # getTrackbarPos returns -1 if the window isn't ready yet; clamp to avoid
    # uint8 overflow which NumPy will reject in future versions.
    hsv_lower[0] = max(0, cv2.getTrackbarPos('H min', WIN_CTRL))
    hsv_lower[1] = max(0, cv2.getTrackbarPos('S min', WIN_CTRL))
    hsv_lower[2] = max(0, cv2.getTrackbarPos('V min', WIN_CTRL))
    hsv_upper[0] = max(0, cv2.getTrackbarPos('H max', WIN_CTRL))
    hsv_upper[1] = max(0, cv2.getTrackbarPos('S max', WIN_CTRL))
    hsv_upper[2] = max(0, cv2.getTrackbarPos('V max', WIN_CTRL))


def _sample_hsv(frame, x0, y0, x1, y1):
    """Return (lower, upper) HSV arrays sampled from the rectangle with margins."""
    patch = frame[y0:y1, x0:x1]
    if patch.size == 0:
        return None, None
    hsv_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    h = hsv_patch[:, :, 0].flatten()
    s = hsv_patch[:, :, 1].flatten()
    v = hsv_patch[:, :, 2].flatten()
    lower = np.array([
        max(0,   int(np.percentile(h,  5)) - MARGIN_H),
        max(0,   int(np.percentile(s,  5)) - MARGIN_S),
        max(0,   int(np.percentile(v,  5)) - MARGIN_V),
    ], dtype=np.uint8)
    upper = np.array([
        min(180, int(np.percentile(h, 95)) + MARGIN_H),
        min(255, int(np.percentile(s, 95)) + MARGIN_S),
        min(255, int(np.percentile(v, 95)) + MARGIN_V),
    ], dtype=np.uint8)
    return lower, upper


def _set_trackbars(lo, hi):
    cv2.setTrackbarPos('H min', WIN_CTRL, int(lo[0]))
    cv2.setTrackbarPos('S min', WIN_CTRL, int(lo[1]))
    cv2.setTrackbarPos('V min', WIN_CTRL, int(lo[2]))
    cv2.setTrackbarPos('H max', WIN_CTRL, int(hi[0]))
    cv2.setTrackbarPos('S max', WIN_CTRL, int(hi[1]))
    cv2.setTrackbarPos('V max', WIN_CTRL, int(hi[2]))


def _mouse_callback(event, x, y, flags, param):
    s = param
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
        lo, hi = _sample_hsv(s['frame'], x0, y0, x1, y1)
        if lo is None:
            return
        hsv_lower[:] = lo
        hsv_upper[:] = hi
        _set_trackbars(lo, hi)
        print(f'Sampled — lower: {lo.tolist()}  upper: {hi.tolist()}')


def main():
    cam_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f'ERROR: cannot open camera index {cam_idx}')
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Controls window
    cv2.namedWindow(WIN_CTRL)
    cv2.createTrackbar('H min', WIN_CTRL,   0, 180, _on_trackbar)
    cv2.createTrackbar('H max', WIN_CTRL, 180, 180, _on_trackbar)
    cv2.createTrackbar('S min', WIN_CTRL,   0, 255, _on_trackbar)
    cv2.createTrackbar('S max', WIN_CTRL, 255, 255, _on_trackbar)
    cv2.createTrackbar('V min', WIN_CTRL,   0, 255, _on_trackbar)
    cv2.createTrackbar('V max', WIN_CTRL, 255, 255, _on_trackbar)
    cv2.createTrackbar('ROI %', WIN_CTRL,  40, 100, _on_trackbar)  # bottom % of frame

    cv2.namedWindow(WIN_CAM, cv2.WINDOW_AUTOSIZE)
    # Mouse callback is registered after the first real imshow inside the loop —
    # on Qt/Wayland the window handle is only valid once the event loop has ticked
    # with real content in the window.
    callback_registered = False

    print()
    print('=== SANCHO HSV Calibration ===')
    print(f'  Camera index : {cam_idx}')
    print('  Drag a rectangle over the trail to auto-sample HSV.')
    print('  Fine-tune with trackbars.  ROI% sets the detection band height.')
    print("  Press 's' to print values for sancho_params.yaml")
    print("  Press 'q' or ESC to quit")
    print()

    while True:
        ret, frame = cap.read()
        if not ret:
            print('Frame read error — check camera connection')
            break

        _state['frame'] = frame.copy()

        roi_pct = max(0.05, cv2.getTrackbarPos('ROI %', WIN_CTRL) / 100.0)
        frame_h, frame_w = frame.shape[:2]
        roi_y = int(frame_h * (1.0 - roi_pct))

        # Compute mask over the full frame (user may drag anywhere)
        hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_full, hsv_lower, hsv_upper)

        # Semi-transparent green overlay on detected pixels
        display = frame.copy()
        overlay = np.zeros_like(display)
        overlay[mask > 0] = (0, 200, 0)
        display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)

        # ROI boundary
        cv2.line(display, (0, roi_y), (frame_w, roi_y), (0, 165, 255), 2)
        cv2.putText(
            display, f'ROI boundary  ({roi_pct * 100:.0f}% of frame)',
            (5, roi_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1,
        )

        # Selection rectangle while dragging
        if _state['drawing']:
            cv2.rectangle(
                display,
                (_state['ix'], _state['iy']),
                (_state['ex'], _state['ey']),
                (0, 255, 255), 2,
            )

        # Current HSV range at the bottom of the frame
        cv2.putText(
            display,
            f"lower {hsv_lower.tolist()}   upper {hsv_upper.tolist()}",
            (5, frame_h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
        )

        cv2.imshow(WIN_CAM, display)
        cv2.imshow(WIN_MASK, mask)

        if not callback_registered:
            cv2.waitKey(1)  # tick event loop so WIN_CAM is fully registered
            cv2.setMouseCallback(WIN_CAM, _mouse_callback, _state)
            callback_registered = True

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('s'):
            roi_pct_final = max(0.05, cv2.getTrackbarPos('ROI %', WIN_CTRL) / 100.0)
            print()
            print('--- paste into sancho_params.yaml ---')
            print(f'    hsv_lower: {hsv_lower.tolist()}')
            print(f'    hsv_upper: {hsv_upper.tolist()}')
            print(f'    roi_height_percent: {roi_pct_final:.2f}')
            print('-------------------------------------')
            print()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
