#!/usr/bin/env python3
"""
Test + calibrazione del detector LAB+CLAHE (vecchia pipeline, niente ONNX).

Apre la webcam, applica lab_mask() + mask_to_error() — le STESSE funzioni del
vecchio camera_node — e mostra maschera + overlay.

CALIBRAZIONE DRAG-TO-SAMPLE:
  Trascina col mouse un rettangolo SUL NASTRO BLU nella finestra "debug_view".
  Lo script legge i valori LAB (canali a,b) DENTRO quell'area DOPO il CLAHE,
  e imposta automaticamente i range a_min/a_max/b_min/b_max = [min-margine,
  max+margine] dei pixel campionati. La maschera si aggiorna subito.

Uso:
    /tmp/sancho_cam_venv/bin/python tools/test_camera_lab.py --camera 0

Tasti:
    q / ESC   esci
    s         stampa i valori correnti (da incollare nei params) + screenshot
    r         reset ai default
    [ / ]     margine campionamento -/+
"""

import argparse
import time

import cv2
import numpy as np


# ── Funzioni di visione (copia fedele dal vecchio camera_node LAB+CLAHE) ─────
def _clahe_lab(roi_bgr, clahe_clip, clahe_tile):
    lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))
    l_eq = clahe.apply(l_ch)
    return cv2.merge([l_eq, a_ch, b_ch])


def lab_mask(roi_bgr, *, lab_lower, lab_upper, clahe_clip, clahe_tile, morph_k,
             min_total_mask_area, min_contour_area, min_solidity,
             min_tape_width_px, min_elongation):
    lab_eq = _clahe_lab(roi_bgr, clahe_clip, clahe_tile)
    raw_mask = cv2.inRange(lab_eq, lab_lower, lab_upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))
    raw_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, kernel)
    raw_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, kernel)

    if cv2.countNonZero(raw_mask) < min_total_mask_area:
        return np.zeros_like(raw_mask)

    clean_mask = np.zeros_like(raw_mask)
    contours, _ = cv2.findContours(raw_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_contour_area:
            continue
        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        if hull_area == 0 or (area / hull_area) < min_solidity:
            continue
        _, _, ww, _ = cv2.boundingRect(cnt)
        if ww < min_tape_width_px:
            continue
        (_, _), (rw, rh), _ = cv2.minAreaRect(cnt)
        short_side = min(rw, rh)
        if short_side < 1.0:
            continue
        if (max(rw, rh) / short_side) < min_elongation:
            continue
        cv2.drawContours(clean_mask, [cnt], -1, 255, -1)
    return clean_mask


def mask_to_error(clean_mask, *, num_strips, max_fit_residual_px,
                  lookahead_row_fraction, debug_roi=None):
    h, w = clean_mask.shape
    half_w = w / 2.0
    strip_h = h // num_strips
    centroids = []
    for i in range(num_strips):
        y0 = h - (i + 1) * strip_h
        y1 = h - i * strip_h
        contours, _ = cv2.findContours(clean_mask[y0:y1, :], cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            continue
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        centroids.append((cx, y0 + cy))
        if debug_roi is not None:
            cv2.drawContours(debug_roi[y0:y1], [largest], -1, (0, 255, 0), 2)
            cv2.circle(debug_roi[y0:y1], (int(cx), int(cy)), 6, (0, 0, 255), -1)
    if debug_roi is not None:
        for i in range(1, num_strips):
            yl = h - i * strip_h
            cv2.line(debug_roi, (0, yl), (w, yl), (100, 100, 100), 1)
    if len(centroids) < 2:
        return None, None
    pts = np.array(centroids)
    a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)
    residual = float(np.mean(np.abs(a * pts[:, 1] + b - pts[:, 0])))
    if residual > max_fit_residual_px:
        return None, None
    x_bottom = a * h + b
    error = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))
    top_y = float(pts[:, 1].min())
    look_y = max(h * lookahead_row_fraction, top_y)
    x_look = a * look_y + b
    look_err = float(np.clip((x_look - half_w) / half_w, -1.0, 1.0))
    if debug_roi is not None:
        cv2.line(debug_roi, (int(b), 0), (int(x_bottom), h), (0, 255, 255), 2)
        cv2.circle(debug_roi, (int(x_look), int(look_y)), 8, (0, 165, 255), -1)
    return error, look_err


# ── Stato calibrazione (mutato dai callback del mouse) ───────────────────────
class Cal:
    def __init__(self, a_min, a_max, b_min, b_max):
        self.a_min, self.a_max = a_min, a_max
        self.b_min, self.b_max = b_min, b_max
        self.margin = 12
        self.dragging = False
        self.x0 = self.y0 = self.x1 = self.y1 = 0
        self.pending_rect = None   # (x0,y0,x1,y1) da campionare al prossimo frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--roi-height-percent", type=float, default=1.00)
    ap.add_argument("--num-strips", type=int, default=3)
    ap.add_argument("--clahe-clip", type=float, default=2.0)
    args = ap.parse_args()

    DEF = (100, 145, 60, 115)
    cal = Cal(*DEF)

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"[test] webcam {args.camera} non apribile")
        return

    WIN = "debug_view"
    cv2.namedWindow(WIN)

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            cal.dragging = True
            cal.x0, cal.y0 = x, y
            cal.x1, cal.y1 = x, y
        elif event == cv2.EVENT_MOUSEMOVE and cal.dragging:
            cal.x1, cal.y1 = x, y
        elif event == cv2.EVENT_LBUTTONUP:
            cal.dragging = False
            cal.x1, cal.y1 = x, y
            cal.pending_rect = (cal.x0, cal.y0, cal.x1, cal.y1)

    cv2.setMouseCallback(WIN, on_mouse)

    print("[test] LAB+CLAHE. Trascina un rettangolo SUL NASTRO BLU per calibrare.")
    print("       q/ESC esci | s stampa+screenshot | r reset | [ ] margine")
    last = time.time()
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        h, w = frame.shape[:2]
        roi_h = int(h * args.roi_height_percent)
        roi = frame[h - roi_h:h, :].copy()
        debug_roi = roi.copy()

        # Campiona l'area selezionata (sui canali a,b DOPO il CLAHE = come la pipeline)
        if cal.pending_rect is not None:
            x0, y0, x1, y1 = cal.pending_rect
            cal.pending_rect = None
            xa, xb = sorted((max(0, x0), min(roi.shape[1] - 1, x1)))
            ya, yb = sorted((max(0, y0), min(roi.shape[0] - 1, y1)))
            if xb - xa >= 3 and yb - ya >= 3:
                lab_eq = _clahe_lab(roi, args.clahe_clip, 8)
                patch = lab_eq[ya:yb, xa:xb]
                a_vals = patch[:, :, 1].astype(int)
                b_vals = patch[:, :, 2].astype(int)
                m = cal.margin
                cal.a_min = max(0, int(a_vals.min()) - m)
                cal.a_max = min(255, int(a_vals.max()) + m)
                cal.b_min = max(0, int(b_vals.min()) - m)
                cal.b_max = min(255, int(b_vals.max()) + m)
                print(f"[calib] area {xb-xa}x{yb-ya}px (margine {m}) -> "
                      f"a[{cal.a_min},{cal.a_max}] b[{cal.b_min},{cal.b_max}]  "
                      f"(a medio={int(a_vals.mean())} b medio={int(b_vals.mean())})")

        mask = lab_mask(
            roi,
            lab_lower=np.array([0, cal.a_min, cal.b_min], dtype=np.uint8),
            lab_upper=np.array([255, cal.a_max, cal.b_max], dtype=np.uint8),
            clahe_clip=args.clahe_clip, clahe_tile=8, morph_k=5,
            min_total_mask_area=3000, min_contour_area=500,
            min_solidity=0.60, min_tape_width_px=15, min_elongation=2.5,
        )
        error, look = mask_to_error(
            mask, num_strips=args.num_strips, max_fit_residual_px=30.0,
            lookahead_row_fraction=0.0, debug_roi=debug_roi,
        )

        # rettangolo di selezione live
        if cal.dragging:
            cv2.rectangle(debug_roi, (cal.x0, cal.y0), (cal.x1, cal.y1), (255, 0, 255), 2)

        now = time.time()
        dt = now - last
        last = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        etxt = "nan" if error is None else f"{error:+.3f}"
        ltxt = "nan" if look is None else f"{look:+.3f}"
        col = (0, 0, 255) if error is None else (0, 255, 0)
        cv2.putText(debug_roi, f"err {etxt}  look {ltxt}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.putText(debug_roi, f"a[{cal.a_min},{cal.a_max}] b[{cal.b_min},{cal.b_max}] "
                    f"m={cal.margin} fps{fps:4.1f}", (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        cv2.imshow(WIN, debug_roi)
        cv2.imshow("mask", mask)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("r"):
            cal.a_min, cal.a_max, cal.b_min, cal.b_max = DEF
            print("[test] reset ai default")
        elif k == ord("["):
            cal.margin = max(0, cal.margin - 2)
        elif k == ord("]"):
            cal.margin = min(60, cal.margin + 2)
        elif k == ord("s"):
            ts = int(time.time())
            cv2.imwrite(f"debug_{ts}.png", debug_roi)
            cv2.imwrite(f"mask_{ts}.png", mask)
            print("\n=== VALORI PER sancho_params.yaml (camera_node) ===")
            print(f"    lab_a_min: {cal.a_min}")
            print(f"    lab_a_max: {cal.a_max}")
            print(f"    lab_b_min: {cal.b_min}")
            print(f"    lab_b_max: {cal.b_max}")
            print(f"  (screenshot: debug_{ts}.png / mask_{ts}.png)\n")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
