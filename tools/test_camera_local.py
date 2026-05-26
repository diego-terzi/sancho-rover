#!/usr/bin/env python3
"""
Test locale del camera_node: apre una webcam, gira il modello ONNX di line
detection e mostra a schermo cosa "vede" il rover.

Riusa LE STESSE funzioni del nodo reale (get_trail_mask, mask_to_error) per
non divergere dalla pipeline di bordo. Nessun ROS richiesto: solo opencv,
numpy, onnxruntime.

Uso:
    python3 tools/test_camera_local.py
    python3 tools/test_camera_local.py --camera 2 --model /percorso/trail_detector.onnx

Tasti nella finestra:
    q / ESC   esci
    m         mostra/nascondi la finestra maschera
    s         salva uno screenshot (debug_view + mask) nella cwd
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

# --- Importa le funzioni REALI del camera_node ------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_NODE_DIR = os.path.join(
    _HERE, "..", "ros2_ws", "src", "sancho_perception", "sancho_perception"
)
sys.path.insert(0, os.path.abspath(_NODE_DIR))

# camera_node.py importa rclpy/cv_bridge in cima (dipendenze ROS non presenti
# sul laptop). Non possiamo importarlo direttamente. Estraiamo SOLO le due
# funzioni pure (get_trail_mask, mask_to_error) eseguendone il sorgente in un
# namespace che ha già cv2/numpy — saltando le righe di import ROS.
def _load_node_functions():
    import re

    src_path = os.path.join(os.path.abspath(_NODE_DIR), "camera_node.py")
    with open(src_path, "r") as f:
        src = f.read()

    # Tieni dall'inizio fino a (esclusa) la definizione della classe CameraNode.
    marker = "\nclass CameraNode"
    idx = src.find(marker)
    if idx != -1:
        src = src[:idx]

    # Rimuovi le righe di import (le dipendenze le forniamo noi nel namespace).
    lines = []
    for ln in src.splitlines():
        stripped = ln.strip()
        if stripped.startswith("import ") or stripped.startswith("from "):
            continue
        lines.append(ln)
    code = "\n".join(lines)

    ns = {"cv2": cv2, "np": np, "os": os}
    exec(compile(code, src_path, "exec"), ns)
    return ns["get_trail_mask"], ns["mask_to_error"]


try:
    get_trail_mask, mask_to_error = _load_node_functions()
    print("[test] funzioni get_trail_mask/mask_to_error estratte da camera_node.py")
except Exception as e:
    print(f"[test] ERRORE estrazione funzioni dal nodo: {e}")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=2,
                    help="indice webcam (default 2 = C270 sul rover; 0 sul laptop)")
    ap.add_argument("--model", default=os.path.join(
        os.path.abspath(_NODE_DIR), "..", "models", "trail_detector.onnx"),
        help="percorso al .onnx")
    # Parametri identici ai default di sancho_params.yaml (camera_node)
    ap.add_argument("--roi-height-percent", type=float, default=1.00)
    ap.add_argument("--num-strips", type=int, default=3)
    ap.add_argument("--max-fit-residual-px", type=float, default=30.0)
    ap.add_argument("--lookahead-row-fraction", type=float, default=0.00)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    model_path = os.path.abspath(args.model)
    if not os.path.isfile(model_path):
        print(f"[test] ERRORE: modello non trovato: {model_path}")
        sys.exit(1)

    import onnxruntime as ort
    print(f"[test] carico modello: {model_path}")
    model = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"[test] ERRORE: webcam {args.camera} non apribile")
        sys.exit(1)

    show_mask = True
    last = time.time()
    fps = 0.0
    print("[test] avviato. q/ESC esci | m maschera | s screenshot")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[test] frame non letto, riprovo...")
            continue

        h, w = frame.shape[:2]
        roi_h = int(h * args.roi_height_percent)
        roi = frame[h - roi_h:h, :].copy()
        debug_roi = roi.copy()

        # --- pipeline reale ---
        mask = get_trail_mask(roi, model)
        error, lookahead = mask_to_error(
            mask,
            num_strips=args.num_strips,
            max_fit_residual_px=args.max_fit_residual_px,
            lookahead_row_fraction=args.lookahead_row_fraction,
            debug_roi=debug_roi,
        )

        # --- overlay testo ---
        now = time.time()
        dt = now - last
        last = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

        err_txt = "nan" if error is None else f"{error:+.3f}"
        look_txt = "nan" if lookahead is None else f"{lookahead:+.3f}"
        color = (0, 0, 255) if error is None else (0, 255, 0)
        cv2.putText(debug_roi, f"trail_error: {err_txt}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(debug_roi, f"lookahead:   {look_txt}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(debug_roi, f"fps: {fps:4.1f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        # barra centrale + indicatore errore
        cx = w // 2
        cv2.line(debug_roi, (cx, 0), (cx, roi_h), (200, 200, 200), 1)
        if error is not None:
            ex = int(cx + error * (w / 2))
            cv2.arrowedLine(debug_roi, (cx, roi_h - 20), (ex, roi_h - 20),
                            (0, 255, 255), 3, tipLength=0.3)

        cv2.imshow("debug_view (pipeline reale)", debug_roi)
        if show_mask:
            cv2.imshow("mask", mask)

        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("m"):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("mask")
        elif k == ord("s"):
            ts = int(time.time())
            cv2.imwrite(f"debug_{ts}.png", debug_roi)
            cv2.imwrite(f"mask_{ts}.png", mask)
            print(f"[test] salvato debug_{ts}.png + mask_{ts}.png")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
