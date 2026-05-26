"""
Camera Node - LAB branch (detector colore LAB + CLAHE)
------------------------------------------------------
Rileva la linea di nastro BLU tramite soglia di colore nello spazio LAB con
equalizzazione CLAHE (nessun modello ML) e pubblica:
  - /trail_error            (Float32): Errore laterale per il PID
  - /trail_lookahead_error  (Float32): Errore anticipato per la velocità
  - /camera/mask_view       (Image): Streaming della maschera binaria pulita
  - /camera/debug_view      (Image): Streaming video con overlay grafico (linee, punti)

Soglie LAB calibrabili dai params (lab_a_min/max, lab_b_min/max). Calibrazione
assistita: tools/test_camera_lab.py (click sul nastro -> scrive i range qui).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def lab_mask(roi_bgr, *, lab_lower, lab_upper, clahe_clip, clahe_tile, morph_k,
             min_total_mask_area, min_contour_area, min_solidity,
             min_tape_width_px, min_elongation):
    """
    Converte in LAB, applica CLAHE sul canale L, soglia il colore (canali a,b)
    e filtra i blob di rumore. Restituisce una maschera binaria già pulita
    (uint8, 255 = nastro, 0 = sfondo).
    """
    lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))
    l_eq = clahe.apply(l_ch)
    lab_eq = cv2.merge([l_eq, a_ch, b_ch])

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


def mask_to_error(clean_mask, *,
                  num_strips,
                  max_fit_residual_px,
                  lookahead_row_fraction,
                  debug_roi=None):
    """
    2. FUNZIONE GEOMETRICA: Divide la maschera pulita in strisce orizzontali,
    trova il baricentro del frammento più grande per striscia ed esegue il fit lineare.
    Se viene passato 'debug_roi', vi disegna sopra i risultati per lo streaming video.
    """
    h, w = clean_mask.shape
    half_w = w / 2.0
    strip_h = h // num_strips
    centroids = []

    for i in range(num_strips):
        y0 = h - (i + 1) * strip_h
        y1 = h - i * strip_h
        
        # Estrazione dei contorni nella singola striscia
        strip_fragment = clean_mask[y0:y1, :]
        contours, _ = cv2.findContours(strip_fragment, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            continue
            
        # Essendo la maschera già pulita, prendiamo semplicemente il pezzo più grande presente
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] == 0:
            continue
            
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        centroids.append((cx, y0 + cy))

        # Disegno di debug facoltativo (Contorni verdi e pallini rossi sui baricentri delle strisce)
        if debug_roi is not None:
            cv2.drawContours(debug_roi[y0:y1], [largest], -1, (0, 255, 0), 2)
            cv2.circle(debug_roi[y0:y1], (int(cx), int(cy)), 6, (0, 0, 255), -1)

    # Disegno delle linee di divisione delle strisce sul video di debug
    if debug_roi is not None:
        for i in range(1, num_strips):
            y_line = h - i * strip_h
            cv2.line(debug_roi, (0, y_line), (w, y_line), (100, 100, 100), 1)

    if len(centroids) < 2:
        return None, None

    # Approssimazione lineare (Linear Fitting)
    pts = np.array(centroids)
    a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)
    
    # Controllo dei residui per rigettare traiettorie incoerenti
    residual = float(np.mean(np.abs(a * pts[:, 1] + b - pts[:, 0])))
    if residual > max_fit_residual_px:
        return None, None

    # Calcolo Errore alla base della ROI
    x_bottom = a * h + b
    error = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))

    # Calcolo Errore Lookahead (Proiettato in avanti)
    top_detected_y = float(pts[:, 1].min())
    lookahead_y = max(h * lookahead_row_fraction, top_detected_y)
    x_lookahead = a * lookahead_y + b
    lookahead_err = float(np.clip((x_lookahead - half_w) / half_w, -1.0, 1.0))

    # Disegno della retta di regressione (Gialla) e del punto di lookahead (Arancione)
    if debug_roi is not None:
        cv2.line(debug_roi, (int(b), 0), (int(x_bottom), h), (0, 255, 255), 2)
        cv2.circle(debug_roi, (int(x_lookahead), int(lookahead_y)), 8, (0, 165, 255), -1)

    return error, lookahead_err


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parametri ROI e Frequenza
        self.roi_height_percent = float(self.declare_parameter('roi_height_percent', 0.70).value)
        self.publish_rate_hz    = float(self.declare_parameter('publish_rate_hz', 10.0).value)

        # Soglie colore LAB (canali a,b) — calibrabili dai params / test tool
        lab_a_min = int(self.declare_parameter('lab_a_min', 100).value)
        lab_a_max = int(self.declare_parameter('lab_a_max', 145).value)
        lab_b_min = int(self.declare_parameter('lab_b_min',  60).value)
        lab_b_max = int(self.declare_parameter('lab_b_max', 115).value)
        self.lab_lower = np.array([0,   lab_a_min, lab_b_min], dtype=np.uint8)
        self.lab_upper = np.array([255, lab_a_max, lab_b_max], dtype=np.uint8)

        # Filtri immagine (CLAHE + morfologia)
        self.clahe_clip = float(self.declare_parameter('clahe_clip', 2.0).value)
        self.clahe_tile = int(self.declare_parameter('clahe_tile', 8).value)
        self.morph_k    = int(self.declare_parameter('morph_kernel_size', 5).value)

        # Filtri qualità blob (anti-rumore)
        self.min_contour_area    = int(self.declare_parameter('min_contour_area', 500).value)
        self.min_solidity        = float(self.declare_parameter('min_solidity', 0.60).value)
        self.min_tape_width_px   = int(self.declare_parameter('min_tape_width_px', 15).value)
        self.min_elongation      = float(self.declare_parameter('min_elongation', 2.5).value)
        self.min_total_mask_area = int(self.declare_parameter('min_total_mask_area', 3000).value)

        # Parametri Fitting Linea
        self.num_strips             = int(self.declare_parameter('num_roi_strips', 3).value)
        self.max_fit_residual_px    = float(self.declare_parameter('max_fit_residual_px', 30.0).value)
        self.lookahead_row_fraction = float(self.declare_parameter('lookahead_row_fraction', 0.0).value)

        # Parametri di Smoothing EMA e Debounce
        self.ema_alpha           = float(self.declare_parameter('ema_alpha', 0.3).value)
        self.lost_trail_patience = int(self.declare_parameter('lost_trail_patience', 5).value)

        self.smoothed_error         = 0.0
        self.smoothed_lookahead_err = 0.0
        self.consecutive_lost       = 0

        # Inizializzazione CvBridge per conversione immagini ROS2 <-> OpenCV
        self.bridge = CvBridge()

        # Inizializzazione Publishers (Numerici + Immagini)
        self.error_pub     = self.create_publisher(Float32, 'trail_error', 1)
        self.lookahead_pub = self.create_publisher(Float32, 'trail_lookahead_error', 1)
        self.mask_view_pub = self.create_publisher(Image, 'camera/mask_view', 1)
        self.debug_view_pub = self.create_publisher(Image, 'camera/debug_view', 1)

        # ── Mode arbitration ──────────────────────────────────────────────
        self._active_mode = 'TRAIL'
        self.create_subscription(String, 'active_mode', self._on_active_mode, 1)

        # ── Videocamera ────────────────────────────────────────────────────
        self._cam_idx      = int(self.declare_parameter('camera_index', 0).value)
        self._frame_width  = int(self.declare_parameter('frame_width',  640).value)
        self._frame_height = int(self.declare_parameter('frame_height', 480).value)
        self.cap = cv2.VideoCapture(self._cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)

        if not self.cap.isOpened():
            self.get_logger().error('Impossibile accedere alla webcam!')
            raise RuntimeError('Camera open failed')

        # Avvio Timer Principale della Pipeline
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)
        self.get_logger().info('CameraNode avviato con successo [Pipeline Pulita + Streaming Live Attivo]')

    def _on_active_mode(self, msg: String):
        new_mode = msg.data
        if new_mode == self._active_mode:
            return
        if new_mode == 'FOLLOW':
            if self.cap.isOpened():
                self.cap.release()
            self.get_logger().info('FOLLOW mode: camera released')
        elif new_mode == 'TRAIL':
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self._cam_idx)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._frame_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)
            self.get_logger().info('TRAIL mode: camera opened')
        self._active_mode = new_mode

    # ──────────────────────────────────────────────────────────────────────
    def process_frame(self):
        if self._active_mode != 'TRAIL':
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame non letto correttamente dalla sorgente video')
            return

        # Estrazione della Region of Interest (ROI)
        h, w = frame.shape[:2]
        roi_y0 = int(h * (1.0 - self.roi_height_percent))
        roi = frame[roi_y0:, :].copy()

        # 1. Maschera colore LAB+CLAHE già filtrata dai blob estranei
        clean_mask = lab_mask(
            roi,
            lab_lower           = self.lab_lower,
            lab_upper           = self.lab_upper,
            clahe_clip          = self.clahe_clip,
            clahe_tile          = self.clahe_tile,
            morph_k             = self.morph_k,
            min_total_mask_area = self.min_total_mask_area,
            min_contour_area    = self.min_contour_area,
            min_solidity        = self.min_solidity,
            min_tape_width_px   = self.min_tape_width_px,
            min_elongation      = self.min_elongation,
        )

        # Creiamo un'area di disegno per l'overlay grafico sul frame completo
        # Passiamo la sezione ROI a `mask_to_error` in modo che possa disegnarci sopra
        debug_frame = frame.copy()
        debug_roi = debug_frame[roi_y0:, :]

        # Disegniamo il rettangolo arancione della ROI globale per feedback visivo
        cv2.rectangle(debug_frame, (0, roi_y0), (w - 1, h - 1), (0, 165, 255), 2)
        cv2.line(debug_frame, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1)

        # 2. Calcolo geometrico degli errori basandosi sulla maschera pulita
        error_raw, lookahead_err_raw = mask_to_error(
            clean_mask,
            num_strips             = self.num_strips,
            max_fit_residual_px    = self.max_fit_residual_px,
            lookahead_row_fraction = self.lookahead_row_fraction,
            debug_roi              = debug_roi  # Passato per abilitare i disegni live
        )

        # Logica di Smoothing (EMA) e contatore Debounce per tracciato perso
        if error_raw is not None:
            if self.consecutive_lost > self.lost_trail_patience:
                self.smoothed_error         = error_raw
                self.smoothed_lookahead_err = lookahead_err_raw
            else:
                self.smoothed_error = (self.ema_alpha * error_raw + 
                                       (1.0 - self.ema_alpha) * self.smoothed_error)
                self.smoothed_lookahead_err = (self.ema_alpha * lookahead_err_raw + 
                                               (1.0 - self.ema_alpha) * self.smoothed_lookahead_err)
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

        # --- PUBBLICAZIONE TOPIC NUMERICI (ROS2) ---
        err_msg = Float32();  err_msg.data = error
        look_msg = Float32(); look_msg.data = lookahead_err
        self.error_pub.publish(err_msg)
        self.lookahead_pub.publish(look_msg)

        # --- PUBBLICAZIONE LIVE VIDEO STREAMING (ROS2) ---
        try:
            # Pubblica la maschera binaria (Mono a 8-bit)
            mask_msg = self.bridge.cv2_to_imgmsg(clean_mask, encoding="mono8")
            self.mask_view_pub.publish(mask_msg)
            
            # Pubblica il frame originale a colori (BGR a 8-bit) con la grafica sovraimpressa
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            self.debug_view_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Errore durante lo streaming delle immagini: {e}")

    def destroy_node(self):
        if self.cap.isOpened():
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