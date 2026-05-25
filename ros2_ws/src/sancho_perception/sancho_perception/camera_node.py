"""
Camera Node - Yolo_Trail_Test branch
--------------------------------------
Rileva la linea di nastro BLU tramite YOLOv8 instance segmentation (ONNX locale) e pubblica:
  - /trail_error            (Float32): Errore laterale per il PID
  - /trail_lookahead_error  (Float32): Errore anticipato per la velocità
  - /camera/mask_view       (Image): Streaming della maschera binaria pulita
  - /camera/debug_view      (Image): Streaming video con overlay grafico (linee, punti)

Dipendenza: pip install onnxruntime  (~16 MB, ARM64 supportato)
Modello:    YOLOv8 instance segmentation, 1 classe (blue_line)
            Input:   [1, 3, 640, 640] float32
            output0: [1, 37, 8400]    — bbox + conf + 32 mask coefficients
            output1: [1, 32, 160, 160] — prototype masks
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np

_YOLO_INPUT_SIZE  = 640
_CONF_THRESHOLD   = 0.25  # TODO: tune — abbassa se non rileva, alza se troppi falsi positivi
_MASK_THRESHOLD   = 0.5


def get_trail_mask(roi_bgr, model):
    """
    Riceve il ROI BGR e la sessione ONNX YOLOv8-seg caricata.
    Restituisce una maschera binaria (uint8) dove 255 = nastro blu, 0 = sfondo.
    """
    h, w = roi_bgr.shape[:2]
    if model is None:
        return np.zeros((h, w), dtype=np.uint8)

    # Pre-processing: 640x640, BGR→RGB, normalizza [0,1], batch dim
    img = cv2.resize(roi_bgr, (_YOLO_INPUT_SIZE, _YOLO_INPUT_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))[np.newaxis]  # [1, 3, 640, 640]

    # Inferenza
    output0, output1 = model.run(None, {'images': img})
    # output0: [1, 37, 8400]    — [x,y,w,h, conf, coef*32] per anchor
    # output1: [1, 32, 160, 160] — prototype masks

    preds  = output0[0]   # [37, 8400]
    protos = output1[0]   # [32, 160, 160]

    # Filtra per confidenza (indice 4 = classe 0: blue_line)
    keep = preds[4] > _CONF_THRESHOLD
    if not np.any(keep):
        return np.zeros((h, w), dtype=np.uint8)

    # Mask coefficients per le detection tenute: [32, N]
    coefs = preds[5:, keep]

    # Calcola instance masks: sigmoid(coefs.T @ protos_flat) → [N, 160, 160]
    protos_flat = protos.reshape(32, -1)                    # [32, 25600]
    raw   = coefs.T @ protos_flat                           # [N, 25600]
    masks = 1.0 / (1.0 + np.exp(-raw))                     # sigmoid
    masks = masks.reshape(-1, 160, 160)                     # [N, 160, 160]

    # Unione di tutte le istanze → maschera binaria → resize al ROI originale
    combined = np.any(masks > _MASK_THRESHOLD, axis=0).astype(np.uint8) * 255
    return cv2.resize(combined, (w, h), interpolation=cv2.INTER_NEAREST)


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

        # Caricamento modello ONNX (onnxruntime)
        model_path = str(self.declare_parameter(
            'model_path',
            'models/trail_detector.onnx'
        ).value)

        # Risolve path relativo rispetto alla share directory del pacchetto
        if not os.path.isabs(model_path):
            model_path = os.path.join(
                get_package_share_directory('sancho_perception'),
                model_path
            )

        try:
            import onnxruntime as ort
            self.model = ort.InferenceSession(
                model_path, providers=['CPUExecutionProvider']
            )
            self.get_logger().info(f'Modello ONNX caricato: {model_path}')
        except Exception as e:
            self.get_logger().warn(f'Modello ONNX non caricato ({e}) — rilevazione disabilitata')
            self.model = None

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

        # 1. Segmentazione istanza: maschera binaria dal modello YOLOv8-seg ONNX
        clean_mask = get_trail_mask(roi, self.model)

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