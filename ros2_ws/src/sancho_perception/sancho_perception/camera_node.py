"""
Camera Node - Versione con Mini-Server Web MJPEG Integrato
----------------------------------------------------------
Rileva la linea di nastro tramite colore LAB + CLAHE e pubblica:
  - /trail_error            (Float32): Errore laterale per il PID
  - /trail_lookahead_error  (Float32): Errore anticipato per la velocità
  - /camera/mask_view       (Image):   Streaming maschera binaria (ROS2)
  - /camera/debug_view      (Image):   Streaming video con overlay  (ROS2)

Streaming HTTP MJPEG (visualizzabile direttamente in Chrome):
  - http://localhost:8080/          → Pagina indice con entrambi i feed
  - http://localhost:8080/debug     → Feed video a colori con overlay grafico
  - http://localhost:8080/mask      → Feed maschera binaria

Autore: Visual Tutor Refactoring
"""

import threading
import time
import subprocess
import re
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# ──────────────────────────────────────────────────────────────────────────────
#  BUFFER THREAD-SAFE per i frame JPEG
# ──────────────────────────────────────────────────────────────────────────────

class FrameStore:
    """
    Mantiene l'ultimo frame JPEG codificato per ciascun canale.
    Scrittura dal thread ROS2, lettura dal thread HTTP: accesso protetto da Lock.
    """
    def __init__(self):
        self._lock   = threading.Lock()
        self._frames = {}           # { 'debug': bytes, 'mask': bytes }

    def update(self, channel: str, bgr_or_gray: np.ndarray, quality: int = 70):
        """Codifica il frame in JPEG e lo salva in modo thread-safe."""
        ret, buf = cv2.imencode('.jpg', bgr_or_gray, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ret:
            with self._lock:
                self._frames[channel] = buf.tobytes()

    def get(self, channel: str) -> bytes | None:
        """Restituisce l'ultimo JPEG disponibile per il canale richiesto."""
        with self._lock:
            return self._frames.get(channel)


# ──────────────────────────────────────────────────────────────────────────────
#  MINI-SERVER HTTP MJPEG
# ──────────────────────────────────────────────────────────────────────────────

# Pagina HTML indice: mostra entrambi i feed affiancati
_INDEX_HTML = """\
<!DOCTYPE html>
<html lang="it">
<head>
  <meta charset="UTF-8">
  <title>Camera Node – Live View</title>
  <style>
    body  {{ background:#111; color:#eee; font-family:sans-serif;
             display:flex; flex-direction:column; align-items:center; padding:20px; }}
    h1   {{ margin-bottom:4px; }}
    .feeds {{ display:flex; gap:20px; flex-wrap:wrap; justify-content:center; }}
    .feed  {{ display:flex; flex-direction:column; align-items:center; gap:6px; }}
    img  {{ border:2px solid #555; border-radius:6px; max-width:640px; width:100%; }}
    a    {{ color:#7af; }}
  </style>
</head>
<body>
  <h1>Camera Node – Live View</h1>
  <p>Feed aggiornati in tempo reale via MJPEG &mdash; nessun plugin richiesto.</p>
  <div class="feeds">
    <div class="feed">
      <span>Debug (colori + overlay)</span>
      <img src="/debug" alt="debug feed">
      <a href="/debug" target="_blank">apri standalone →</a>
    </div>
    <div class="feed">
      <span>Maschera binaria</span>
      <img src="/mask" alt="mask feed">
      <a href="/mask" target="_blank">apri standalone →</a>
    </div>
  </div>
</body>
</html>
"""

_BOUNDARY = b"--mjpegframe"


def _make_handler(store: FrameStore):
    """Crea la classe handler HTTP con riferimento allo store condiviso."""

    class _Handler(BaseHTTPRequestHandler):

        # Sopprime i log di accesso nella console (opzionale: metti `pass` per riattivarli)
        def log_message(self, fmt, *args):  # noqa: N802
            pass

        def do_GET(self):  # noqa: N802
            if self.path == '/':
                self._serve_index()
            elif self.path in ('/debug', '/mask'):
                channel = self.path.lstrip('/')
                self._serve_mjpeg(channel)
            else:
                self.send_error(404, "Not found")

        # ── Pagina indice HTML ──────────────────────────────────────────────
        def _serve_index(self):
            body = _INDEX_HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        # ── Stream MJPEG (multipart/x-mixed-replace) ───────────────────────
        def _serve_mjpeg(self, channel: str):
            self.send_response(200)
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={_BOUNDARY.decode()}"
            )
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()

            try:
                while True:
                    jpeg = store.get(channel)
                    if jpeg is None:
                        # Nessun frame disponibile: aspetta un tick e riprova
                        time.sleep(0.05)
                        continue

                    # Intestazione di ogni parte MJPEG
                    header = (
                        f"{_BOUNDARY.decode()}\r\n"
                        f"Content-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpeg)}\r\n"
                        f"\r\n"
                    ).encode()

                    self.wfile.write(header + jpeg + b"\r\n")
                    self.wfile.flush()

                    # ~30 fps al massimo, ma non bloccante per il nodo ROS2
                    time.sleep(0.033)

            except (BrokenPipeError, ConnectionResetError):
                # Il browser ha chiuso la connessione: uscita pulita
                pass

    return _Handler


class MjpegServer:
    """
    Lancia il server HTTP in un thread daemon.
    Si avvia con .start() e si ferma automaticamente quando il processo termina.
    """

    def __init__(self, store: FrameStore, host: str = "0.0.0.0", port: int = 8080):
        self._store  = store
        self._host   = host
        self._port   = port
        self._server = HTTPServer((host, port), _make_handler(store))
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._server.shutdown()


# ──────────────────────────────────────────────────────────────────────────────
#  TUNNEL CLOUDFLARE (opzionale, avviato automaticamente se cloudflared è installato)
# ──────────────────────────────────────────────────────────────────────────────

class CloudflareTunnel:
    """
    Avvia cloudflared in background e cattura l'URL pubblico dal suo output.
    Se cloudflared non è installato, fallisce silenziosamente senza bloccare il nodo.
    """

    def __init__(self, port: int, on_url=None):
        """
        port   : porta locale da esporre
        on_url : callback chiamata con l'URL pubblico appena disponibile
        """
        self._port    = port
        self._on_url  = on_url
        self._process = None
        self._thread  = None

    def start(self):
        try:
            self._process = subprocess.Popen(
                ['cloudflared', 'tunnel', '--url', f'http://localhost:{self._port}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,   # cloudflared scrive su stderr
                text=True
            )
            self._thread = threading.Thread(target=self._watch_output, daemon=True)
            self._thread.start()
        except FileNotFoundError:
            # cloudflared non installato: ignora silenziosamente
            pass

    def _watch_output(self):
        """Legge l'output di cloudflared riga per riga cercando l'URL pubblico."""
        url_pattern = re.compile(r'https://[a-z0-9\-]+\.trycloudflare\.com')
        for line in self._process.stdout:
            match = url_pattern.search(line)
            if match and self._on_url:
                self._on_url(match.group(0))
                # Una volta trovato l'URL non serve continuare a parsare
                break

    def stop(self):
        if self._process:
            self._process.terminate()


# ──────────────────────────────────────────────────────────────────────────────
#  FUNZIONI DI VISIONE (invariate rispetto alla versione precedente)
# ──────────────────────────────────────────────────────────────────────────────

def lab_mask(roi_bgr, *,
             lab_lower, lab_upper,
             clahe_clip, clahe_tile,
             morph_k,
             min_total_mask_area,
             min_contour_area,
             min_solidity,
             min_tape_width_px,
             min_elongation):
    """
    1. FUNZIONE DI VISIONE: Converte in LAB, applica CLAHE, esegue la soglia colore
    ed esclude i blob di rumore direttamente in questa fase.
    Restituisce una maschera binaria già perfettamente pulita.
    """
    lab = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_tile, clahe_tile))
    l_eq  = clahe.apply(l_ch)
    lab_eq = cv2.merge([l_eq, a_ch, b_ch])

    raw_mask = cv2.inRange(lab_eq, lab_lower, lab_upper)
    kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))
    raw_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN,  kernel)
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
        _, _, w, _ = cv2.boundingRect(cnt)
        if w < min_tape_width_px:
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
    """
    h, w    = clean_mask.shape
    half_w  = w / 2.0
    strip_h = h // num_strips
    centroids = []

    for i in range(num_strips):
        y0 = h - (i + 1) * strip_h
        y1 = h - i * strip_h
        strip_fragment = clean_mask[y0:y1, :]
        contours, _ = cv2.findContours(strip_fragment, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
            y_line = h - i * strip_h
            cv2.line(debug_roi, (0, y_line), (w, y_line), (100, 100, 100), 1)

    if len(centroids) < 2:
        return None, None

    pts  = np.array(centroids)
    a, b = np.polyfit(pts[:, 1], pts[:, 0], 1)

    residual = float(np.mean(np.abs(a * pts[:, 1] + b - pts[:, 0])))
    if residual > max_fit_residual_px:
        return None, None

    x_bottom = a * h + b
    error    = float(np.clip((x_bottom - half_w) / half_w, -1.0, 1.0))

    top_detected_y = float(pts[:, 1].min())
    lookahead_y    = max(h * lookahead_row_fraction, top_detected_y)
    x_lookahead    = a * lookahead_y + b
    lookahead_err  = float(np.clip((x_lookahead - half_w) / half_w, -1.0, 1.0))

    if debug_roi is not None:
        cv2.line(debug_roi,   (int(b), 0), (int(x_bottom), h), (0, 255, 255), 2)
        cv2.circle(debug_roi, (int(x_lookahead), int(lookahead_y)), 8, (0, 165, 255), -1)

    return error, lookahead_err


# ──────────────────────────────────────────────────────────────────────────────
#  NODO ROS2
# ──────────────────────────────────────────────────────────────────────────────

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ── Parametri ROI e Frequenza ──────────────────────────────────────
        self.roi_height_percent = float(self.declare_parameter('roi_height_percent', 0.70).value)
        self.publish_rate_hz    = float(self.declare_parameter('publish_rate_hz', 30.0).value)

        # ── Parametri Spazio Colore LAB ────────────────────────────────────
        lab_a_min = int(self.declare_parameter('lab_a_min', 100).value)
        lab_a_max = int(self.declare_parameter('lab_a_max', 145).value)
        lab_b_min = int(self.declare_parameter('lab_b_min',  60).value)
        lab_b_max = int(self.declare_parameter('lab_b_max', 115).value)
        self.lab_lower = np.array([0,   lab_a_min, lab_b_min], dtype=np.uint8)
        self.lab_upper = np.array([255, lab_a_max, lab_b_max], dtype=np.uint8)

        # ── Parametri Filtri Immagine ──────────────────────────────────────
        self.clahe_clip = float(self.declare_parameter('clahe_clip', 2.0).value)
        self.clahe_tile = int(self.declare_parameter('clahe_tile', 8).value)
        self.morph_k    = int(self.declare_parameter('morph_kernel_size', 5).value)

        # ── Parametri Qualità Blob ─────────────────────────────────────────
        self.min_contour_area    = int(self.declare_parameter('min_contour_area', 500).value)
        self.min_solidity        = float(self.declare_parameter('min_solidity', 0.60).value)
        self.min_tape_width_px   = int(self.declare_parameter('min_tape_width_px', 15).value)
        self.min_elongation      = float(self.declare_parameter('min_elongation', 2.5).value)
        self.min_total_mask_area = int(self.declare_parameter('min_total_mask_area', 3000).value)

        # ── Parametri Fitting Linea ────────────────────────────────────────
        self.num_strips             = int(self.declare_parameter('num_roi_strips', 3).value)
        self.max_fit_residual_px    = float(self.declare_parameter('max_fit_residual_px', 30.0).value)
        self.lookahead_row_fraction = float(self.declare_parameter('lookahead_row_fraction', 0.0).value)

        # ── Parametri Smoothing EMA e Debounce ────────────────────────────
        self.ema_alpha           = float(self.declare_parameter('ema_alpha', 0.3).value)
        self.lost_trail_patience = int(self.declare_parameter('lost_trail_patience', 5).value)

        self.smoothed_error         = 0.0
        self.smoothed_lookahead_err = 0.0
        self.consecutive_lost       = 0

        # ── Mini-Server Web MJPEG ──────────────────────────────────────────
        web_host = str(self.declare_parameter('web_host', '0.0.0.0').value)
        web_port = int(self.declare_parameter('web_port', 8080).value)
        self._frame_store  = FrameStore()
        self._mjpeg_server = MjpegServer(self._frame_store, host=web_host, port=web_port)
        self._mjpeg_server.start()
        self.get_logger().info(
            f'Mini-server MJPEG attivo → http://localhost:{web_port}/ '
            f'(debug: /debug | maschera: /mask)'
        )

        # ── Tunnel Cloudflare automatico (se cloudflared è installato) ────
        def _log_tunnel_url(url: str):
            self.get_logger().info(
                f'\n  ╔══════════════════════════════════════════════╗\n'
                f'  ║  LINK PUBBLICO CAMERA → {url}\n'
                f'  ╚══════════════════════════════════════════════╝'
            )
        self._tunnel = CloudflareTunnel(port=web_port, on_url=_log_tunnel_url)
        self._tunnel.start()

        # ── CvBridge e Publishers ROS2 ────────────────────────────────────
        self.bridge         = CvBridge()
        self.error_pub      = self.create_publisher(Float32, 'trail_error', 1)
        self.lookahead_pub  = self.create_publisher(Float32, 'trail_lookahead_error', 1)
        self.mask_view_pub  = self.create_publisher(Image, 'camera/mask_view', 1)
        self.debug_view_pub = self.create_publisher(Image, 'camera/debug_view', 1)

        # ── Videocamera ────────────────────────────────────────────────────
        cam_idx = int(self.declare_parameter('camera_index', 0).value)
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(self.declare_parameter('frame_width',  640).value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.declare_parameter('frame_height', 480).value))

        if not self.cap.isOpened():
            self.get_logger().error('Impossibile accedere alla webcam!')
            raise RuntimeError('Camera open failed')

        # ── Timer principale della pipeline ───────────────────────────────
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.process_frame)
        self.get_logger().info('CameraNode avviato con successo [Pipeline Pulita + Web MJPEG + ROS2 Streaming]')

    # ──────────────────────────────────────────────────────────────────────
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame non letto correttamente dalla sorgente video')
            return

        h, w   = frame.shape[:2]
        roi_y0 = int(h * (1.0 - self.roi_height_percent))
        roi    = frame[roi_y0:, :].copy()

        # 1. Maschera colore filtrata
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
            min_elongation      = self.min_elongation
        )

        # Frame di debug con overlay grafico
        debug_frame = frame.copy()
        debug_roi   = debug_frame[roi_y0:, :]
        cv2.rectangle(debug_frame, (0, roi_y0), (w - 1, h - 1), (0, 165, 255), 2)
        cv2.line(debug_frame,      (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1)

        # 2. Calcolo errori geometrici
        error_raw, lookahead_err_raw = mask_to_error(
            clean_mask,
            num_strips             = self.num_strips,
            max_fit_residual_px    = self.max_fit_residual_px,
            lookahead_row_fraction = self.lookahead_row_fraction,
            debug_roi              = debug_roi
        )

        # Smoothing EMA + debounce tracciato perso
        if error_raw is not None:
            if self.consecutive_lost > self.lost_trail_patience:
                self.smoothed_error         = error_raw
                self.smoothed_lookahead_err = lookahead_err_raw
            else:
                self.smoothed_error = (
                    self.ema_alpha * error_raw +
                    (1.0 - self.ema_alpha) * self.smoothed_error
                )
                self.smoothed_lookahead_err = (
                    self.ema_alpha * lookahead_err_raw +
                    (1.0 - self.ema_alpha) * self.smoothed_lookahead_err
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

        # ── Pubblicazione topic numerici ROS2 ─────────────────────────────
        err_msg  = Float32();  err_msg.data  = error
        look_msg = Float32();  look_msg.data = lookahead_err
        self.error_pub.publish(err_msg)
        self.lookahead_pub.publish(look_msg)

        # ── Aggiornamento buffer per il mini-server web ────────────────────
        self._frame_store.update('debug', debug_frame, quality=70)
        self._frame_store.update('mask',  clean_mask,  quality=70)

        # ── Pubblicazione topic immagine ROS2 ─────────────────────────────
        try:
            mask_msg  = self.bridge.cv2_to_imgmsg(clean_mask,   encoding="mono8")
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            self.mask_view_pub.publish(mask_msg)
            self.debug_view_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Errore durante lo streaming ROS2 delle immagini: {e}")

    def destroy_node(self):
        self._tunnel.stop()
        self._mjpeg_server.stop()
        self.cap.release()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────

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