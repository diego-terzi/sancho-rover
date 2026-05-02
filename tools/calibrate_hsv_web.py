#!/usr/bin/env python3
"""
SANCHO HSV calibration over the web — for use directly on the rover.

Run on the Arduino UNO Q (or any Linux host with the C270 attached):

    python3 tools/calibrate_hsv_web.py [camera_index]    (default: 0)

Then on any device on the same network — your laptop, your phone — open

    http://<unoq-ip>:8080

You'll see the live camera with the HSV mask overlay, plus a sidebar with
sliders for the HSV bounds and the ROI percentage. Drag sliders → mask
updates live. When the trail looks isolated (white in mask, black background),
click "Copy YAML" — values are formatted ready to paste into
ros2_ws/src/sancho_bringup/config/sancho_params.yaml.

Stop with Ctrl+C.

Notes:
  - The camera is exclusive: you must stop the ROS 2 stack
    (i.e. Ctrl+C the docker run) before running this. Otherwise OpenCV here
    will fail to open the device.
  - The web stream is MJPEG over plain HTTP — no auth. Use only on your
    local Wi-Fi.
  - Find the UNO Q's IP with `hostname -I` on the UNO Q itself.
"""

import sys
import time
import threading

import cv2
import numpy as np

try:
    from flask import Flask, Response, request, jsonify
except ImportError:
    print("ERROR: Flask not installed.")
    print("  sudo apt install python3-flask")
    print("  or: pip3 install --break-system-packages flask")
    sys.exit(1)


CAM_INDEX = int(sys.argv[1]) if len(sys.argv) > 1 else 0
PORT = 8080

# ── shared state, mutated by HTTP, read by camera thread ────────────────────
_lock = threading.Lock()
_params = {
    'h_min': 0,   'h_max': 180,
    's_min': 0,   's_max': 255,
    'v_min': 0,   'v_max': 255,
    'roi_pct': 40,
}
_latest = {'overlay': None, 'mask': None}

# ── camera capture ──────────────────────────────────────────────────────────
_cap = cv2.VideoCapture(CAM_INDEX)
if not _cap.isOpened():
    print(f"ERROR: cannot open camera index {CAM_INDEX}")
    print("If the ROS 2 stack is running, stop it first (Ctrl+C the docker run).")
    sys.exit(1)
_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


def _camera_loop():
    while True:
        ok, frame = _cap.read()
        if not ok:
            time.sleep(0.05)
            continue

        with _lock:
            p = dict(_params)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([p['h_min'], p['s_min'], p['v_min']]),
            np.array([p['h_max'], p['s_max'], p['v_max']]),
        )

        overlay = np.zeros_like(frame)
        overlay[mask > 0] = (0, 200, 0)  # green where the mask is hot
        blended = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)

        # ROI boundary line
        H = frame.shape[0]
        roi_y = int(H * (1 - p['roi_pct'] / 100.0))
        cv2.line(blended, (0, roi_y), (frame.shape[1], roi_y), (0, 165, 255), 2)
        cv2.putText(
            blended,
            f"ROI {p['roi_pct']}%   H{p['h_min']}-{p['h_max']}  S{p['s_min']}-{p['s_max']}  V{p['v_min']}-{p['v_max']}",
            (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
        )

        _latest['overlay'] = blended
        _latest['mask'] = mask


threading.Thread(target=_camera_loop, daemon=True).start()


# ── HTML page ───────────────────────────────────────────────────────────────
HTML = """<!doctype html>
<html><head><title>SANCHO HSV Calibration</title>
<style>
  body{font-family:system-ui,-apple-system,sans-serif;background:#1e1e1e;color:#ddd;margin:20px}
  h2,h3{margin:8px 0}
  .row{display:flex;gap:24px;flex-wrap:wrap}
  .col{display:flex;flex-direction:column}
  img{border:1px solid #555;background:#000;display:block}
  label{display:inline-block;width:80px;color:#bbb}
  input[type=range]{width:260px;vertical-align:middle}
  .value{display:inline-block;width:50px;text-align:right;color:#0e8;font-family:monospace}
  button{padding:10px 18px;margin-top:10px;background:#0a4;color:#fff;border:none;cursor:pointer;font-size:14px;border-radius:4px}
  button:hover{background:#0c5}
  pre{background:#000;padding:12px;border:1px solid #444;color:#0e8;border-radius:4px;white-space:pre-wrap}
  .controls{padding:16px;background:#252525;border-radius:6px}
  .row > div{display:inline-block}
</style></head>
<body>
  <h2>SANCHO — HSV Calibration <span style="color:#888;font-size:14px">(camera %d)</span></h2>
  <div class="row">
    <div class="col">
      <h3>Camera + mask overlay</h3>
      <img src="/video" width="640" height="480">
    </div>
    <div class="col">
      <h3>Mask only</h3>
      <img src="/mask" width="320" height="240">
      <div class="controls">
        <h3>HSV Bounds + ROI</h3>
        <div><label>H min</label><input id=h_min type=range min=0 max=180 value=0 oninput=u()><span id=h_min_v class=value>0</span></div>
        <div><label>H max</label><input id=h_max type=range min=0 max=180 value=180 oninput=u()><span id=h_max_v class=value>180</span></div>
        <div><label>S min</label><input id=s_min type=range min=0 max=255 value=0 oninput=u()><span id=s_min_v class=value>0</span></div>
        <div><label>S max</label><input id=s_max type=range min=0 max=255 value=255 oninput=u()><span id=s_max_v class=value>255</span></div>
        <div><label>V min</label><input id=v_min type=range min=0 max=255 value=0 oninput=u()><span id=v_min_v class=value>0</span></div>
        <div><label>V max</label><input id=v_max type=range min=0 max=255 value=255 oninput=u()><span id=v_max_v class=value>255</span></div>
        <div><label>ROI %</label><input id=roi_pct type=range min=5 max=100 value=40 oninput=u()><span id=roi_pct_v class=value>40</span></div>
        <button onclick=cp()>Copy YAML to clipboard</button>
        <pre id=out>(adjust sliders, then click "Copy YAML")</pre>
      </div>
    </div>
  </div>
<script>
const KS=['h_min','h_max','s_min','s_max','v_min','v_max','roi_pct'];
function u(){
  const o={};
  KS.forEach(k=>{o[k]=parseInt(document.getElementById(k).value);document.getElementById(k+'_v').textContent=o[k];});
  fetch('/params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)});
}
function cp(){
  const v={};KS.forEach(k=>v[k]=document.getElementById(k).value);
  const yaml = `    hsv_lower: [${v.h_min}, ${v.s_min}, ${v.v_min}]\\n` +
               `    hsv_upper: [${v.h_max}, ${v.s_max}, ${v.v_max}]\\n` +
               `    roi_height_percent: ${(v.roi_pct/100).toFixed(2)}`;
  document.getElementById('out').textContent = yaml;
  navigator.clipboard.writeText(yaml).catch(()=>{});
}
</script>
</body></html>
""" % CAM_INDEX


# ── Flask routes ────────────────────────────────────────────────────────────
app = Flask(__name__)


@app.route('/')
def root():
    return HTML


@app.route('/params', methods=['POST'])
def update_params():
    data = request.get_json(force=True)
    with _lock:
        for key in _params:
            if key in data:
                _params[key] = int(data[key])
    return jsonify(ok=True)


def _mjpeg_iter(key, gray_to_bgr=False):
    while True:
        frame = _latest.get(key)
        if frame is None:
            time.sleep(0.05)
            continue
        if gray_to_bgr:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            continue
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n'
            + jpeg.tobytes() + b'\r\n'
        )
        time.sleep(0.040)  # ~25 fps cap


@app.route('/video')
def video():
    return Response(_mjpeg_iter('overlay'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/mask')
def mask():
    return Response(_mjpeg_iter('mask', gray_to_bgr=True),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def _print_banner():
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        ip = "<this-machine-ip>"
    print()
    print("=" * 60)
    print(f"  SANCHO HSV calibration — camera {CAM_INDEX}")
    print(f"  Open in browser:  http://{ip}:{PORT}")
    print("  Ctrl+C to stop.")
    print("=" * 60)
    print()


if __name__ == '__main__':
    _print_banner()
    app.run(host='0.0.0.0', port=PORT, threaded=True)
