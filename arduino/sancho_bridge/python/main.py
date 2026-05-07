"""
SANCHO App Lab Python — bidirectional UDP-Bridge shim.

Two flows:
  1. Motor commands     (Docker → MCU)        UDP :9001 → Bridge.notify(...)
  2. Ultrasonic readings (MCU → Docker)       Bridge handler → UDP :9002

Flow 1 is rock-solid: Bridge.notify() is a known API. Flow 2 needs the
Python side to register a handler for events the MCU sends via
Bridge.notify("distance_cm", value) — and the exact name of that
registration method on the Python side is undocumented in the App Lab
material we have. So this script does runtime discovery: it tries the
most likely candidate method names; if none work, the motor flow keeps
running and we just lose obstacle detection (the rover drives but won't
auto-stop in front of obstacles).

Run from Arduino App Lab as a normal app — the framework provides
arduino.app_utils via its per-app venv.
"""

import socket
import struct

from arduino.app_utils import App, Bridge


LISTEN_HOST    = "0.0.0.0"
MOTOR_PORT     = 9001
SENSOR_PORT    = 9002

MOTOR_FMT      = "<hh"
MOTOR_LEN      = struct.calcsize(MOTOR_FMT)
DISTANCE_FMT   = "<H"
RECV_TIMEOUT_S = 1.0


_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_recv_sock.bind((LISTEN_HOST, MOTOR_PORT))
_recv_sock.settimeout(RECV_TIMEOUT_S)

_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

_ros_ip = None  # learned from incoming motor packets


def on_distance_cm(cm):
    """Bridge handler invoked by the MCU's notify("distance_cm", value)."""
    if _ros_ip is None:
        return
    try:
        cm_clamped = max(0, min(65535, int(cm)))
        _send_sock.sendto(
            struct.pack(DISTANCE_FMT, cm_clamped),
            (_ros_ip, SENSOR_PORT),
        )
    except Exception as e:
        print(f"[sancho_bridge] sensor forward failed: {e}")


# ── Bridge API discovery ─────────────────────────────────────────────────────
# Print every public method on the Bridge object so we can see, in App Lab's
# log, what's actually available on this machine.
_bridge_api = sorted(m for m in dir(Bridge) if not m.startswith('_'))
print(f"[sancho_bridge] Bridge methods available: {_bridge_api}")

# Try a list of plausible names for "register a Python handler for an MCU
# notification". Whichever one works first wins; the others stay untouched.
_registered_via = None
for method_name in ('provide', 'subscribe', 'on', 'handle', 'register',
                    'add_handler', 'add_listener', 'notify_handler',
                    'provide_safe', 'register_callback'):
    method = getattr(Bridge, method_name, None)
    if not callable(method):
        continue
    try:
        method("distance_cm", on_distance_cm)
        _registered_via = method_name
        break
    except Exception as e:
        print(f"[sancho_bridge] Bridge.{method_name}() raised: {e}")

if _registered_via:
    print(f"[sancho_bridge] distance_cm handler registered via Bridge.{_registered_via}()")
else:
    print("[sancho_bridge] WARNING: no working register API found — sensor flow disabled")
    print("[sancho_bridge] Motor flow continues normally; OBSTACLE_STOP will not trigger.")

# ── Main motor-forwarding loop ───────────────────────────────────────────────

print(f"[sancho_bridge] shim up | motors :{MOTOR_PORT} (Docker→MCU) | "
      f"sensors :{SENSOR_PORT} (MCU→Docker, dest auto-learned)")


def loop():
    global _ros_ip
    try:
        data, addr = _recv_sock.recvfrom(64)
    except socket.timeout:
        return
    if len(data) < MOTOR_LEN:
        return
    _ros_ip = addr[0]
    left, right = struct.unpack(MOTOR_FMT, data[:MOTOR_LEN])
    Bridge.notify("set_motors", int(left), int(right))


App.run(user_loop=loop)
