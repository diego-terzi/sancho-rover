"""
SANCHO App Lab Python — UDP-to-Bridge shim.

Why this exists:
  Our ROS 2 stack (camera_node, controller_node, motor_bridge_node) lives in
  a Docker container because ROS 2 Jazzy doesn't have packages for the
  QRB2210's Debian Trixie. The Arduino_RouterBridge Python client, on the
  other hand, is part of Arduino's per-app virtualenvs and is hard to import
  from inside Docker without dragging in App Lab's whole environment.

  This shim resolves the impedance mismatch:
    - Listens for UDP datagrams from Docker on 127.0.0.1:9001
    - Each datagram is 2 little-endian int16: [left_pwm, right_pwm]
    - Forwards each one to the MCU as Bridge.notify("set_motors", L, R)

  motor_bridge_node (Python in Docker) keeps its full safety logic
  (kinematics, watchdog, telemetry); the only thing it offloads is the
  final hop to the MCU.

Run from Arduino App Lab as a normal app — the App Lab framework provides
the arduino.app_utils module via its per-app venv.
"""

import socket
import struct

from arduino.app_utils import App, Bridge

UDP_HOST = "127.0.0.1"
UDP_PORT = 9001
PACKET_FMT = "<hh"               # 2 little-endian signed shorts: left, right
PACKET_LEN = struct.calcsize(PACKET_FMT)
RECV_TIMEOUT_S = 1.0              # blocking-ish recv; idle when no traffic

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_sock.bind((UDP_HOST, UDP_PORT))
_sock.settimeout(RECV_TIMEOUT_S)
print(f"[sancho_bridge] UDP listener up on {UDP_HOST}:{UDP_PORT}, "
      f"forwarding to MCU via Bridge.notify('set_motors', ...)")


def loop():
    try:
        data, _ = _sock.recvfrom(64)
    except socket.timeout:
        # No traffic — let the MCU's own 500 ms watchdog handle motor stop.
        return

    if len(data) < PACKET_LEN:
        return

    left, right = struct.unpack(PACKET_FMT, data[:PACKET_LEN])
    # Notify (fire-and-forget) is the right semantic for periodic motor
    # updates: we don't need an ack, the next packet will overwrite, and
    # if the MCU misses one we'll send another in 50 ms.
    Bridge.notify("set_motors", int(left), int(right))


App.run(user_loop=loop)
