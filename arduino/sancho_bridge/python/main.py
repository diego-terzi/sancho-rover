"""
SANCHO App Lab Python — bidirectional UDP-Bridge shim.

Why this exists:
  Our ROS 2 stack (camera_node, controller_node, motor_bridge_node, sensor_node)
  lives in a Docker container because ROS 2 Jazzy doesn't have packages for the
  QRB2210's Debian Trixie. The Arduino_RouterBridge Python client, on the other
  hand, is part of Arduino's per-app virtualenvs and is hard to import from
  inside Docker without dragging in App Lab's whole environment.

  This shim handles two flows:

    1. Motor commands  (Docker → MCU)
       UDP datagram on :9001  →  Bridge.notify("set_motors", L, R)
       Packet: 2× int16 little-endian = [left_pwm, right_pwm]

    2. Ultrasonic readings (MCU → Docker)
       Bridge.provide_safe("distance_cm")  →  UDP datagram to ROS host:9002
       Packet: 1× uint16 little-endian = [distance_cm]

  The destination address for flow (2) is *learned* from the source address of
  flow (1) — every motor packet from Docker tells us where to reply. This means
  no static IP wiring even if the Docker container restarts.

Run from Arduino App Lab as a normal app — the App Lab framework provides the
arduino.app_utils module via its per-app venv.
"""

import socket
import struct

from arduino.app_utils import App, Bridge


LISTEN_HOST       = "0.0.0.0"  # bind to all interfaces — App Lab runs us in its own container
MOTOR_PORT        = 9001
SENSOR_PORT       = 9002       # destination port on the ROS container

MOTOR_FMT         = "<hh"      # 2 little-endian signed shorts: left, right
MOTOR_LEN         = struct.calcsize(MOTOR_FMT)
DISTANCE_FMT      = "<H"       # 1 little-endian unsigned short: distance in cm
RECV_TIMEOUT_S    = 1.0


_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_recv_sock.bind((LISTEN_HOST, MOTOR_PORT))
_recv_sock.settimeout(RECV_TIMEOUT_S)

_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# IP address of the ROS container, learned from incoming motor packets so we
# don't need to hard-code Docker's bridge-network address.
_ros_ip = None


def on_distance_cm(cm):
    """Bridge handler invoked by the MCU's notify("distance_cm", value)."""
    if _ros_ip is None:
        return  # no ROS peer known yet — drop silently until first motor packet arrives
    try:
        cm_clamped = max(0, min(65535, int(cm)))
        _send_sock.sendto(
            struct.pack(DISTANCE_FMT, cm_clamped),
            (_ros_ip, SENSOR_PORT),
        )
    except Exception as e:
        print(f"[sancho_bridge] sensor forward failed: {e}")


# Register the handler for MCU notifications. App Lab's Bridge uses the same
# provide_safe() name on both sides — Python provides a function the MCU calls
# via Bridge.notify("distance_cm", value). This is the mirror of how the MCU
# provides set_motors and Python notifies it.
Bridge.provide_safe("distance_cm", on_distance_cm)


print(f"[sancho_bridge] bidirectional UDP shim up | "
      f"motors :{MOTOR_PORT} (Docker→MCU) | "
      f"sensors :{SENSOR_PORT} (MCU→Docker, dest auto-learned)")


def loop():
    global _ros_ip
    try:
        data, addr = _recv_sock.recvfrom(64)
    except socket.timeout:
        # No traffic — let the MCU's own 500 ms watchdog handle motor stop.
        return

    if len(data) < MOTOR_LEN:
        return

    _ros_ip = addr[0]  # remember the ROS container's IP for the sensor reply path
    left, right = struct.unpack(MOTOR_FMT, data[:MOTOR_LEN])
    Bridge.notify("set_motors", int(left), int(right))


App.run(user_loop=loop)
