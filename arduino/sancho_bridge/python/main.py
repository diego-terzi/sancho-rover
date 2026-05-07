"""
SANCHO App Lab Python — bidirectional UDP-Bridge shim.

Two flows:
  1. Motor commands     (Docker → MCU)        UDP :9001 → Bridge.notify(...)
  2. Ultrasonic readings (MCU → Docker)       Bridge.provide handler → UDP :9002

Bridge API confirmed on hardware (May 2026):
  Bridge methods are ['call', 'notify', 'provide', 'unprovide'].
    notify(name, *args)   = send fire-and-forget event to MCU
    provide(name, fn)     = register a Python handler for events from MCU
    call(name, *args)     = synchronous RPC, waits for return value
    unprovide(name)       = unregister handler

The destination address for flow (2) is learned from the source of flow (1):
every motor packet from Docker tells us where to reply, so no static IP
wiring is needed.
"""

import socket
import struct
import time

from arduino.app_utils import App, Bridge


LISTEN_HOST    = "0.0.0.0"
MOTOR_PORT     = 9001
SENSOR_PORT    = 9002

MOTOR_FMT      = "<hh"
MOTOR_LEN      = struct.calcsize(MOTOR_FMT)
DISTANCE_FMT   = "<H"
RECV_TIMEOUT_S = 1.0
HEARTBEAT_S    = 5.0


_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_recv_sock.bind((LISTEN_HOST, MOTOR_PORT))
_recv_sock.settimeout(RECV_TIMEOUT_S)

_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

_ros_ip = None
_motor_count = 0
_last_motor_log = 0.0
_last_motor_lr = (0, 0)


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


Bridge.provide("distance_cm", on_distance_cm)

print(f"[sancho_bridge] shim up | motors :{MOTOR_PORT} (Docker→MCU) | "
      f"sensors :{SENSOR_PORT} (MCU→Docker, dest auto-learned)")


def loop():
    global _ros_ip, _motor_count, _last_motor_log, _last_motor_lr
    try:
        data, addr = _recv_sock.recvfrom(64)
    except socket.timeout:
        return
    if len(data) < MOTOR_LEN:
        return

    _ros_ip = addr[0]
    left, right = struct.unpack(MOTOR_FMT, data[:MOTOR_LEN])
    Bridge.notify("set_motors", int(left), int(right))

    _motor_count += 1
    _last_motor_lr = (int(left), int(right))
    now = time.time()
    if now - _last_motor_log >= HEARTBEAT_S:
        _last_motor_log = now
        print(f"[sancho_bridge] motor packets fwd'd: {_motor_count} "
              f"(last L={_last_motor_lr[0]} R={_last_motor_lr[1]})")


App.run(user_loop=loop)
