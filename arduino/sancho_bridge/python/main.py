"""
SANCHO App Lab Python — bidirectional UDP-Bridge shim.

Two flows:
  1. Motor commands     (Docker → MCU)        UDP :9001 → Bridge.call(...)
  2. Ultrasonic readings (MCU → Docker)       Bridge.provide handler → UDP :9002

Bridge API (confirmed on hardware May 2026, dir(Bridge)):
    ['call', 'notify', 'provide', 'unprovide']
    notify  — async fire-and-forget. ⚠ Don't use for high-rate motor cmds:
              messages queue up and the MCU sees them at the bridge's
              actual throughput (≈ 0.2 Hz on this hardware), processing
              the oldest first. We saw the MCU stuck on stale L=0 R=0
              while Python was already sending non-zero values.
    call    — synchronous RPC. Python loop self-throttles to the bridge's
              real rate, so what reaches the MCU is *always* the latest
              command, not a stale one from minutes ago.
    provide — register Python handler for an MCU.notify event.

To keep things responsive on the UDP side we also drain the socket each
loop iteration: only the most recent motor packet is forwarded; older
ones piled up in the kernel buffer are discarded.
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

print(f"[sancho_bridge] shim up | motors :{MOTOR_PORT} (Docker→MCU via Bridge.call) | "
      f"sensors :{SENSOR_PORT} (MCU→Docker via Bridge.provide)")


def loop():
    global _ros_ip, _motor_count, _last_motor_log

    # Block up to RECV_TIMEOUT_S waiting for the first packet
    try:
        data, addr = _recv_sock.recvfrom(64)
    except socket.timeout:
        return
    if len(data) < MOTOR_LEN:
        return

    # Drain the socket: keep only the most recent packet, discard older ones
    # that piled up while we were busy with the previous Bridge.call.
    _recv_sock.setblocking(False)
    try:
        while True:
            data2, addr2 = _recv_sock.recvfrom(64)
            if len(data2) >= MOTOR_LEN:
                data, addr = data2, addr2
    except (BlockingIOError, socket.timeout):
        pass
    finally:
        _recv_sock.settimeout(RECV_TIMEOUT_S)

    _ros_ip = addr[0]
    left, right = struct.unpack(MOTOR_FMT, data[:MOTOR_LEN])

    # Synchronous call — blocks until the MCU acknowledges. This is what
    # keeps the Python side throttled to the real Bridge rate and prevents
    # the queue-of-stale-zeros scenario we saw with notify().
    try:
        Bridge.call("set_motors", int(left), int(right))
    except Exception as e:
        # Don't let a transient Bridge error kill the loop; the MCU has its
        # own 500 ms watchdog so motors will stop if calls keep failing.
        print(f"[sancho_bridge] Bridge.call(set_motors) failed: {e}")
        return

    _motor_count += 1
    now = time.time()
    if now - _last_motor_log >= HEARTBEAT_S:
        _last_motor_log = now
        print(f"[sancho_bridge] motor calls completed: {_motor_count} "
              f"(last L={int(left)} R={int(right)})")


App.run(user_loop=loop)
