"""
SANCHO App Lab Python — UDP-to-Bridge shim, motors only.

Motor path:
    ROS 2 motor_bridge_node → UDP :9001 → here → Bridge.call("set_motors", L, R) → MCU

Why call() and not notify():
  notify() queues fire-and-forget on the bridge; if the bridge has any latency,
  the queue builds up and the MCU processes stale FIFO values. call() is
  synchronous: Python self-throttles to whatever rate the bridge can sustain.

Design notes:
  - Drain the UDP socket each loop iteration: only the most recent motor packet
    is forwarded; older ones piled up in the kernel buffer are discarded.
  - RECV_TIMEOUT_S is short (50 ms) so the loop stays responsive even when
    no motor packets are arriving (rover stopped).
"""

import socket
import struct
import time

from arduino.app_utils import App, Bridge


LISTEN_HOST    = "0.0.0.0"
MOTOR_PORT     = 9001

MOTOR_FMT      = "<hh"
MOTOR_LEN      = struct.calcsize(MOTOR_FMT)
RECV_TIMEOUT_S = 0.05   # 50 ms
HEARTBEAT_S    = 5.0


_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_recv_sock.bind((LISTEN_HOST, MOTOR_PORT))
_recv_sock.settimeout(RECV_TIMEOUT_S)

_motor_count    = 0
_last_motor_log = 0.0

print(f"[sancho_bridge] shim up | motor UDP {LISTEN_HOST}:{MOTOR_PORT}")


def loop():
    global _motor_count, _last_motor_log

    # Receive one packet (blocks up to RECV_TIMEOUT_S)
    try:
        data, _addr = _recv_sock.recvfrom(64)
    except socket.timeout:
        return

    if data is None or len(data) < MOTOR_LEN:
        return

    # Drain any packets that piled up — forward only the most recent one
    _recv_sock.setblocking(False)
    try:
        while True:
            data2, _ = _recv_sock.recvfrom(64)
            if len(data2) >= MOTOR_LEN:
                data = data2
    except (BlockingIOError, socket.timeout):
        pass
    finally:
        _recv_sock.settimeout(RECV_TIMEOUT_S)

    left, right = struct.unpack(MOTOR_FMT, data[:MOTOR_LEN])
    try:
        Bridge.call("set_motors", int(left), int(right))
    except Exception as e:
        print(f"[sancho_bridge] Bridge.call(set_motors) failed: {e}")
        return

    _motor_count += 1
    now = time.time()
    if now - _last_motor_log >= HEARTBEAT_S:
        _last_motor_log = now
        print(
            f"[sancho_bridge] motor calls: {_motor_count} "
            f"(last L={int(left)} R={int(right)})"
        )


App.run(user_loop=loop)
