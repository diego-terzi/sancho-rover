"""
SANCHO App Lab Python — UDP-to-Bridge shim, motors-only.

Stripped to the simplest possible flow: receive motor packets from ROS over
UDP, forward to MCU as Bridge.call("set_motors", L, R). Sensor flow removed
because outgoing Bridge.notify("distance_cm", ...) traffic from the MCU was
saturating the bridge and starving incoming set_motors calls. Once basic
4WD motion is verified end-to-end we'll re-introduce the sensor path with
rate limits.

Why call() not notify():
  notify() queues fire-and-forget on the bridge; if the bridge has *any*
  latency, the queue builds up and the MCU processes stale FIFO values
  while newer ones never arrive. We saw this on hardware.
  call() is synchronous: Python self-throttles to whatever rate the bridge
  can actually sustain, and the MCU always processes the latest value.
  Failures (e.g. >10 s round-trip) surface as visible exceptions instead
  of silently turning into stale-zero behaviour.

Design notes:
  - Drain the UDP socket each loop iteration: only the most recent packet
    is forwarded; older ones piled up in the kernel buffer are discarded.
  - With no other Bridge traffic competing, call should be fast (<100 ms).
"""

import socket
import struct
import time

from arduino.app_utils import App, Bridge


LISTEN_HOST    = "0.0.0.0"
MOTOR_PORT     = 9001

MOTOR_FMT      = "<hh"
MOTOR_LEN      = struct.calcsize(MOTOR_FMT)
RECV_TIMEOUT_S = 1.0
HEARTBEAT_S    = 5.0


_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_recv_sock.bind((LISTEN_HOST, MOTOR_PORT))
_recv_sock.settimeout(RECV_TIMEOUT_S)

_motor_count = 0
_last_motor_log = 0.0

print(f"[sancho_bridge] motors-only shim up | listening UDP {LISTEN_HOST}:{MOTOR_PORT}")


def loop():
    global _motor_count, _last_motor_log

    try:
        data, addr = _recv_sock.recvfrom(64)
    except socket.timeout:
        return
    if len(data) < MOTOR_LEN:
        return

    # Drain any additional packets that piled up while we were busy.
    _recv_sock.setblocking(False)
    try:
        while True:
            data2, _addr2 = _recv_sock.recvfrom(64)
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
        print(f"[sancho_bridge] motor calls completed: {_motor_count} "
              f"(last L={int(left)} R={int(right)})")


App.run(user_loop=loop)
