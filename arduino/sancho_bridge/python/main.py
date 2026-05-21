"""
SANCHO App Lab Python — UDP-to-Bridge shim, motors + ultrasonic sensor.

Motor path (unchanged):
    ROS 2 motor_bridge_node → UDP :9001 → here → Bridge.call("set_motors", L, R) → MCU

Sensor path (new):
    here → Bridge.call("get_distance") → MCU HC-SR04
         → UDP :9002 → sensor_node (ROS 2) → /scan

Why call() and not notify():
  notify() queues fire-and-forget on the bridge; if the bridge has any latency,
  the queue builds up and the MCU processes stale FIFO values. call() is
  synchronous: Python self-throttles to whatever rate the bridge can sustain.
  Sensor reads are rate-limited to SENSOR_HZ to avoid starving motor calls.

Design notes:
  - Drain the UDP socket each loop iteration: only the most recent motor packet
    is forwarded; older ones piled up in the kernel buffer are discarded.
  - RECV_TIMEOUT_S is short (50 ms) so the sensor timer fires on time even
    when no motor packets are arriving (rover stopped).
  - Bridge.call("get_distance") blocks up to ~30 ms (pulseIn timeout on MCU).
    At 10 Hz sensor reads this adds at most one missed motor packet per cycle,
    well within the 500 ms MCU watchdog.
"""

import socket
import struct
import time

from arduino.app_utils import App, Bridge


LISTEN_HOST       = "0.0.0.0"
MOTOR_PORT        = 9001

SENSOR_HOST       = "172.20.10.1"
SENSOR_PORT       = 9002
SENSOR_HZ         = 15
SENSOR_INTERVAL_S = 1.0 / SENSOR_HZ

MOTOR_FMT      = "<hh"
MOTOR_LEN      = struct.calcsize(MOTOR_FMT)
SENSOR_FMT     = "<H"
RECV_TIMEOUT_S = 0.05   # 50 ms — short enough to hit the 10 Hz sensor deadline
HEARTBEAT_S    = 5.0


_recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_recv_sock.bind((LISTEN_HOST, MOTOR_PORT))
_recv_sock.settimeout(RECV_TIMEOUT_S)

_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

_motor_count     = 0
_sensor_count    = 0
_last_motor_log  = 0.0
_last_sensor_log = 0.0
_last_sensor_t   = 0.0

print(
    f"[sancho_bridge] shim up | "
    f"motor UDP {LISTEN_HOST}:{MOTOR_PORT} | "
    f"sensor UDP -> {SENSOR_HOST}:{SENSOR_PORT} @ {SENSOR_HZ} Hz"
)


def loop():
    global _motor_count, _sensor_count
    global _last_motor_log, _last_sensor_log, _last_sensor_t

    # ── Motor path ───────────────────────────────────────────────────────────
    try:
        data, _addr = _recv_sock.recvfrom(64)
    except socket.timeout:
        data = None

    if data is not None and len(data) >= MOTOR_LEN:
        # Drain any packets that piled up while we were handling the sensor.
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

        _motor_count += 1
        now = time.time()
        if now - _last_motor_log >= HEARTBEAT_S:
            _last_motor_log = now
            print(
                f"[sancho_bridge] motor calls: {_motor_count} "
                f"(last L={int(left)} R={int(right)})"
            )

    # ── Sensor path ──────────────────────────────────────────────────────────
    now = time.time()
    if now - _last_sensor_t >= SENSOR_INTERVAL_S:
        _last_sensor_t = now
        try:
            cm = Bridge.call("get_distance")
            cm_u16 = max(0, min(65535, int(cm)))
            _send_sock.sendto(
                struct.pack(SENSOR_FMT, cm_u16),
                (SENSOR_HOST, SENSOR_PORT),
            )
            _sensor_count += 1
            if now - _last_sensor_log >= HEARTBEAT_S:
                _last_sensor_log = now
                print(
                    f"[sancho_bridge] sensor reads: {_sensor_count} "
                    f"(last {cm_u16} cm)"
                )
        except Exception as e:
            print(f"[sancho_bridge] Bridge.call(get_distance) failed: {e}")


App.run(user_loop=loop)
