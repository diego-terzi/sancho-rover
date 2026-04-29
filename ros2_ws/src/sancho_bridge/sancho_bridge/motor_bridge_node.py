"""
motor_bridge_node — converts /cmd_vel into per-track PWM and forwards to the MCU.

Subscribes to /cmd_vel (Twist), applies differential-drive kinematics, maps the
result to signed PWM in [-255, +255], and emits the result two ways:

  1. /motor_pwm (Int16MultiArray)            — telemetry, consumed by sim_node
  2. UDP datagram to 127.0.0.1:9001          — consumed by an Arduino App Lab
                                               Python shim (arduino/sancho_bridge/python/main.py)
                                               that calls Bridge.notify("set_motors", L, R)

The two-step UDP→Bridge path exists because our ROS 2 stack runs in Docker,
while Arduino's `arduino.app_utils.Bridge` lives in App Lab's per-app venv on
the host. UDP is the simplest decoupling: Docker doesn't need to know anything
about App Lab; the App Lab shim doesn't need to know anything about ROS 2.

A 500 ms software watchdog forces a stop command when /cmd_vel stops arriving.
This is the first of three independent safety layers:
  1. this Python watchdog
  2. an MCU-side firmware watchdog (also 500 ms)
  3. PWM = 0 = BTS7960 half-bridges off → motors coast

`dry_run` mode skips both /motor_pwm publish (no, it still publishes — telemetry is
always on) and the UDP send. Use it on dev machines that don't have the App Lab
shim running, just to verify the kinematics math.
"""

import math
import socket
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray


# Wire format for the UDP packets to the App Lab shim:
#   2 little-endian signed shorts: [left_pwm, right_pwm]
_PWM_PACKET_FMT = "<hh"


class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        # --- parameters ---------------------------------------------------
        # Kinematics (from hardware measurement)
        self.declare_parameter('wheel_separation', 0.265)
        self.declare_parameter('wheel_diameter',   0.06)
        self.declare_parameter('motor_rpm',        333.0)

        # PWM limits
        self.declare_parameter('max_pwm',       255)
        self.declare_parameter('deadband_pwm',  0)

        # Per-motor calibration
        self.declare_parameter('invert_left',   False)
        self.declare_parameter('invert_right',  False)
        self.declare_parameter('left_scale',    1.0)
        self.declare_parameter('right_scale',   1.0)

        # Safety
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('watchdog_rate_hz', 20.0)

        # UDP forwarding to the App Lab Bridge shim
        self.declare_parameter('udp_target_host', '127.0.0.1')
        self.declare_parameter('udp_target_port', 9001)
        self.declare_parameter('dry_run', False)

        # --- read params --------------------------------------------------
        self.d         = float(self.get_parameter('wheel_separation').value)
        self.diameter  = float(self.get_parameter('wheel_diameter').value)
        self.rpm       = float(self.get_parameter('motor_rpm').value)
        self.max_pwm   = int(self.get_parameter('max_pwm').value)
        self.deadband  = int(self.get_parameter('deadband_pwm').value)
        self.inv_left  = bool(self.get_parameter('invert_left').value)
        self.inv_right = bool(self.get_parameter('invert_right').value)
        self.sc_left   = float(self.get_parameter('left_scale').value)
        self.sc_right  = float(self.get_parameter('right_scale').value)
        self.wd_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.wd_rate   = float(self.get_parameter('watchdog_rate_hz').value)
        self.udp_host  = str(self.get_parameter('udp_target_host').value)
        self.udp_port  = int(self.get_parameter('udp_target_port').value)
        self.dry_run   = bool(self.get_parameter('dry_run').value)

        # --- derived ------------------------------------------------------
        self.v_max = (self.rpm / 60.0) * math.pi * self.diameter
        if self.v_max <= 0.0:
            raise RuntimeError(
                f'v_max must be > 0 (got {self.v_max}). Check wheel_diameter and motor_rpm.'
            )

        # --- UDP socket ---------------------------------------------------
        self.udp_target = (self.udp_host, self.udp_port)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connect-style: the OS caches the route, slightly cheaper than per-call resolution
        # We don't actually connect (UDP is connectionless), just record the target.

        # --- state --------------------------------------------------------
        self.last_cmd_time = None
        self.last_left_pwm = 0
        self.last_right_pwm = 0
        self.stopped = True

        # --- I/O + timer --------------------------------------------------
        self.pwm_pub = self.create_publisher(Int16MultiArray, 'motor_pwm', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(1.0 / self.wd_rate, self._watchdog)

        # Send an explicit stop at startup so the MCU sees a known 0 ASAP.
        self._send(0, 0)

        self.get_logger().info(
            f'motor_bridge_node ready | d={self.d:.3f} m, '
            f'diam={self.diameter:.3f} m, rpm={self.rpm:.1f} -> v_max={self.v_max:.3f} m/s | '
            f'watchdog={self.wd_timeout:.2f} s | udp -> {self.udp_host}:{self.udp_port} | '
            f'dry_run={self.dry_run}'
        )

    # ------------------------------------------------------------------ cmd_vel

    def _on_cmd_vel(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        # Differential-drive kinematics
        # ω>0 = turn left (REP-103) → right track faster, left track slower
        v_left  = vx - wz * self.d / 2.0
        v_right = vx + wz * self.d / 2.0

        pwm_left  = self._vel_to_pwm(v_left,  self.sc_left,  self.inv_left)
        pwm_right = self._vel_to_pwm(v_right, self.sc_right, self.inv_right)

        self._send(pwm_left, pwm_right)
        self.last_cmd_time = self.get_clock().now()
        self.stopped = (pwm_left == 0 and pwm_right == 0)

    # ------------------------------------------------------------------ mapping

    def _vel_to_pwm(self, v: float, scale: float, invert: bool) -> int:
        """Map a track velocity (m/s) to signed PWM in [-max_pwm, +max_pwm]."""
        pwm = (v / self.v_max) * self.max_pwm * scale
        pwm = int(round(pwm))
        pwm = max(-self.max_pwm, min(self.max_pwm, pwm))

        if self.deadband > 0 and 0 < abs(pwm) < self.deadband:
            pwm = self.deadband if pwm > 0 else -self.deadband

        if invert:
            pwm = -pwm
        return pwm

    # ------------------------------------------------------------------ output

    def _send(self, left: int, right: int):
        self.last_left_pwm = left
        self.last_right_pwm = right

        # Telemetry — always publish (consumed by sim_node).
        tel = Int16MultiArray()
        tel.data = [int(left), int(right)]
        try:
            self.pwm_pub.publish(tel)
        except Exception:
            pass

        if self.dry_run:
            self.get_logger().info(f'[dry_run] set_motors(L={left:+4d}, R={right:+4d})')
            return

        # Forward to the App Lab Python shim via UDP.
        # Clamp to int16 range just in case (max_pwm is 255 by default).
        l16 = max(-32768, min(32767, int(left)))
        r16 = max(-32768, min(32767, int(right)))
        try:
            self.udp_sock.sendto(struct.pack(_PWM_PACKET_FMT, l16, r16), self.udp_target)
        except Exception as e:
            self.get_logger().error(f'UDP send to {self.udp_target} failed: {e}')

    # ------------------------------------------------------------------ watchdog

    def _watchdog(self):
        if self.last_cmd_time is None:
            return
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.wd_timeout and not self.stopped:
            self.get_logger().warn(
                f'watchdog: no /cmd_vel for {age:.2f}s (>{self.wd_timeout:.2f}s) — stopping motors'
            )
            self._send(0, 0)
            self.stopped = True

    # ------------------------------------------------------------------ shutdown

    def destroy_node(self):
        # Best-effort stop on exit.
        try:
            self._send(0, 0)
        except Exception:
            pass
        try:
            self.udp_sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
