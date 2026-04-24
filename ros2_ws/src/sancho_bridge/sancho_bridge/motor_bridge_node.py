"""
motor_bridge_node — the only node that talks to the MCU.

Subscribes to /cmd_vel (Twist), converts the velocity command into per-track PWM
using differential-drive kinematics, and sends `set_motors(left, right)` over the
Arduino Bridge RPC to the MCU. A watchdog stops the motors if /cmd_vel stops
arriving — the rover must not keep rolling when the controller dies.

On dev machines (no Bridge library, no MCU), the node runs in `dry_run` mode:
it logs the PWMs it *would* send but does not call the Bridge. This lets you
validate the kinematics and watchdog end-to-end without hardware.

Pipeline:
    /cmd_vel ──► _on_cmd_vel ──► diff-drive kinematics ──► velocity → PWM ──►
      clamp/scale/invert/deadband ──► bridge.call("set_motors", L, R)

    timer @ watchdog_rate_hz ──► if no cmd recently, send set_motors(0, 0)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

# The Arduino Bridge library is only installed on the MPU (Arduino UNO Q).
# On a dev machine the import will fail — the node then runs in dry_run mode.
try:
    # TODO: confirm the actual Python import path on the Arduino UNO Q.
    # The Arduino Bridge project exposes a Python client alongside the C++
    # Arduino-side library; adjust this import once the deployment image is built.
    from arduino_bridge import Bridge  # type: ignore
    _BRIDGE_IMPORT_OK = True
    _BRIDGE_IMPORT_ERR = ''
except Exception as _e:  # ImportError, or any runtime failure at import time
    Bridge = None  # type: ignore[assignment]
    _BRIDGE_IMPORT_OK = False
    _BRIDGE_IMPORT_ERR = str(_e)


class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        # --- parameters ---------------------------------------------------
        # Kinematics (from hardware measurement)
        self.declare_parameter('wheel_separation', 0.30)   # m, track center-to-center
        self.declare_parameter('wheel_diameter',   0.06)   # m, driven sprocket
        self.declare_parameter('motor_rpm',        333.0)  # no-load RPM at the output shaft @ 12 V

        # PWM limits
        self.declare_parameter('max_pwm',       255)       # IBT-2 / BTS7960 is 8-bit PWM
        self.declare_parameter('deadband_pwm',  0)         # min PWM below which motors don't spin

        # Per-motor calibration
        self.declare_parameter('invert_left',   False)
        self.declare_parameter('invert_right',  False)
        self.declare_parameter('left_scale',    1.0)
        self.declare_parameter('right_scale',   1.0)

        # Safety
        self.declare_parameter('watchdog_timeout', 0.5)    # s; stop if no cmd_vel for this long
        self.declare_parameter('watchdog_rate_hz', 20.0)   # watchdog check frequency

        # Dev/test — defaults to auto: True when Bridge not importable, else False
        self.declare_parameter('dry_run', not _BRIDGE_IMPORT_OK)

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
        self.dry_run   = bool(self.get_parameter('dry_run').value)

        # --- derived ------------------------------------------------------
        # v_max = (rpm/60) * pi * diameter   [m/s at full PWM, no-load]
        self.v_max = (self.rpm / 60.0) * math.pi * self.diameter
        if self.v_max <= 0.0:
            raise RuntimeError(
                f'v_max must be > 0 (got {self.v_max}). Check wheel_diameter and motor_rpm.'
            )

        # --- bridge init --------------------------------------------------
        self.bridge = None
        if not self.dry_run:
            if not _BRIDGE_IMPORT_OK:
                self.get_logger().warn(
                    f'Arduino Bridge import failed ({_BRIDGE_IMPORT_ERR}); forcing dry_run=True'
                )
                self.dry_run = True
            else:
                try:
                    self.bridge = Bridge()           # TODO: confirm constructor signature
                    self.bridge.begin()              # TODO: confirm init method on hardware
                except Exception as e:
                    self.get_logger().error(
                        f'Failed to initialize Bridge: {e}. Forcing dry_run=True.'
                    )
                    self.bridge = None
                    self.dry_run = True

        # --- state --------------------------------------------------------
        self.last_cmd_time = None
        self.last_left_pwm = 0
        self.last_right_pwm = 0
        self.stopped = True  # initial state: motors not running

        # --- I/O + timer --------------------------------------------------
        # Create the publisher before the first _send() so the startup stop
        # can publish its telemetry.
        self.pwm_pub = self.create_publisher(Int16MultiArray, 'motor_pwm', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(1.0 / self.wd_rate, self._watchdog)

        # Send an explicit stop at startup so the MCU's PWM is at known 0
        self._send(0, 0)

        self.get_logger().info(
            f'motor_bridge_node ready | d={self.d:.3f} m, '
            f'diam={self.diameter:.3f} m, rpm={self.rpm:.1f} -> v_max={self.v_max:.3f} m/s | '
            f'watchdog={self.wd_timeout:.2f} s | dry_run={self.dry_run}'
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

        # Deadband: if we're commanding non-zero motion but below the stiction
        # threshold, bump up to the deadband so the motor actually moves.
        # A true zero command still produces PWM 0 (not deadband).
        if self.deadband > 0 and 0 < abs(pwm) < self.deadband:
            pwm = self.deadband if pwm > 0 else -self.deadband

        if invert:
            pwm = -pwm
        return pwm

    # ------------------------------------------------------------------ output

    def _send(self, left: int, right: int):
        self.last_left_pwm = left
        self.last_right_pwm = right

        # Telemetry — always publish, even in dry_run
        tel = Int16MultiArray()
        tel.data = [int(left), int(right)]
        self.pwm_pub.publish(tel)

        if self.dry_run:
            self.get_logger().info(f'[dry_run] set_motors(L={left:+4d}, R={right:+4d})')
            return
        try:
            self.bridge.call('set_motors', left, right)
        except Exception as e:
            self.get_logger().error(f'bridge.call(set_motors) failed: {e}')

    # ------------------------------------------------------------------ watchdog

    def _watchdog(self):
        if self.last_cmd_time is None:
            return  # never received a command — leave motors at 0, nothing to time out
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.wd_timeout and not self.stopped:
            self.get_logger().warn(
                f'watchdog: no /cmd_vel for {age:.2f}s (>{self.wd_timeout:.2f}s) — stopping motors'
            )
            self._send(0, 0)
            self.stopped = True

    # ------------------------------------------------------------------ shutdown

    def destroy_node(self):
        # Always leave the rover stopped on exit
        self._send(0, 0)
        if self.bridge is not None:
            try:
                self.bridge.call('emergency_stop')
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
