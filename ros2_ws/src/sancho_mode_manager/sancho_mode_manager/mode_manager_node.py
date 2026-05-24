"""
Mode manager node: reads a two-position GPIO switch and muxes /trail_cmd_vel
or /follow_cmd_vel onto /cmd_vel. Also publishes /active_mode so camera nodes
can arbitrate access to the shared camera device.

GPIO wiring convention: LOW = active (pin pulled to GND through switch,
internal pull-up keeps pin HIGH when switch is open).
  gpio_pin_trail  reads LOW → TRAIL mode
  gpio_pin_follow reads LOW → FOLLOW mode

If gpiod import or GPIO open fails, the node locks to default_mode and
continues running (safe fallback for dev machines without GPIO).

Runtime notes:
  - python3-gpiod must be installed; if absent, GPIO is silently disabled.
  - On some gpiod versions LINE_REQ_DIR_IN has a different name; the node
    catches any AttributeError and falls back to a plain integer request.
  - transition_stop_ms is implemented with time.sleep inside the 10 Hz timer
    (blocks ≤200 ms — acceptable per the design spec).
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager_node')

        self._gpio_chip    = str(self.declare_parameter('gpio_chip',           'gpiochip0').value)
        self._pin_trail    = int(self.declare_parameter('gpio_pin_trail',       0).value)
        self._pin_follow   = int(self.declare_parameter('gpio_pin_follow',      1).value)
        self._poll_hz      = float(self.declare_parameter('poll_rate_hz',       10.0).value)
        self._debounce_ms  = float(self.declare_parameter('switch_debounce_ms', 50.0).value)
        self._stop_ms      = float(self.declare_parameter('transition_stop_ms', 200.0).value)
        self._default_mode = str(self.declare_parameter('default_mode',         'TRAIL').value)

        self._active_mode  = self._default_mode
        self._trail_twist  = None
        self._follow_twist = None

        self._gpio_ok      = False
        self._trail_line   = None
        self._follow_line  = None
        self._setup_gpio()

        self.create_subscription(Twist, 'trail_cmd_vel',  self._on_trail,  1)
        self.create_subscription(Twist, 'follow_cmd_vel', self._on_follow, 1)

        self._cmd_vel_pub     = self.create_publisher(Twist,  'cmd_vel',     1)
        self._active_mode_pub = self.create_publisher(String, 'active_mode', 1)

        self._candidate_mode         = self._active_mode
        self._candidate_stable_since = None  # monotonic seconds

        self._publish_active_mode()

        self.create_timer(1.0 / self._poll_hz, self._poll_step)
        self.get_logger().info(
            f'mode_manager_node started, mode={self._active_mode}, gpio={self._gpio_ok}'
        )

    def _setup_gpio(self):
        try:
            import gpiod  # noqa: PLC0415 — import inside method for optional dep
            chip = gpiod.Chip(self._gpio_chip)
            self._trail_line  = chip.get_line(self._pin_trail)
            self._follow_line = chip.get_line(self._pin_follow)
            try:
                req_type = gpiod.LINE_REQ_DIR_IN
            except AttributeError:
                req_type = 1  # gpiod 1.x fallback constant
            self._trail_line.request(consumer='mode_manager', type=req_type)
            self._follow_line.request(consumer='mode_manager', type=req_type)
            self._gpio_ok = True
            self.get_logger().info('GPIO initialized successfully')
        except Exception as exc:
            self.get_logger().warn(
                f'GPIO init failed ({exc}) — mode locked to {self._default_mode}'
            )

    def _on_trail(self, msg: Twist):
        self._trail_twist = msg

    def _on_follow(self, msg: Twist):
        self._follow_twist = msg

    def _read_gpio_raw(self) -> str:
        if not self._gpio_ok:
            return self._active_mode
        try:
            trail_low  = (self._trail_line.get_value()  == 0)
            follow_low = (self._follow_line.get_value() == 0)
        except Exception:
            return self._active_mode
        if trail_low and follow_low:
            self.get_logger().warn(
                'Both GPIO pins LOW simultaneously — wiring error, keeping current mode'
            )
            return self._active_mode
        if trail_low:
            return 'TRAIL'
        if follow_low:
            return 'FOLLOW'
        return self._active_mode  # neither LOW: hold current mode

    def _poll_step(self):
        raw = self._read_gpio_raw()
        now = time.monotonic()

        if raw != self._candidate_mode:
            self._candidate_mode = raw
            self._candidate_stable_since = now
        else:
            if (self._candidate_stable_since is not None
                    and self._candidate_mode != self._active_mode):
                elapsed_ms = (now - self._candidate_stable_since) * 1000.0
                if elapsed_ms >= self._debounce_ms:
                    self._do_transition(self._candidate_mode)
                    self._candidate_stable_since = None

        twist = (self._trail_twist if self._active_mode == 'TRAIL' else self._follow_twist)
        self._cmd_vel_pub.publish(twist or Twist())

    def _do_transition(self, new_mode: str):
        self.get_logger().info(f'Mode: {self._active_mode} -> {new_mode}')
        self._cmd_vel_pub.publish(Twist())              # stop immediately
        time.sleep(self._stop_ms / 1000.0)             # hold stop for transition_stop_ms
        self._active_mode = new_mode
        self._publish_active_mode()

    def _publish_active_mode(self):
        msg = String()
        msg.data = self._active_mode
        self._active_mode_pub.publish(msg)

    def destroy_node(self):
        for line in (self._trail_line, self._follow_line):
            if line is not None:
                try:
                    line.release()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
