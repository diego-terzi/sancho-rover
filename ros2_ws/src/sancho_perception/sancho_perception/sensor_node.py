"""
sensor_node — UDP receiver for the front HC-SR04 ultrasonic, publishes /scan.

Data path:
    HC-SR04 → STM32U585 → Bridge.notify("distance_cm") → App Lab Python shim
            → UDP datagram → this node → /scan (sensor_msgs/Range)

The MCU samples the ultrasonic at ~20 Hz; each sample is a UDP packet of
2 bytes: uint16 little-endian distance in centimetres. A reading of 0 means
"no echo within timeout" (HC-SR04 timed out) and is mapped here to max_range
so consumers see "free space ahead" rather than "obstacle at 0 m".
"""

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


PACKET_FMT = "<H"
PACKET_LEN = struct.calcsize(PACKET_FMT)


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('udp_listen_host', '0.0.0.0')
        self.declare_parameter('udp_listen_port', 9002)
        self.declare_parameter('frame_id', 'ultrasonic_front')
        self.declare_parameter('field_of_view_rad', 0.26)
        self.declare_parameter('min_range_m', 0.02)
        self.declare_parameter('max_range_m', 4.0)

        self.host      = self.get_parameter('udp_listen_host').value
        self.port      = int(self.get_parameter('udp_listen_port').value)
        self.frame_id  = self.get_parameter('frame_id').value
        self.fov       = float(self.get_parameter('field_of_view_rad').value)
        self.min_range = float(self.get_parameter('min_range_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)

        self.scan_pub = self.create_publisher(Range, 'scan', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(0.5)

        self._stop = False
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'sensor_node ready | UDP {self.host}:{self.port} -> /scan | '
            f'range {self.min_range}..{self.max_range} m | fov={self.fov:.2f} rad'
        )

    def _recv_loop(self):
        while not self._stop and rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                return  # socket closed during shutdown
            if len(data) < PACKET_LEN:
                continue
            (cm,) = struct.unpack(PACKET_FMT, data[:PACKET_LEN])
            self._publish(cm)

    def _publish(self, cm: int):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = self.max_range if cm == 0 else cm / 100.0
        self.scan_pub.publish(msg)

    def destroy_node(self):
        self._stop = True
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
