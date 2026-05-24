"""
Follow camera node: detects an ArUco marker and publishes lateral error and distance.

Publishes:
  /follow_error    (Float32): lateral error [-1,+1], NaN if marker absent.
                   Positive = marker right of frame centre.
  /follow_distance (Float32): distance in metres from camera to marker,
                   NaN if absent or camera_matrix is empty.

Subscribes:
  /active_mode (String): pauses capture when not 'FOLLOW'.
                In TRAIL mode the camera device is released so that
                camera_node (trail-follow) can open it exclusively.

Runtime notes:
  - cv2.aruco.estimatePoseSingleMarkers requires camera_matrix (9 floats,
    row-major) and dist_coeffs (5 floats). If camera_matrix is empty the
    node logs a one-time WARNING and always publishes NaN on /follow_distance.
  - gpiod is NOT used here; GPIO is handled by mode_manager_node only.
"""

import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class FollowCameraNode(Node):
    def __init__(self):
        super().__init__('follow_camera_node')

        self._cam_idx      = int(self.declare_parameter('camera_index',   0).value)
        self._frame_width  = int(self.declare_parameter('frame_width',    640).value)
        self._frame_height = int(self.declare_parameter('frame_height',   480).value)
        self._rate_hz      = float(self.declare_parameter('publish_rate_hz', 30.0).value)
        aruco_dict_str     = str(self.declare_parameter('aruco_dict',     'DICT_4X4_50').value)
        self._marker_id    = int(self.declare_parameter('marker_id',      0).value)
        self._marker_size  = float(self.declare_parameter('marker_size_m', 0.10).value)
        cam_mat_flat       = list(self.declare_parameter('camera_matrix', []).value or [])
        dist_flat          = list(self.declare_parameter('dist_coeffs',   []).value or [])
        self._patience     = int(self.declare_parameter('lost_marker_patience', 10).value)
        self._ema_alpha    = float(self.declare_parameter('ema_alpha',    0.3).value)
        show_debug         = bool(self.declare_parameter('show_debug',    True).value)

        if len(cam_mat_flat) == 9:
            self._camera_matrix = np.array(cam_mat_flat, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs = (
                np.array(dist_flat, dtype=np.float64) if dist_flat else np.zeros(5)
            )
        else:
            self._camera_matrix = None
            self._dist_coeffs = None
            self.get_logger().warn(
                'camera_matrix is empty — /follow_distance will always be NaN. '
                'Provide 9 floats (row-major) in sancho_params.yaml to enable pose estimation.'
            )

        self._show_debug = show_debug and bool(os.environ.get('DISPLAY'))

        # ArUco detector — defensive API: try new OpenCV 4.7+ API, fall back to 4.x
        aruco_dict_id = getattr(cv2.aruco, aruco_dict_str, cv2.aruco.DICT_4X4_50)
        try:
            dict_ = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        except AttributeError:
            dict_ = cv2.aruco.Dictionary_get(aruco_dict_id)
        try:
            params_ = cv2.aruco.DetectorParameters()
            _det = cv2.aruco.ArucoDetector(dict_, params_)
            self._detect = lambda gray: _det.detectMarkers(gray)[:2]
        except AttributeError:
            params_ = cv2.aruco.DetectorParameters_create()
            self._detect = (
                lambda gray: cv2.aruco.detectMarkers(gray, dict_, parameters=params_)[:2]
            )

        self._active_mode    = 'TRAIL'
        self._cap            = None
        self._smoothed_error = 0.0
        self._lost_counter   = 0
        self._last_error     = float('nan')
        self._last_distance  = float('nan')

        self._error_pub    = self.create_publisher(Float32, 'follow_error',    1)
        self._distance_pub = self.create_publisher(Float32, 'follow_distance', 1)

        self.create_subscription(String, 'active_mode', self._on_active_mode, 1)

        self.create_timer(1.0 / self._rate_hz, self._process_frame)
        self.get_logger().info('follow_camera_node started (waiting for FOLLOW mode)')

    def _on_active_mode(self, msg: String):
        new_mode = msg.data
        if new_mode == self._active_mode:
            return
        if new_mode == 'FOLLOW':
            self._cap = cv2.VideoCapture(self._cam_idx)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._frame_width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)
            self.get_logger().info('FOLLOW mode: camera opened')
        elif new_mode == 'TRAIL':
            if self._cap is not None and self._cap.isOpened():
                self._cap.release()
            self._cap = None
            self._last_error    = float('nan')
            self._last_distance = float('nan')
            self._publish(float('nan'), float('nan'))
            self.get_logger().info('TRAIL mode: camera released')
        self._active_mode = new_mode

    def _process_frame(self):
        if self._active_mode != 'FOLLOW':
            return
        if self._cap is None or not self._cap.isOpened():
            return

        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('follow_camera_node: frame read failed')
            return

        h, w  = frame.shape[:2]
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)

        found = False
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if int(mid) != self._marker_id:
                    continue
                found = True
                pts = corners[i][0]
                cx  = float(np.mean(pts[:, 0]))
                cy  = float(np.mean(pts[:, 1]))
                error = float(np.clip((cx - w / 2.0) / (w / 2.0), -1.0, 1.0))

                distance = float('nan')
                if self._camera_matrix is not None:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], self._marker_size,
                        self._camera_matrix, self._dist_coeffs
                    )
                    distance = float(np.linalg.norm(tvecs[0][0]))

                if self._lost_counter > self._patience:
                    self._smoothed_error = error
                else:
                    self._smoothed_error = (
                        self._ema_alpha * error
                        + (1.0 - self._ema_alpha) * self._smoothed_error
                    )
                self._lost_counter  = 0
                self._last_error    = self._smoothed_error
                self._last_distance = distance

                if self._show_debug:
                    cv2.polylines(frame, [pts.astype(int)], True, (0, 255, 0), 2)
                    cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    label = f'err={self._last_error:.2f}  d={distance:.2f}m'
                    cv2.putText(frame, label, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                break

        if not found:
            self._lost_counter += 1
            if self._lost_counter > self._patience:
                self._last_error    = float('nan')
                self._last_distance = float('nan')

        self._publish(self._last_error, self._last_distance)

        if self._show_debug:
            cv2.imshow('follow_camera', frame)
            cv2.waitKey(1)

    def _publish(self, error: float, distance: float):
        err_msg = Float32()
        err_msg.data = error
        dist_msg = Float32()
        dist_msg.data = distance
        self._error_pub.publish(err_msg)
        self._distance_pub.publish(dist_msg)

    def destroy_node(self):
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()
        if self._show_debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
