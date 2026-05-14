import math
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

_SIM_STATE = Path('/workspace/tmp/sim_state.json')


def _sim_defaults() -> tuple[str, str]:
    """Read world/vehicle written by sim.py, falling back to hardcoded defaults."""
    try:
        state = json.loads(_SIM_STATE.read_text())
        return state.get('world', 'aruco'), state.get('vehicle', 'x500_mono_cam_down')
    except (FileNotFoundError, json.JSONDecodeError):
        return 'aruco', 'x500_mono_cam_down'
    

def _obj_points(size_mm: float) -> np.ndarray:
    h = size_mm / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def _build_detector(dict_name: str):
    registry = {
        '4x4_50':    cv2.aruco.DICT_4X4_50,
        '4x4_1000':  cv2.aruco.DICT_4X4_1000,
        '5x5_1000':  cv2.aruco.DICT_5X5_1000,
        '6x6_1000':  cv2.aruco.DICT_6X6_1000,
    }
    aruco_dict = cv2.aruco.getPredefinedDictionary(registry[dict_name])
    try:
        params = cv2.aruco.DetectorParameters()
        return cv2.aruco.ArucoDetector(aruco_dict, params)
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()
        return (aruco_dict, params)


def _detect(gray: np.ndarray, detector) -> tuple:
    small = cv2.resize(gray, (gray.shape[1] // 2, gray.shape[0] // 2))
    if isinstance(detector, tuple):
        aruco_dict, params = detector
        corners, ids, _ = cv2.aruco.detectMarkers(small, aruco_dict, parameters=params)
    else:
        corners, ids, _ = detector.detectMarkers(small)
    if ids is None:
        return [], []
    corners = [c * 2.0 for c in corners]
    return corners, ids


def _estimate_pose(corners, obj_pts, mtx, dist):
    rvecs, tvecs = [], []
    for c in corners:
        _, rvec, tvec = cv2.solvePnP(obj_pts, c[0], mtx, dist,
                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(rvec)
        tvecs.append(tvec)
    return rvecs, tvecs


def _draw_brackets(frame, pts, colour, thickness=3):
    blen = max(12, int(np.linalg.norm(pts[1] - pts[0]) * 0.25))
    for j, pt in enumerate(pts):
        for nb in [pts[(j + 1) % 4], pts[(j - 1) % 4]]:
            d = (nb - pt).astype(float)
            n = d / (np.linalg.norm(d) + 1e-6)
            cv2.line(frame, tuple(pt), tuple((pt + n * blen).astype(int)), colour, thickness)


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        default_world, default_vehicle = _sim_defaults()
        self.declare_parameter('world', default_world)
        self.declare_parameter('vehicle', default_vehicle)
        self.declare_parameter('display', False)
        self.declare_parameter('marker_size_mm', 500.0)
        self.declare_parameter('dict_name', '4x4_50')

        world = self.get_parameter('world').get_parameter_value().string_value
        vehicle = self.get_parameter('vehicle').get_parameter_value().string_value
        self._dictionary = self.get_parameter('dict_name').get_parameter_value().string_value
        self._display = self.get_parameter('display').get_parameter_value().bool_value
        self._marker_size_mm = self.get_parameter('marker_size_mm').get_parameter_value().double_value


        # Camera intrinsics for x500_mono_cam_down (sim fallback)
        fx = (1280 / 2) / math.tan(1.74 / 2)
        self._mtx  = np.array([[fx, 0, 640.0],
                                [0, fx, 480.0],
                                [0,  0,   1.0]], dtype=np.float64)
        self._dist = np.zeros((5, 1), dtype=np.float64)

        self._obj_pts  = _obj_points(self._marker_size_mm)
        self._detector = _build_detector(self._dictionary)
        self._bridge   = CvBridge()

        topic = f'/world/{world}/model/{vehicle}_0/link/camera_link/sensor/camera/image'
        self.get_logger().info(f'Subscribing to: {topic}')

        self.create_subscription(Image, topic, self._on_image, 10)
        self._pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)

        if self._display:
            cv2.namedWindow('ArUco Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('ArUco Detection', 960, 720)

    def _on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids = _detect(gray, self._detector)
        if not corners:
            if self._display:
                cv2.putText(frame, 'SCANNING...', (12, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 100, 255), 2)
            self._show(frame)
            return

        rvecs, tvecs = _estimate_pose(corners, self._obj_pts, self._mtx, self._dist)

        # Pick the closest marker (smallest z) to publish
        best_i   = int(np.argmin([float(np.linalg.norm(t)) for t in tvecs]))
        best_tvec = tvecs[best_i].flatten()
        best_id   = int(ids[best_i][0])

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = str(best_id)
        pose_msg.pose.position.x = float(best_tvec[0])
        pose_msg.pose.position.y = float(best_tvec[1])
        pose_msg.pose.position.z = float(best_tvec[2])
        self._pub.publish(pose_msg)

        self.get_logger().info(
            f'ID{best_id}  x={best_tvec[0]:.1f}  y={best_tvec[1]:.1f}  z={best_tvec[2]:.1f} mm'
        )

        if self._display:
            colour = (0, 255, 0)
            b = 6
            cv2.rectangle(frame, (b, b), (frame.shape[1]-b, frame.shape[0]-b), colour, b)
            for i, mid in enumerate(ids):
                pts = corners[i][0].astype(int)
                _draw_brackets(frame, pts, colour)
                cv2.drawFrameAxes(frame, self._mtx, self._dist, rvecs[i], tvecs[i],
                                  self._obj_pts[1][0])
                dist_mm = float(np.linalg.norm(tvecs[i]))
                cv2.putText(frame, f'ID{mid[0]}  {dist_mm:.0f}mm',
                            (pts[0][0], pts[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
        self._show(frame)

    def _show(self, frame):
        if self._display:
            cv2.imshow('ArUco Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()