import json
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
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


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_feed')

        default_world, default_vehicle = _sim_defaults()

        self.declare_parameter('world', default_world)
        self.declare_parameter('vehicle', default_vehicle)
        self.declare_parameter('display', True)

        world = self.get_parameter('world').get_parameter_value().string_value
        vehicle = self.get_parameter('vehicle').get_parameter_value().string_value
        self._display = self.get_parameter('display').get_parameter_value().bool_value

        topic = f'/world/{world}/model/{vehicle}_0/link/camera_link/sensor/camera/image'
        self.get_logger().info(f'Subscribing to: {topic}')
        self._bridge = CvBridge()
        self._frame_count = 0
        self.create_subscription(Image, topic, self._on_image, 10)

        if self._display:
            cv2.namedWindow('Aero Camera', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Aero Camera', 960, 720)

    def _on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._frame_count += 1

        if self._frame_count % 30 == 1:
            self.get_logger().info(f'Frame {self._frame_count}: {msg.width}x{msg.height}')

        if self._display:
            cv2.imshow('Aero Camera', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
