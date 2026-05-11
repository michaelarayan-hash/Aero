import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_feed')

        self.declare_parameter('world', 'default')
        self.declare_parameter('vehicle', 'x500_mono_cam_down')
        self.declare_parameter('display', False)

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
