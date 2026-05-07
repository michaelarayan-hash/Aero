import sys
import threading
import time
from pathlib import Path
import numpy as np
import cv2

sys.path.insert(0, "/usr/lib/python3/dist-packages")
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GzImage
import config

TOPIC_TEMPLATE = "/world/{world}/model/{vehicle}_0/link/camera_link/sensor/camera/image"

_FMT = {
    3: (np.uint8, 3),
    8: (np.uint8, 3),
    1: (np.uint8, 1),
}


class SimCamera:
    def __init__(self, world: str = config.WORLD, vehicle: str = config.VEHICLE):
        topic = TOPIC_TEMPLATE.format(world=world, vehicle=vehicle)
        self._frame = None
        self._lock = threading.Lock()
        self._node = Node()
        print(f"Subscribing to camera topic: {topic}")
        self._node.subscribe(GzImage, topic, self._on_image)

    def _on_image(self, msg: GzImage):
        '''Callback for incoming camera images
        Converts the raw image data to a BGR OpenCV format and stores it as the latest frame.
        '''
        fmt = _FMT.get(msg.pixel_format_type)
        if fmt is None:
            return
        dtype, channels = fmt
        raw = np.frombuffer(msg.data, dtype=dtype)
        img = raw.reshape((msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width))
        bgr = img[:, :, ::-1].copy() if channels > 1 else np.stack([img, img, img], axis=-1)
        with self._lock:
            self._frame = bgr

    def read(self) -> tuple[bool, np.ndarray | None]:
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    def wait_for_frame(self, timeout: float = 5) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.read()[0]:
                return True
            time.sleep(0.05)
        return False