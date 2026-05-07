"""
Minimal Gazebo camera subscriber for the Tutorials_michy folder.

Usage:
    from gz_cam import SimCamera
    cam = SimCamera()
    cam.wait_for_frame(timeout=10)
    ok, frame = cam.read()   # frame is a BGR numpy array
"""

import json
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GzImage
import config

TOPIC_TEMPLATE = "/world/{world}/model/{vehicle}_0/link/camera_link/sensor/camera/image"
_STATE_FILE = Path("/tmp/sim_state.json")


def _read_sim_state() -> dict:
    try:
        return json.loads(_STATE_FILE.read_text())
    except Exception:
        return {}


_FMT = {
    3: (np.uint8, 3),   # RGB_INT8
    8: (np.uint8, 3),   # BGR_INT8
    1: (np.uint8, 1),   # L_INT8 (grayscale)
}


class SimCamera:
    def __init__(self, world: str | None = None, vehicle: str | None = None):
        state = _read_sim_state()
        world = world or state.get("world", config.WORLD)
        vehicle = vehicle or state.get("vehicle", config.VEHICLE)
        self.topic = TOPIC_TEMPLATE.format(world=world, vehicle=vehicle)
        self._frame = None
        self._lock = threading.Lock()
        self._node = Node()
        self._node.subscribe(GzImage, self.topic, self._on_image)

    def _on_image(self, msg: GzImage):
        fmt = _FMT.get(msg.pixel_format_type)
        if fmt is None:
            return
        dtype, channels = fmt
        raw = np.frombuffer(msg.data, dtype=dtype)
        img = raw.reshape(
            (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
        )
        bgr = img[:, :, ::-1].copy() if channels > 1 else np.stack([img, img, img], axis=-1)
        with self._lock:
            self._frame = bgr

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Return (True, BGR frame) if a frame is available, else (False, None)."""
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    def wait_for_frame(self, timeout: float = 10.0) -> bool:
        """Block until the first frame arrives or timeout (seconds) expires."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.read()[0]:
                return True
            time.sleep(0.05)
        return False
