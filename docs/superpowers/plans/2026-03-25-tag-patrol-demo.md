# Tag Patrol Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A single-command demo where a drone autonomously patrols a waypoint circuit over a Gazebo world containing ArUco and AprilTag markers, displaying real-time distance annotations from the downward camera.

**Architecture:** `tag_detector.py` wraps both `cv2.aruco` and `pupil_apriltags` behind a unified `detect(frame)` interface. `tag_patrol.py` runs MAVSDK offboard flight in a background `asyncio` thread and OpenCV display on the main thread, connected via `threading.Event`. Gazebo marker models (6 total) live in `Simulation/models/` and the world SDF in `Simulation/worlds/`.

**Tech Stack:** Python 3.12, MAVSDK (offboard NED), gz-transport13, OpenCV 4.x (cv2.aruco), pupil-apriltags, pytest

---

## File Map

| Action | Path | Responsibility |
|--------|------|----------------|
| Create | `Simulation/tools/generate_markers.py` | Generates 6 PNG images for Gazebo textures |
| Create | `Simulation/models/aruco_4x4_id{0,1,2}/` | Gazebo model dirs (model.config + model.sdf + marker.png) |
| Create | `Simulation/models/apriltag_36h11_id{0,1,2}/` | Same for AprilTag models |
| Create | `Simulation/worlds/tag_demo.sdf` | Gazebo world with ground + sun + 6 marker includes |
| Create | `Simulation/algorithms/tag_detector.py` | `Detection` dataclass + `TagDetector` class |
| Create | `Simulation/tests/test_tag_detector.py` | Unit tests for `TagDetector` |
| Create | `Simulation/algorithms/tag_patrol.py` | Flight + display main script |
| Modify | `Simulation/requirements.txt` | Add `pupil-apriltags` |
| Modify | `Simulation/pyproject.toml` | Add `pupil-apriltags` dep + pytest dev dep |

---

## Task 1: Add `pupil-apriltags` dependency

**Files:**
- Modify: `Simulation/requirements.txt`
- Modify: `Simulation/pyproject.toml`

- [ ] **Step 1: Check whether `cv2.aruco` is already available**

`cv2.aruco` requires `opencv-contrib-python`, not `opencv-python`. Verify:

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
python3 -c "import cv2.aruco; print('aruco ok')"
```

- If it prints `aruco ok`: the correct package is installed, no change needed.
- If it raises `AttributeError: module 'cv2' has no attribute 'aruco'`: run:

```bash
pip uninstall opencv-python -y
pip install "opencv-contrib-python>=4.8,<4.10"
```

Then update `requirements.txt` and `pyproject.toml` to use `opencv-contrib-python` instead of `opencv-python`.

- [ ] **Step 2: Update requirements.txt**

```
mavsdk>=2.0.0
numpy>=1.24,<2
opencv-contrib-python>=4.8,<4.10
pupil-apriltags>=1.0.1
```

(Replace `opencv-python` with `opencv-contrib-python` if Step 1 required a swap; otherwise just add `pupil-apriltags`.)

- [ ] **Step 3: Update pyproject.toml**

```toml
[project]
name = "aero-simulation"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = [
    "mavsdk>=2.0.0",
    "numpy<2",
    "opencv-contrib-python>=4.8,<4.10",
    "pupil-apriltags>=1.0.1",
]

[dependency-groups]
dev = ["pytest>=8.0"]
```

- [ ] **Step 4: Install**

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
uv sync
```

Expected: installs `pupil_apriltags` and `pytest` without errors.

- [ ] **Step 5: Verify both packages**

```bash
python3 -c "import cv2.aruco; from pupil_apriltags import Detector; print('all ok')"
```

Expected: `all ok`

- [ ] **Step 6: Commit**

```bash
git add Simulation/requirements.txt Simulation/pyproject.toml Simulation/uv.lock
git commit -m "feat: add pupil-apriltags and pytest deps, ensure opencv-contrib"
```

---

## Task 2: Generate marker PNG images

**Files:**
- Create: `Simulation/tools/generate_markers.py`

Uses `cv2.aruco` to generate both ArUco and AprilTag images (OpenCV's `DICT_APRILTAG_36h11` produces bit-compatible images with `pupil_apriltags`).

- [ ] **Step 1: Create tools directory and script**

```bash
mkdir -p ~/Aero/Simulation/tools
```

Create `Simulation/tools/generate_markers.py`:

```python
#!/usr/bin/env python3
"""
Generate PNG marker images for the tag_demo Gazebo world.

Outputs:
  Simulation/models/aruco_4x4_id{0,1,2}/marker.png
  Simulation/models/apriltag_36h11_id{0,1,2}/marker.png

Run from Simulation/:
    python3 tools/generate_markers.py
"""
from pathlib import Path
import cv2
import numpy as np

MODELS_DIR = Path(__file__).parent.parent / "models"
SIZE_PX = 500  # fill the full PNG; no extra border needed


def write_marker(img_gray: np.ndarray, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(str(out_path), bgr)
    print(f"  wrote {out_path}")


def main() -> None:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    april_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    print("ArUco 4x4_50:")
    for mid in range(3):
        img = cv2.aruco.generateImageMarker(aruco_dict, mid, SIZE_PX)
        write_marker(img, MODELS_DIR / f"aruco_4x4_id{mid}" / "marker.png")

    print("AprilTag 36h11:")
    for mid in range(3):
        img = cv2.aruco.generateImageMarker(april_dict, mid, SIZE_PX)
        write_marker(img, MODELS_DIR / f"apriltag_36h11_id{mid}" / "marker.png")

    print("Done.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the script**

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
python3 tools/generate_markers.py
```

Expected output:
```
ArUco 4x4_50:
  wrote .../models/aruco_4x4_id0/marker.png
  wrote .../models/aruco_4x4_id1/marker.png
  wrote .../models/aruco_4x4_id2/marker.png
AprilTag 36h11:
  wrote .../models/apriltag_36h11_id0/marker.png
  wrote .../models/apriltag_36h11_id1/marker.png
  wrote .../models/apriltag_36h11_id2/marker.png
Done.
```

- [ ] **Step 3: Verify images look correct**

```bash
python3 -c "
import cv2, glob
for p in sorted(glob.glob('models/*/marker.png')):
    img = cv2.imread(p)
    print(f'{p}: {img.shape}')
"
```

Expected: 6 lines, each `(500, 500, 3)`

- [ ] **Step 4: Commit**

```bash
git add Simulation/tools/generate_markers.py Simulation/models/
git commit -m "feat: add marker PNG generator and generated images"
```

---

## Task 3: Create Gazebo models and world SDF

**Files:**
- Create: `Simulation/models/aruco_4x4_id{0,1,2}/model.config` (×3)
- Create: `Simulation/models/aruco_4x4_id{0,1,2}/model.sdf` (×3)
- Create: `Simulation/models/apriltag_36h11_id{0,1,2}/model.config` (×3)
- Create: `Simulation/models/apriltag_36h11_id{0,1,2}/model.sdf` (×3)
- Create: `Simulation/worlds/tag_demo.sdf`

The model pattern is identical to PX4's `arucotag` model: a flat `<plane>` geometry with a PBR albedo texture.

- [ ] **Step 1: Create model files using the generator script**

Create `Simulation/tools/create_models.py`:

```python
#!/usr/bin/env python3
"""
Write model.config and model.sdf for each marker model directory.
Run AFTER generate_markers.py (PNG files must already exist).

Run from Simulation/:
    python3 tools/create_models.py
"""
from pathlib import Path

MODELS_DIR = Path(__file__).parent.parent / "models"

CONFIG_TEMPLATE = """\
<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
"""

SDF_TEMPLATE = """\
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>0 0 0.001 0 0 0</pose>
    <link name="base">
      <visual name="base_visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>0.5 0.5</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://{name}/marker.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MARKERS = [
    ("aruco_4x4_id0",        "ArUco 4x4_50 marker ID 0"),
    ("aruco_4x4_id1",        "ArUco 4x4_50 marker ID 1"),
    ("aruco_4x4_id2",        "ArUco 4x4_50 marker ID 2"),
    ("apriltag_36h11_id0",   "AprilTag 36h11 marker ID 0"),
    ("apriltag_36h11_id1",   "AprilTag 36h11 marker ID 1"),
    ("apriltag_36h11_id2",   "AprilTag 36h11 marker ID 2"),
]


def main() -> None:
    for name, desc in MARKERS:
        d = MODELS_DIR / name
        d.mkdir(parents=True, exist_ok=True)
        (d / "model.config").write_text(CONFIG_TEMPLATE.format(name=name, description=desc))
        (d / "model.sdf").write_text(SDF_TEMPLATE.format(name=name))
        print(f"  {name}/model.config + model.sdf")
    print("Done.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it**

```bash
python3 tools/create_models.py
```

Expected: prints 6 model names then `Done.`

- [ ] **Step 3: Create worlds directory and tag_demo.sdf**

```bash
mkdir -p ~/Aero/Simulation/worlds
```

Create `Simulation/worlds/tag_demo.sdf`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="tag_demo">

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>1 1</size></plane>
          </geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
      </link>
      <pose>0 0 0 0 0 0</pose>
      <self_collide>false</self_collide>
    </model>

    <!-- Directional sun light (same params as aruco.sdf) -->
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range><linear>0</linear>
        <constant>1</constant><quadratic>0</quadratic>
      </attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>

    <!-- GPS origin — same as aruco.sdf so MAVLink/EKF behaves identically -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <!--
      Marker layout (top-down, x=north, y=east):
        ( 5,  5)  ArUco 4x4 ID0       ( 5,  0)  AprilTag 36h11 ID0    ( 5, -5)  ArUco 4x4 ID1
        (-5,  5)  AprilTag 36h11 ID1  (-5,  0)  ArUco 4x4 ID2         (-5, -5)  AprilTag 36h11 ID2
    -->
    <include>
      <uri>model://aruco_4x4_id0</uri>
      <name>aruco_id0</name>
      <pose>5 5 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://apriltag_36h11_id0</uri>
      <name>april_id0</name>
      <pose>5 0 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://aruco_4x4_id1</uri>
      <name>aruco_id1</name>
      <pose>5 -5 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://apriltag_36h11_id1</uri>
      <name>april_id1</name>
      <pose>-5 5 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://aruco_4x4_id2</uri>
      <name>aruco_id2</name>
      <pose>-5 0 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://apriltag_36h11_id2</uri>
      <name>april_id2</name>
      <pose>-5 -5 0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

- [ ] **Step 4: Smoke-test world loads in Gazebo**

```bash
# Terminal 1 — headless sim with custom models path
cd ~/Aero/Simulation && source .venv/bin/activate
GZ_SIM_RESOURCE_PATH=$HOME/Aero/Simulation/models \
PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds \
python3 sim.py --world tag_demo --gui
```

Expected: Gazebo GUI opens showing a flat ground with 6 flat marker tiles. No "model not found" errors in the server output.

- [ ] **Step 5: Commit**

```bash
git add Simulation/tools/create_models.py Simulation/models/ Simulation/worlds/
git commit -m "feat: add Gazebo marker models and tag_demo world"
```

---

## Task 4: Create `tag_detector.py` with unit tests (TDD)

**Files:**
- Create: `Simulation/algorithms/tag_detector.py`
- Create: `Simulation/tests/__init__.py`
- Create: `Simulation/tests/test_tag_detector.py`

- [ ] **Step 1: Create tests directory**

```bash
mkdir -p ~/Aero/Simulation/tests
touch ~/Aero/Simulation/tests/__init__.py
```

- [ ] **Step 2: Write failing tests**

Create `Simulation/tests/test_tag_detector.py`:

```python
"""Unit tests for TagDetector.

Run from Simulation/:
    pytest tests/test_tag_detector.py -v
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))
from algorithms.tag_detector import Detection, TagDetector

# ── Helpers ───────────────────────────────────────────────────────────────────

W, H = 640, 640
MARKER_MM = 200.0  # physical size used in synthetic test images

# Simple pinhole camera matrix for a 640×640 image, ~90° FOV
_MTX = np.array([
    [320.0,   0.0, 320.0],
    [  0.0, 320.0, 320.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)
_DIST = np.zeros((5, 1), dtype=np.float64)


def make_aruco_frame(marker_id: int, size_px: int = 300) -> np.ndarray:
    """White 640×640 image with an ArUco 4x4_50 marker centred."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(d, marker_id, size_px)
    frame = np.full((H, W), 255, dtype=np.uint8)
    offset = (W - size_px) // 2
    frame[offset: offset + size_px, offset: offset + size_px] = marker
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def make_apriltag_frame(marker_id: int, size_px: int = 300) -> np.ndarray:
    """White 640×640 image with a cv2 DICT_APRILTAG_36h11 marker centred."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(d, marker_id, size_px)
    frame = np.full((H, W), 255, dtype=np.uint8)
    offset = (W - size_px) // 2
    frame[offset: offset + size_px, offset: offset + size_px] = marker
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


# ── Tests ─────────────────────────────────────────────────────────────────────

class TestTagDetectorAruco:
    def setup_method(self):
        self.det = TagDetector(
            marker_size_mm=MARKER_MM,
            camera_matrix=_MTX,
            dist_coeffs=_DIST,
            aruco_dicts=["4x4_50"],
            apriltag_families=[],
        )

    def test_detects_aruco_marker(self):
        frame = make_aruco_frame(marker_id=0)
        results = self.det.detect(frame)
        assert len(results) == 1
        d = results[0]
        assert d.type == "aruco"
        assert d.family == "4x4_50"
        assert d.marker_id == 0

    def test_distance_is_positive(self):
        frame = make_aruco_frame(marker_id=0)
        results = self.det.detect(frame)
        assert results[0].distance_mm > 0

    def test_corners_shape(self):
        frame = make_aruco_frame(marker_id=0)
        results = self.det.detect(frame)
        assert results[0].corners.shape == (1, 4, 2)
        assert results[0].corners.dtype == np.float32

    def test_rvec_tvec_shapes(self):
        frame = make_aruco_frame(marker_id=0)
        results = self.det.detect(frame)
        assert results[0].rvec.shape == (3, 1)
        assert results[0].tvec.shape == (3, 1)

    def test_no_detection_on_blank_frame(self):
        blank = np.full((H, W, 3), 200, dtype=np.uint8)
        results = self.det.detect(blank)
        assert results == []

    def test_different_marker_ids(self):
        for mid in [0, 1, 2]:
            frame = make_aruco_frame(marker_id=mid)
            results = self.det.detect(frame)
            assert len(results) == 1
            assert results[0].marker_id == mid

    def test_draw_does_not_raise(self):
        frame = make_aruco_frame(marker_id=0)
        results = self.det.detect(frame)
        self.det.draw(frame, results)  # must not raise


class TestTagDetectorAprilTag:
    def setup_method(self):
        self.det = TagDetector(
            marker_size_mm=MARKER_MM,
            camera_matrix=_MTX,
            dist_coeffs=_DIST,
            aruco_dicts=[],
            apriltag_families=["tag36h11"],
        )

    def test_detects_apriltag_marker(self):
        frame = make_apriltag_frame(marker_id=0)
        results = self.det.detect(frame)
        assert len(results) == 1
        d = results[0]
        assert d.type == "apriltag"
        assert d.family == "tag36h11"
        assert d.marker_id == 0

    def test_distance_is_positive(self):
        frame = make_apriltag_frame(marker_id=0)
        results = self.det.detect(frame)
        assert results[0].distance_mm > 0

    def test_corners_shape(self):
        frame = make_apriltag_frame(marker_id=0)
        results = self.det.detect(frame)
        assert results[0].corners.shape == (1, 4, 2)
        assert results[0].corners.dtype == np.float32


class TestTagDetectorBoth:
    def test_aruco_and_apriltag_simultaneously(self):
        det = TagDetector(
            marker_size_mm=MARKER_MM,
            camera_matrix=_MTX,
            dist_coeffs=_DIST,
            aruco_dicts=["4x4_50"],
            apriltag_families=["tag36h11"],
        )
        # ArUco frame: should find 1 aruco, 0 apriltag
        frame = make_aruco_frame(marker_id=0)
        results = det.detect(frame)
        types = [r.type for r in results]
        assert "aruco" in types

    def test_no_detectors_returns_empty(self):
        det = TagDetector(
            marker_size_mm=MARKER_MM,
            camera_matrix=_MTX,
            dist_coeffs=_DIST,
            aruco_dicts=[],
            apriltag_families=[],
        )
        frame = make_aruco_frame(marker_id=0)
        assert det.detect(frame) == []
```

- [ ] **Step 3: Run tests — expect all to FAIL (module missing)**

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
pytest tests/test_tag_detector.py -v 2>&1 | head -20
```

Expected: `ImportError: cannot import name 'TagDetector' from 'algorithms.tag_detector'` or `ModuleNotFoundError`.

- [ ] **Step 4: Implement `tag_detector.py`**

Create `Simulation/algorithms/tag_detector.py`:

```python
"""
tag_detector.py — unified ArUco + AprilTag detector for simulation.

Usage (importable):
    from algorithms.tag_detector import TagDetector, Detection
    det = TagDetector(500.0, camera_matrix, dist_coeffs,
                      aruco_dicts=["4x4_50"], apriltag_families=["tag36h11"])
    detections = det.detect(frame)   # list[Detection]
    det.draw(frame, detections)
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import cv2.aruco as aruco
import numpy as np

try:
    from pupil_apriltags import Detector as _AprilDetector
    _APRIL_AVAILABLE = True
except ImportError:
    _APRIL_AVAILABLE = False

# ── ArUco dictionary registry (subset; extend as needed) ─────────────────────
_ARUCO_REGISTRY: dict[str, int] = {
    "4x4_50":            cv2.aruco.DICT_4X4_50,
    "4x4_100":           cv2.aruco.DICT_4X4_100,
    "4x4_250":           cv2.aruco.DICT_4X4_250,
    "5x5_50":            cv2.aruco.DICT_5X5_50,
    "5x5_100":           cv2.aruco.DICT_5X5_100,
    "6x6_50":            cv2.aruco.DICT_6X6_50,
    "cv_apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

# Colour palettes: ArUco = green tones, AprilTag = cyan/yellow tones
_ARUCO_COLOURS = [(0, 255, 0), (50, 220, 0), (100, 255, 80)]
_APRIL_COLOURS = [(0, 220, 255), (0, 180, 220), (50, 255, 220)]


# ── Detection result ──────────────────────────────────────────────────────────

@dataclass
class Detection:
    type: str            # "aruco" or "apriltag"
    family: str          # e.g. "4x4_50", "tag36h11"
    marker_id: int
    distance_mm: float   # L2 norm of tvec
    corners: np.ndarray  # shape (1, 4, 2), float32, TL/TR/BR/BL order
    rvec: np.ndarray     # shape (3, 1)
    tvec: np.ndarray     # shape (3, 1), units mm
    colour: tuple        # BGR


# ── Detector ──────────────────────────────────────────────────────────────────

class TagDetector:
    """
    Detects ArUco and AprilTag markers in a single BGR frame.

    Args:
        marker_size_mm:    Physical side length of markers in mm.
        camera_matrix:     3×3 intrinsic matrix (float64).
        dist_coeffs:       Distortion coefficients (float64).
        aruco_dicts:       List of dict names from _ARUCO_REGISTRY.
        apriltag_families: List of pupil_apriltags family strings.
    """

    def __init__(
        self,
        marker_size_mm: float,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        aruco_dicts: list[str] | None = None,
        apriltag_families: list[str] | None = None,
    ) -> None:
        self.marker_size_mm = marker_size_mm
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self._tag_size_m = marker_size_mm / 1000.0

        h = marker_size_mm / 2
        self._obj_pts = np.array(
            [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32
        )

        # ArUco detectors
        self._aruco: list[dict] = []
        for i, name in enumerate(aruco_dicts or []):
            if name not in _ARUCO_REGISTRY:
                raise ValueError(f"Unknown aruco dict: {name!r}")
            d = aruco.getPredefinedDictionary(_ARUCO_REGISTRY[name])
            p = aruco.DetectorParameters()
            self._aruco.append({
                "name": name,
                "det": aruco.ArucoDetector(d, p),
                "colour": _ARUCO_COLOURS[i % len(_ARUCO_COLOURS)],
            })

        # AprilTag detectors — one per family
        self._april: list[dict] = []
        for i, fam in enumerate(apriltag_families or []):
            if not _APRIL_AVAILABLE:
                raise ImportError("pupil_apriltags not installed: pip install pupil-apriltags")
            self._april.append({
                "family": fam,
                "det": _AprilDetector(families=fam),
                "colour": _APRIL_COLOURS[i % len(_APRIL_COLOURS)],
            })

    # ── Public API ────────────────────────────────────────────────────────────

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Run all detectors on `frame` (BGR). Returns list of Detection."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results: list[Detection] = []
        results.extend(self._detect_aruco(gray))
        results.extend(self._detect_april(gray))
        return results

    def draw(self, frame: np.ndarray, detections: list[Detection]) -> None:
        """Annotate `frame` in-place with brackets, axes, and distance labels."""
        if detections:
            b = 6
            cv2.rectangle(
                frame, (b, b), (frame.shape[1] - b, frame.shape[0] - b), (0, 255, 0), b
            )
        for det in detections:
            pts = det.corners[0].astype(int)
            self._draw_brackets(frame, pts, det.colour)
            cv2.drawFrameAxes(
                frame, self.camera_matrix, self.dist_coeffs,
                det.rvec, det.tvec, self.marker_size_mm / 2,
            )
            label = f"[{det.type}/{det.family}] ID{det.marker_id}  {det.distance_mm:.0f}mm"
            cv2.putText(
                frame, label, (pts[0][0], pts[0][1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, det.colour, 2,
            )

    # ── Private helpers ───────────────────────────────────────────────────────

    def _detect_aruco(self, gray: np.ndarray) -> list[Detection]:
        out = []
        for d in self._aruco:
            corners, ids, _ = d["det"].detectMarkers(gray)
            if ids is None or len(ids) == 0:
                continue
            for i, mid in enumerate(ids):
                c = corners[i]  # (1, 4, 2) float32
                _, rvec, tvec = cv2.solvePnP(
                    self._obj_pts, c[0], self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                out.append(Detection(
                    type="aruco",
                    family=d["name"],
                    marker_id=int(mid[0]),
                    distance_mm=float(np.linalg.norm(tvec)),
                    corners=c,
                    rvec=rvec,
                    tvec=tvec,
                    colour=d["colour"],
                ))
        return out

    def _detect_april(self, gray: np.ndarray) -> list[Detection]:
        if not self._april:
            return []
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        out = []
        for d in self._april:
            tags = d["det"].detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(fx, fy, cx, cy),
                tag_size=self._tag_size_m,
            )
            for tag in tags:
                corners = tag.corners.reshape(1, 4, 2).astype(np.float32)
                tvec_mm = (tag.pose_t.flatten() * 1000.0).reshape(3, 1)
                rvec, _ = cv2.Rodrigues(tag.pose_R)
                out.append(Detection(
                    type="apriltag",
                    family=d["family"],
                    marker_id=tag.tag_id,
                    distance_mm=float(np.linalg.norm(tvec_mm)),
                    corners=corners,
                    rvec=rvec,
                    tvec=tvec_mm,
                    colour=d["colour"],
                ))
        return out

    @staticmethod
    def _draw_brackets(frame: np.ndarray, pts: np.ndarray, colour: tuple) -> None:
        blen = max(12, int(np.linalg.norm(pts[1] - pts[0]) * 0.25))
        for j, pt in enumerate(pts):
            for nb in [pts[(j + 1) % 4], pts[(j - 1) % 4]]:
                d = (nb - pt).astype(float)
                n = d / (np.linalg.norm(d) + 1e-6)
                cv2.line(frame, tuple(pt), tuple((pt + n * blen).astype(int)), colour, 3)
```

- [ ] **Step 5: Run tests — expect all to PASS**

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
pytest tests/test_tag_detector.py -v
```

Expected: all tests pass. If `test_detects_apriltag_marker` fails with a detection count issue, check that the AprilTag PNG from Task 2 matches the bit pattern `pupil_apriltags` expects — they both use the same tag36h11 standard.

- [ ] **Step 6: Commit**

```bash
git add Simulation/algorithms/tag_detector.py Simulation/tests/
git commit -m "feat: add TagDetector with ArUco + AprilTag support, all tests passing"
```

---

## Task 5: Create `tag_patrol.py`

**Files:**
- Create: `Simulation/algorithms/tag_patrol.py`

- [ ] **Step 1: Create the script**

Create `Simulation/algorithms/tag_patrol.py`:

```python
"""
Tag patrol demo — autonomous waypoint circuit with ArUco + AprilTag detection.

Usage:
    cd ~/Aero/Simulation && source .venv/bin/activate

    # Start sim first (separate terminal):
    GZ_SIM_RESOURCE_PATH=$HOME/Aero/Simulation/models \\
    PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds \\
    python3 sim.py --world tag_demo

    # Then run patrol:
    python3 algorithms/tag_patrol.py --world tag_demo

Controls:
    Q  in OpenCV window — graceful shutdown (lands drone)
"""
import asyncio
import sys
import threading
import time
from pathlib import Path

import argparse
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

import config
from connection_test import connect_drone_ctx
from algorithms.camera_feed import SimCamera, load_calibration
from algorithms.tag_detector import TagDetector

from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw

# ── Constants ─────────────────────────────────────────────────────────────────
VEHICLE          = "x500_mono_cam_down"
MARKER_SIZE_MM   = 500.0
ARUCO_DICTS      = ["4x4_50"]
APRILTAG_FAMILIES = ["tag36h11"]
SETPOINT_HZ      = 10
ACCEPTANCE_M     = 0.5     # waypoint acceptance radius
HOVER_SEC        = 2.0     # dwell time at each waypoint
IN_AIR_TIMEOUT   = 30
LAND_TIMEOUT     = 60


# ── Waypoints ─────────────────────────────────────────────────────────────────

def make_waypoints(altitude_m: float, spacing_m: float) -> list[tuple]:
    """Square circuit at `altitude_m` AGL covering the 2×3 marker grid."""
    s, d = spacing_m, -altitude_m
    return [
        ( s,  s, d),   # NE
        ( s, -s, d),   # NW
        (-s, -s, d),   # SW
        (-s,  s, d),   # SE
    ]


# ── Flight helpers ────────────────────────────────────────────────────────────

async def _wait_in_air(drone, state: bool, timeout: float, label: str) -> None:
    async def _watch():
        async for in_air in drone.telemetry.in_air():
            if in_air == state:
                return
    try:
        await asyncio.wait_for(_watch(), timeout=timeout)
    except asyncio.TimeoutError:
        raise TimeoutError(f"Timed out waiting for {label} after {timeout}s")


async def _get_ned_position(drone):
    """Return (north_m, east_m, down_m) from the first telemetry sample."""
    async for pv in drone.telemetry.position_velocity_ned():
        return pv.position.north_m, pv.position.east_m, pv.position.down_m


async def fly_to_ned(
    drone, n: float, e: float, d: float, shutdown_event: threading.Event
) -> None:
    """Stream NED setpoint at SETPOINT_HZ until within ACCEPTANCE_M or shutdown."""
    target = PositionNedYaw(n, e, d, 0.0)
    period = 1.0 / SETPOINT_HZ
    while not shutdown_event.is_set():
        await drone.offboard.set_position_ned(target)
        try:
            cn, ce, cd = await asyncio.wait_for(_get_ned_position(drone), timeout=2.0)
        except asyncio.TimeoutError:
            await asyncio.sleep(period)
            continue
        dist = ((cn - n) ** 2 + (ce - e) ** 2 + (cd - d) ** 2) ** 0.5
        if dist < ACCEPTANCE_M:
            return
        await asyncio.sleep(period)


# ── Patrol coroutine ──────────────────────────────────────────────────────────

async def run_patrol(
    args: argparse.Namespace,
    shutdown_event: threading.Event,
    waypoint_idx: list,         # [int] — mutable for display thread to read
) -> None:
    waypoints = make_waypoints(args.altitude, args.spacing)

    try:
        async with connect_drone_ctx() as drone:
            print("[patrol] Waiting for GPS fix...")
            async for health in drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    break

            print(f"[patrol] Arming + taking off to {args.altitude}m AGL...")
            await drone.action.set_takeoff_altitude(args.altitude)
            await drone.action.arm()
            await drone.action.takeoff()
            await _wait_in_air(drone, True, IN_AIR_TIMEOUT, "airborne")
            print("[patrol] Airborne. Entering offboard mode.")

            # Prime offboard with a hold setpoint at current altitude
            hold = PositionNedYaw(0.0, 0.0, -args.altitude, 0.0)
            await drone.offboard.set_position_ned(hold)
            await drone.offboard.start()

            for i, (n, e, d) in enumerate(waypoints):
                if shutdown_event.is_set():
                    break
                waypoint_idx[0] = i
                print(f"[patrol] Waypoint {i + 1}/{len(waypoints)}: N={n} E={e} D={d}")
                await fly_to_ned(drone, n, e, d, shutdown_event)
                await asyncio.sleep(HOVER_SEC)

            print("[patrol] Circuit complete. Landing...")
            try:
                await drone.offboard.stop()
            except OffboardError:
                pass
            await drone.action.land()
            await _wait_in_air(drone, False, LAND_TIMEOUT, "landed")
            try:
                await drone.action.disarm()
            except ActionError:
                pass
            print("[patrol] Landed and disarmed.")

    except Exception as e:
        print(f"[patrol] ERROR: {e}")
    finally:
        shutdown_event.set()


# ── Display loop ──────────────────────────────────────────────────────────────

def display_loop(
    cam: SimCamera,
    detector: TagDetector,
    shutdown_event: threading.Event,
    waypoint_idx: list,
    n_waypoints: int,
) -> None:
    cv2.namedWindow("Tag Patrol", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Tag Patrol", 1280, 960)

    frame_budget_ms = 1000.0 / 30
    total_seen = 0

    while not shutdown_event.is_set():
        t0 = time.monotonic()
        ok, frame = cam.read()
        if not ok:
            time.sleep(0.01)
            continue

        detections = detector.detect(frame)
        detector.draw(frame, detections)
        total_seen += len(detections)

        # Status overlay
        wp = waypoint_idx[0] + 1
        cv2.putText(frame, f"Waypoint {wp}/{n_waypoints}",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(frame, f"Detections this frame: {len(detections)}",
                    (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        elapsed_ms = (time.monotonic() - t0) * 1000
        lag_col = (0, 60, 255) if elapsed_ms > frame_budget_ms else (120, 120, 120)
        cv2.putText(frame, f"{elapsed_ms:.1f}ms",
                    (12, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, lag_col, 1)

        cv2.imshow("Tag Patrol", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("[display] Q pressed — shutting down.")
            shutdown_event.set()

    cv2.destroyAllWindows()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--world",    default="tag_demo",
                        help="Gazebo world name (default: tag_demo)")
    parser.add_argument("--altitude", type=float, default=3.0,
                        help="Patrol altitude in metres AGL (default: 3.0)")
    parser.add_argument("--spacing",  type=float, default=7.0,
                        help="Waypoint square half-side in metres (default: 7.0)")
    parser.add_argument("--calib",    default="calibration.npz",
                        help="Path to calibration.npz (default: calibration.npz)")
    args = parser.parse_args()

    # Camera intrinsics — falls back to synthetic sim values if no .npz
    mtx, dist = load_calibration(args.calib)

    detector = TagDetector(
        marker_size_mm=MARKER_SIZE_MM,
        camera_matrix=mtx,
        dist_coeffs=dist,
        aruco_dicts=ARUCO_DICTS,
        apriltag_families=APRILTAG_FAMILIES,
    )

    cam = SimCamera(world=args.world, vehicle=VEHICLE)
    cam.start()
    print(f"[main] Waiting for first frame from {cam.topic} ...")
    if not cam.wait_for_frame(timeout=15):
        print("[ERROR] No frames received — is the simulation running?")
        print(f"  Start with: GZ_SIM_RESOURCE_PATH=$HOME/Aero/Simulation/models "
              f"PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds python3 sim.py --world {args.world}")
        cam.stop()
        sys.exit(1)

    shutdown_event = threading.Event()
    waypoint_idx = [0]
    waypoints = make_waypoints(args.altitude, args.spacing)

    patrol_thread = threading.Thread(
        target=lambda: asyncio.run(
            run_patrol(args, shutdown_event, waypoint_idx)
        ),
        daemon=True,
        name="patrol",
    )
    patrol_thread.start()

    display_loop(cam, detector, shutdown_event, waypoint_idx, len(waypoints))

    patrol_thread.join(timeout=30)
    cam.stop()
    print("[main] Done.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Syntax check**

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
python3 -c "import algorithms.tag_patrol; print('syntax ok')"
```

Expected: `syntax ok`

- [ ] **Step 3: Commit**

```bash
git add Simulation/algorithms/tag_patrol.py
git commit -m "feat: add tag_patrol autonomous waypoint + detection demo"
```

---

## Task 6: End-to-end smoke test (manual — requires running simulation)

This is a manual acceptance test. It cannot be automated without a running PX4 + Gazebo stack. Follow these steps in order.

- [ ] **Step 1: Start the simulation (Terminal 1)**

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
GZ_SIM_RESOURCE_PATH=$HOME/Aero/Simulation/models \
PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds \
python3 sim.py --world tag_demo
```

`sim.py` calls `launch_sim.sh` via `subprocess.Popen` without an explicit `env=` argument, so `GZ_SIM_RESOURCE_PATH` is automatically inherited by the child process and passed through to Gazebo. No changes to `sim.py` or `launch_sim.sh` are needed.

Wait for: `[OK] Simulation is ready.`

- [ ] **Step 2: Run the patrol demo (Terminal 2)**

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
python3 algorithms/tag_patrol.py --world tag_demo
```

Expected sequence in terminal:
```
[main] Waiting for first frame from /world/tag_demo/model/...
[patrol] Waiting for GPS fix...
[patrol] Arming + taking off to 3.0m AGL...
[patrol] Airborne. Entering offboard mode.
[patrol] Waypoint 1/4: N=7.0 E=7.0 D=-3.0
...
```

Expected in OpenCV window:
- Live camera feed from downward camera
- Green annotation boxes with `[aruco/4x4_50] ID0  XXXX mm` labels when drone passes over markers
- Cyan annotation boxes with `[apriltag/tag36h11] ID0  XXXX mm` labels for AprilTags
- `Waypoint N/4` overlay updating as drone moves

- [ ] **Step 3: Verify clean shutdown**

Press `Q` in the OpenCV window.

Expected:
```
[display] Q pressed — shutting down.
[patrol] Circuit complete. Landing...
[patrol] Landed and disarmed.
[main] Done.
```

- [ ] **Step 4: Run unit tests one final time**

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
pytest tests/test_tag_detector.py -v
```

Expected: all green.

---

## Quick Reference

```bash
# Install deps
cd ~/Aero/Simulation && source .venv/bin/activate && uv sync

# Generate marker images + model files (one-time setup)
python3 tools/generate_markers.py
python3 tools/create_models.py

# Run demo (two terminals)
# T1:
GZ_SIM_RESOURCE_PATH=$HOME/Aero/Simulation/models \
PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds \
python3 sim.py --world tag_demo

# T2:
python3 algorithms/tag_patrol.py --world tag_demo

# Optional flags
python3 algorithms/tag_patrol.py --world tag_demo --altitude 5.0 --spacing 10.0
```
