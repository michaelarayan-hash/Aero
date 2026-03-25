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
MARKER_MM = 200.0

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
    """White 640×640 image with a DICT_APRILTAG_36h11 marker centred."""
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
