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

# ── ArUco dictionary registry ─────────────────────────────────────────────────
_ARUCO_REGISTRY: dict[str, int] = {
    "4x4_50":            cv2.aruco.DICT_4X4_50,
    "4x4_100":           cv2.aruco.DICT_4X4_100,
    "4x4_250":           cv2.aruco.DICT_4X4_250,
    "5x5_50":            cv2.aruco.DICT_5X5_50,
    "5x5_100":           cv2.aruco.DICT_5X5_100,
    "6x6_50":            cv2.aruco.DICT_6X6_50,
    "cv_apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

# Colour palettes: ArUco = green tones, AprilTag = cyan tones
_ARUCO_COLOURS = [(0, 255, 0), (50, 220, 0), (100, 255, 80)]
_APRIL_COLOURS = [(0, 220, 255), (0, 180, 220), (50, 255, 220)]


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
