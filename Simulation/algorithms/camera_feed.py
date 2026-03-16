"""
Gazebo simulated camera feed with ArUco marker detection.

Mirrors the detection logic of Code/Callibration/3_aruco_test.py, using the
SimCamera gz-transport subscriber as the frame source instead of rpicam-vid/USB cam.

Usage (standalone viewer):
    cd Simulation && source .venv/bin/activate
    python3 algorithms/camera_feed.py [--world forest] [--vehicle x500_mono_cam]
    # Q = quit   V = log distance to CSV

Importable:
    from algorithms.camera_feed import SimCamera
    cam = SimCamera(world="forest")
    cam.start()
    ok, frame = cam.read()   # (bool, BGR numpy array)
    cam.stop()
"""

import csv
import math
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

# ── gz-transport is a system package, not in the venv ────────────────────────
GZ_PYTHON = "/usr/lib/python3/dist-packages"
if GZ_PYTHON not in sys.path:
    sys.path.insert(0, GZ_PYTHON)

try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image as GzImage
except ImportError as e:
    print(f"[ERROR] gz-transport13 / gz-msgs10 not found: {e}")
    print("  Install: sudo apt-get install python3-gz-transport13 python3-gz-msgs10")
    sys.exit(1)

sys.path.insert(0, str(Path(__file__).parent.parent))
import config

# ── ArUco settings (match the printed/simulated marker) ──────────────────────
MARKER_SIZE_MM   = 100.0
ARUCO_DICT_TYPE  = cv2.aruco.DICT_4X4_50
CSV_FILE         = "aruco_distance_results.csv"

# Sim camera intrinsics (1280×960, FOV 1.74 rad) — used when no calibration.npz
_SIM_W, _SIM_H = 1280, 960
_SIM_FX = _SIM_W / (2 * math.tan(1.74 / 2))
SIM_CAMERA_MATRIX = np.array([
    [_SIM_FX,    0,    _SIM_W / 2],
    [   0,    _SIM_FX, _SIM_H / 2],
    [   0,       0,        1     ],
], dtype=np.float64)
SIM_DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)

# Marker corner object points for solvePnP
_half = MARKER_SIZE_MM / 2
MARKER_OBJ_POINTS = np.array([
    [-_half,  _half, 0],
    [ _half,  _half, 0],
    [ _half, -_half, 0],
    [-_half, -_half, 0],
], dtype=np.float32)

# Camera topic template
TOPIC_TEMPLATE = (
    "/world/{world}/model/{vehicle}_0"
    "/link/camera_link/sensor/camera/image"
)

# Pixel format → (dtype, channels)
_FMT = {
    3: (np.uint8, 3),   # RGB_INT8
    8: (np.uint8, 3),   # BGR_INT8
    1: (np.uint8, 1),   # L_INT8
}


# ── Calibration ───────────────────────────────────────────────────────────────

def load_calibration(filepath: str) -> tuple[np.ndarray, np.ndarray]:
    """Load camera matrix and distortion from calibration.npz.

    Falls back to synthetic sim camera intrinsics if the file is missing.
    """
    path = Path(filepath)
    if not path.exists():
        # Search relative to repo root as well
        repo_root = Path(__file__).parent.parent.parent
        alt = repo_root / "calibration.npz"
        if alt.exists():
            path = alt
        else:
            print(f"[WARN] calibration.npz not found — using synthetic sim camera matrix.")
            print(f"  Run Code/Callibration/2_calibrate.py to generate a real calibration.")
            return SIM_CAMERA_MATRIX.copy(), SIM_DIST_COEFFS.copy()

    with np.load(str(path)) as data:
        mtx  = data["K"]
        dist = data["dist"]
        calib_size = tuple(data["image_size"]) if "image_size" in data else None

    print(f"[INFO] Loaded calibration from {path}")

    # Scale K if calibration resolution differs from sim camera resolution
    if calib_size and (calib_size[0] != _SIM_W or calib_size[1] != _SIM_H):
        sx = _SIM_W / calib_size[0]
        sy = _SIM_H / calib_size[1]
        mtx = mtx.copy()
        mtx[0, 0] *= sx; mtx[0, 2] *= sx
        mtx[1, 1] *= sy; mtx[1, 2] *= sy
        print(f"[INFO] Scaled K from {calib_size} → {(_SIM_W, _SIM_H)}")

    return mtx, dist


# ── ArUco helpers ─────────────────────────────────────────────────────────────

def build_detector():
    """Build ArucoDetector, falling back to the old API for older OpenCV."""
    aruco_dict  = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    params      = cv2.aruco.DetectorParameters()
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector, aruco_dict, params, True
    except AttributeError:
        return None, aruco_dict, params, False


def estimate_pose(corners, mtx, dist):
    rvecs, tvecs = [], []
    for c in corners:
        _, rvec, tvec = cv2.solvePnP(
            MARKER_OBJ_POINTS, c[0], mtx, dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        rvecs.append(rvec)
        tvecs.append(tvec)
    return rvecs, tvecs


def draw_detections(frame, corners, ids, rvecs, tvecs, mtx, dist):
    """Draw brackets, axes, distance labels and green border — same style as 3_aruco_test.py."""
    border = 6
    cv2.rectangle(frame, (border, border),
                  (frame.shape[1] - border, frame.shape[0] - border),
                  (0, 255, 0), border)

    for i, marker_id in enumerate(ids):
        pts = corners[i][0].astype(int)

        bracket_len   = max(12, int(np.linalg.norm(pts[1] - pts[0]) * 0.25))
        bracket_color = (0, 255, 0)
        for j, pt in enumerate(pts):
            next_pt = pts[(j + 1) % 4]
            prev_pt = pts[(j - 1) % 4]
            dn = (next_pt - pt).astype(float); dn /= np.linalg.norm(dn) + 1e-6
            dp = (prev_pt - pt).astype(float); dp /= np.linalg.norm(dp) + 1e-6
            cv2.line(frame, tuple(pt), tuple((pt + dn * bracket_len).astype(int)), bracket_color, 3)
            cv2.line(frame, tuple(pt), tuple((pt + dp * bracket_len).astype(int)), bracket_color, 3)

        cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], MARKER_SIZE_MM / 2)
        distance = float(np.linalg.norm(tvecs[i]))
        cv2.putText(frame, f"ID{marker_id[0]}  {distance:.1f}mm",
                    (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)


def append_csv_row(estimate_mm, real_mm):
    error_mm      = estimate_mm - real_mm
    error_percent = abs(error_mm) / real_mm * 100 if real_mm else float("inf")
    file_exists   = os.path.isfile(CSV_FILE)
    with open(CSV_FILE, mode="a", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["estimate_mm", "real_mm", "error_mm", "error_percent"]
        )
        if not file_exists:
            writer.writeheader()
        writer.writerow({
            "estimate_mm":   round(estimate_mm, 2),
            "real_mm":       round(real_mm, 2),
            "error_mm":      round(error_mm, 2),
            "error_percent": round(error_percent, 2),
        })
    return error_mm, error_percent


# ── SimCamera ─────────────────────────────────────────────────────────────────

class SimCamera:
    """
    Thread-safe Gazebo camera subscriber.  call read() to get the latest BGR frame.
    Same interface as cv2.VideoCapture so it can be used as a drop-in.
    """

    def __init__(self, world: str = config.WORLD, vehicle: str = config.VEHICLE):
        self.topic  = TOPIC_TEMPLATE.format(world=world, vehicle=vehicle)
        self._frame: np.ndarray | None = None
        self._lock  = threading.Lock()
        self._node: Node | None = None
        self._running = False

    def start(self):
        self._node = Node()
        self._node.subscribe(GzImage, self.topic, self._on_image)
        self._running = True
        print(f"[SimCamera] Subscribed to {self.topic}")

    def stop(self):
        if self._node and self._running:
            self._node.unsubscribe(self.topic)
            self._running = False

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Return (ok, bgr_frame) — same interface as cv2.VideoCapture.read()."""
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    def wait_for_frame(self, timeout: float = 15.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._frame is not None:
                    return True
            time.sleep(0.05)
        return False

    def _on_image(self, msg: GzImage):
        fmt = _FMT.get(msg.pixel_format_type)
        if fmt is None:
            return
        dtype, channels = fmt
        raw = np.frombuffer(msg.data, dtype=dtype)

        if channels == 1:
            img = raw.reshape((msg.height, msg.width))
            bgr = np.stack([img, img, img], axis=-1)
        else:
            img = raw.reshape((msg.height, msg.width, channels))
            # gz publishes RGB; OpenCV expects BGR
            bgr = img[:, :, ::-1].copy()

        with self._lock:
            self._frame = bgr


# ── Standalone viewer with ArUco detection ────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Sim camera feed with ArUco detection")
    parser.add_argument("--world",   default=config.WORLD,   help="Gazebo world name")
    parser.add_argument("--vehicle", default=config.VEHICLE, help="Vehicle model name")
    parser.add_argument("--calib",   default="calibration.npz",
                        help="Path to calibration.npz (default: calibration.npz)")
    parser.add_argument("--marker-size", type=float, default=MARKER_SIZE_MM,
                        help=f"ArUco marker side length in mm (default: {MARKER_SIZE_MM})")
    args = parser.parse_args()

    mtx, dist = load_calibration(args.calib)
    detector, aruco_dict, aruco_params, use_new_api = build_detector()

    cam = SimCamera(world=args.world, vehicle=args.vehicle)
    cam.start()

    print(f"Waiting for first frame on {cam.topic} ...")
    if not cam.wait_for_frame(timeout=15):
        print("[ERROR] No frames received. Is the simulation running?")
        print(f"  Start with: python3 sim.py --world {args.world} --gui")
        cam.stop()
        sys.exit(1)

    print("Receiving frames.  Q = quit   V = log distance to CSV")

    cv2.namedWindow("Sim Camera — ArUco", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Sim Camera — ArUco", 1280, 960)

    frame_budget_ms  = 1000.0 / 30
    rvecs, tvecs, ids = [], [], None  # persist for V handler

    while True:
        t_start = time.monotonic()
        ok, frame = cam.read()
        if not ok:
            time.sleep(0.01)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if use_new_api:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        detected = ids is not None and len(ids) > 0

        if detected:
            rvecs, tvecs = estimate_pose(corners, mtx, dist)
            draw_detections(frame, corners, ids, rvecs, tvecs, mtx, dist)
            status = f"DETECTED: {len(ids)} marker{'s' if len(ids) > 1 else ''}"
            cv2.putText(frame, status, (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "SCANNING...", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 100, 255), 2)

        elapsed_ms = (time.monotonic() - t_start) * 1000
        lag_color  = (0, 60, 255) if elapsed_ms > frame_budget_ms else (180, 180, 180)
        cv2.putText(frame, f"{elapsed_ms:.1f}ms / {frame_budget_ms:.0f}ms budget",
                    (12, frame.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, lag_color, 1)

        cv2.imshow("Sim Camera — ArUco", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("v"):
            if ids is not None and len(ids) > 0:
                print(f"\n--- Verification ({len(ids)} markers) ---")
                for i, marker_id in enumerate(ids):
                    calc_dist = float(np.linalg.norm(tvecs[i]))
                    print(f"Marker ID {marker_id[0]}  estimated: {calc_dist:.2f} mm")
                    try:
                        raw = input(f"  Enter TRUE distance for Marker {marker_id[0]} (mm): ")
                        if not raw.strip():
                            print("  Skipped.")
                            continue
                        err, pct = append_csv_row(calc_dist, float(raw))
                        print(f"  Error: {err:+.2f} mm ({pct:.2f}%)  → saved to {CSV_FILE}")
                    except ValueError:
                        print("  Invalid input. Skipping.")
                print("--------------------\n")
            else:
                print("No markers detected.")

    cam.stop()
    cv2.destroyAllWindows()
