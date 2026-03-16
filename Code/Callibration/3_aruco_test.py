import csv
import cv2
import cv2.aruco as aruco
import numpy as np
import subprocess
import os
import sys
import time

# ── Settings ────────────────────────────────────────────────────────────────
CALIB_FILE = "calibration.npz"
MARKER_SIZE_MM = 100.0  # Size of the ArUco marker in mm. CHANGE THIS TO MATCH YOUR MARKER!
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  # CHANGE THIS TO MATCH YOUR MARKER DICT!
DEFAULT_WIDTH, DEFAULT_HEIGHT = 1280, 720
FRAMERATE = 15  # Lower this if you see lag buildup (try 10 or 15)
CSV_FILE = "aruco_distance_results.csv"
# ────────────────────────────────────────────────────────────────────────────

# Marker corner object points in marker-local space (used by solvePnP)
_half = MARKER_SIZE_MM / 2
MARKER_OBJ_POINTS = np.array([
    [-_half,  _half, 0],
    [ _half,  _half, 0],
    [ _half, -_half, 0],
    [-_half, -_half, 0],
], dtype=np.float32)


def load_calibration(filepath):
    if not os.path.exists(filepath):
        print(f"Error: Calibration file '{filepath}' not found.")
        print("Please run '2_calibrate.py' first.")
        sys.exit(1)

    with np.load(filepath) as data:
        mtx = data['K']
        dist = data['dist']

        if 'image_size' in data:
            calib_size = tuple(data['image_size'])  # (width, height)
        else:
            calib_size = None

        print(f"Loaded calibration from {filepath}")
        return mtx, dist, calib_size


def estimate_pose(corners, mtx, dist):
    """Estimate rvec/tvec for each detected marker using solvePnP (works on all OpenCV versions)."""
    rvecs, tvecs = [], []
    for c in corners:
        _, rvec, tvec = cv2.solvePnP(
            MARKER_OBJ_POINTS, c[0], mtx, dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        rvecs.append(rvec)
        tvecs.append(tvec)
    return rvecs, tvecs


def append_csv_row(estimate_mm, real_mm, csv_path=CSV_FILE):
    """Append a measurement row to CSV, creating the file with a header if missing."""
    os.makedirs(os.path.dirname(csv_path), exist_ok=True) if os.path.dirname(csv_path) else None

    error_mm = estimate_mm - real_mm
    error_percent = (abs(error_mm) / real_mm * 100) if real_mm else float('inf')

    file_exists = os.path.isfile(csv_path)
    with open(csv_path, mode='a', newline='') as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["estimate_mm", "real_mm", "error_mm", "error_percent"]
        )
        if not file_exists:
            writer.writeheader()
        writer.writerow({
            "estimate_mm": round(estimate_mm, 2),
            "real_mm": round(real_mm, 2),
            "error_mm": round(error_mm, 2),
            "error_percent": round(error_percent, 2)
        })
    return error_mm, error_percent


def main():
    # Load calibration
    mtx, dist, calib_size = load_calibration(CALIB_FILE)
    print(f"Calibration Matrix:\n{mtx}")
    print(f"Distortion Coefficients:\n{dist}")

    # Determine resolution
    if calib_size:
        width, height = calib_size
    else:
        width, height = DEFAULT_WIDTH, DEFAULT_HEIGHT

    # Build ArUco detector — new API (4.7+) with fallback to old API
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    parameters = cv2.aruco.DetectorParameters()
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        use_new_api = True
    except AttributeError:
        use_new_api = False

    # Start video capture (rpicam-vid preferred, cv2.VideoCapture fallback)
    proc = None
    cap = None
    frame_size = None

    try:
        cmd = [
            "rpicam-vid",
            "--width",     str(width),
            "--height",    str(height),
            "--framerate", str(FRAMERATE),
            "--codec",     "yuv420",
            "--timeout",   "0",
            "--nopreview",
            "-o", "-"
        ]
        subprocess.check_call(["which", "rpicam-vid"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"Starting rpicam-vid: {width}x{height}")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        frame_size = width * height * 3 // 2  # YUV420

    except (FileNotFoundError, subprocess.CalledProcessError):
        print("Warning: 'rpicam-vid' not found or failed. Falling back to cv2.VideoCapture(0).")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not cap.isOpened():
            print("Error: Could not open camera.")
            sys.exit(1)

        # Check actual resolution and scale K if it differs
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            if w != width or h != height:
                print(f"Warning: Requested {width}x{height}, but got {w}x{h}. Scaling K.")
                scale_x = w / width
                scale_y = h / height
                mtx[0, 0] *= scale_x
                mtx[1, 1] *= scale_y
                mtx[0, 2] *= scale_x
                mtx[1, 2] *= scale_y
                width, height = w, h

    print("\nControls:")
    print("  'q' - Quit")
    print("  'v' - Verify distance (enter measured distance in console)")
    print(f"Using Marker Size: {MARKER_SIZE_MM} mm")

    rvecs, tvecs, ids = [], [], None  # keep last-known pose for 'v' handler
    frame_budget_ms = 1000.0 / FRAMERATE  # ms per frame at current framerate

    while True:
        t_start = time.monotonic()
        frame = None

        if proc:
            raw = proc.stdout.read(frame_size)
            if len(raw) < frame_size:
                print("Camera stream ended unexpectedly.")
                break
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((height * 3 // 2, width))
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        elif cap:
            ret, frame = cap.read()
            if not ret:
                break

        if frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if use_new_api:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        detected = ids is not None and len(ids) > 0

        if detected:
            rvecs, tvecs = estimate_pose(corners, mtx, dist)  # persisted for 'v' handler

            # Green border around the whole frame when a marker is on screen
            border = 6
            cv2.rectangle(frame, (border, border),
                          (frame.shape[1] - border, frame.shape[0] - border),
                          (0, 255, 0), border)

            for i, marker_id in enumerate(ids):
                pts = corners[i][0].astype(int)  # 4 corner points

                # Corner bracket highlights (more prominent than drawDetectedMarkers)
                bracket_len = max(12, int(np.linalg.norm(pts[1] - pts[0]) * 0.25))
                bracket_color = (0, 255, 0)
                bracket_thickness = 3
                for j, pt in enumerate(pts):
                    next_pt = pts[(j + 1) % 4]
                    prev_pt = pts[(j - 1) % 4]
                    dir_next = (next_pt - pt).astype(float)
                    dir_prev = (prev_pt - pt).astype(float)
                    norm_next = dir_next / (np.linalg.norm(dir_next) + 1e-6)
                    norm_prev = dir_prev / (np.linalg.norm(dir_prev) + 1e-6)
                    p1 = (pt + norm_next * bracket_len).astype(int)
                    p2 = (pt + norm_prev * bracket_len).astype(int)
                    cv2.line(frame, tuple(pt), tuple(p1), bracket_color, bracket_thickness)
                    cv2.line(frame, tuple(pt), tuple(p2), bracket_color, bracket_thickness)

                # Pose axes and distance label
                cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], MARKER_SIZE_MM / 2)
                distance = float(np.linalg.norm(tvecs[i]))
                label_pt = (pts[0][0], pts[0][1] - 10)
                cv2.putText(frame, f"ID{marker_id[0]}  {distance:.1f}mm",
                            label_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            # Status bar — top-left
            status_text = f"DETECTED: {len(ids)} marker{'s' if len(ids) > 1 else ''}"
            cv2.putText(frame, status_text, (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            # No marker visible
            cv2.putText(frame, "SCANNING...", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 100, 255), 2)

        # Lag indicator — red if processing is slower than the frame budget
        elapsed_ms = (time.monotonic() - t_start) * 1000
        lag_color = (0, 60, 255) if elapsed_ms > frame_budget_ms else (180, 180, 180)
        cv2.putText(frame, f"{elapsed_ms:.1f}ms / {frame_budget_ms:.0f}ms budget",
                    (12, frame.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, lag_color, 1)

        cv2.imshow('ArUco Distance Test', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('v'):
            if ids is not None and len(ids) > 0:
                print(f"\n--- Verification ({len(ids)} markers) ---")
                for i, marker_id in enumerate(ids):
                    calc_dist = float(np.linalg.norm(tvecs[i]))
                    print(f"Marker ID: {marker_id[0]}")
                    print(f"  Estimated Distance: {calc_dist:.2f} mm")
                    try:
                        true_dist_str = input(f"  Enter TRUE distance for Marker {marker_id[0]} (mm): ")
                        if not true_dist_str.strip():
                            print("  Skipped.")
                            continue
                        true_dist = float(true_dist_str)
                        error, error_percent = append_csv_row(calc_dist, true_dist, CSV_FILE)
                        print(f"  Error: {error:+.2f} mm ({error_percent:.2f}%)")
                        print(f"  Saved to: {CSV_FILE}")
                    except ValueError:
                        print("  Invalid input. Skipping.")
                print("--------------------\n")
            else:
                print("No markers detected for verification.")

    if proc:
        proc.terminate()
    elif cap:
        cap.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
