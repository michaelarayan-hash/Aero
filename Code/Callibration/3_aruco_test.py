import cv2
import cv2.aruco as aruco
import numpy as np
import subprocess

# ── Settings ────────────────────────────────────────────────────────────────
CALIB_FILE = "calibration.npz"
MARKER_SIZE_MM = 50.0  # Size of the ArUco marker in mm. CHANGE THIS TO MATCH YOUR MARKER!
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  # CHANGE THIS TO MATCH YOUR MARKER DICT!
DEFAULT_WIDTH, DEFAULT_HEIGHT = 1280, 720
# ────────────────────────────────────────────────────────────────────────────

def load_calibration(filepath):
    if not os.path.exists(filepath):
        print(f"Error: Calibration file '{filepath}' not found.")
        print("Please run '2_calibrate.py' first.")
        sys.exit(1)
    
    with np.load(filepath) as data:
        mtx = data['K']
        dist = data['dist']
        
        # Check if image_size is present
        if 'image_size' in data:
            calib_size = tuple(data['image_size']) # (width, height)
        else:
            calib_size = None
            
        print(f"Loaded calibration from {filepath}")
        return mtx, dist, calib_size

def start_camera(width, height):
    """Starts the rpicam-vid process and returns the subprocess object."""
    cmd = [
        "rpicam-vid",
        "--width",       str(width),
        "--height",      str(height),
        "--framerate",   "30",
        "--codec",       "yuv420",
        "--timeout",     "0",
        "--nopreview",
        "-o", "-"
    ]
    
    print(f"Starting camera: {width}x{height}")
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        return proc
    except FileNotFoundError:
        print("Error: 'rpicam-vid' not found. Are you running on a Raspberry Pi?")
        print("Fallback to standard cv2.VideoCapture(0)...")
        return None

def main():
    # Load calibration
    mtx, dist, calib_size = load_calibration(CALIB_FILE)
    print(f"Calibration Matrix:\n{mtx}")
    print(f"Distortion Coefficients:\n{dist}")
    
    # Determine resolution
    if calib_size:
        width, height = calib_size
    else:
        width, height = 1280, 720  # Fallback

    # Start video capture (rpicam-vid)
    proc = None
    cap = None

    try:
        # Try to use rpicam-vid first
        cmd = [
            "rpicam-vid",
            "--width",       str(width),
            "--height",      str(height),
            "--framerate",   "30",
            "--codec",       "yuv420",
            "--timeout",     "0",
            "--nopreview",
            "-o", "-"
        ]
        
        # Check if rpicam-vid exists before running
        subprocess.check_call(["which", "rpicam-vid"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print(f"Starting rpicam-vid: {width}x{height}")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        frame_size = width * height * 3 // 2 # YUV420

    except (FileNotFoundError, subprocess.CalledProcessError):
        print("Warning: 'rpicam-vid' not found or failed. Falling back to cv2.VideoCapture(0).")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        if not cap.isOpened():
            print("Error: Could not open camera.")
            sys.exit(1)
            
        # Check actual resolution
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            if w != width or h != height:
                print(f"Warning: Requested {width}x{height}, but got {w}x{h}.")
                # Scale K if resolution differs
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

    while True:
        frame = None
        
        if proc:
            # Read from rpicam-vid stdout
            raw = proc.stdout.read(frame_size)
            if len(raw) < frame_size:
                print("Camera stream ended unexpectedly.")
                break
            
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((height * 3 // 2, width))
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        elif cap:
            # Read from cv2.VideoCapture
            ret, frame = cap.read()
            if not ret:
                break
        
        if frame is None:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose for each marker
            try:
                # estimatePoseSingleMarkers is the standard way, though deprecated in 4.7+
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_MM, mtx, dist)
            except AttributeError:
                 print("Error: cv2.aruco.estimatePoseSingleMarkers not found.")
                 break

            for i, marker_id in enumerate(ids):
                # Draw axis
                try:
                    cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], MARKER_SIZE_MM / 2)
                except AttributeError:
                    # Fallback for older OpenCV versions
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], MARKER_SIZE_MM / 2)
                
                # Calculate distance
                # tvec is [x, y, z] in camera coordinates
                tvec = tvecs[i][0]
                distance = np.linalg.norm(tvec)
                
                # Display distance
                cv2.putText(frame, f"Dist: {distance:.1f} mm", 
                           (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('ArUco Distance Test', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('v'):
            if ids is not None and len(ids) > 0:
                # Pause and ask for input
                print(f"\n--- Verification ---")
                print(f"Detected {len(ids)} markers.")
                
                for i, marker_id in enumerate(ids):
                    tvec = tvecs[i][0]
                    calc_dist = np.linalg.norm(tvec)
                    print(f"Marker ID: {marker_id[0]}")
                    print(f"  Estimated Distance: {calc_dist:.2f} mm")
                    
                    try:
                        true_dist_str = input(f"  Enter TRUE distance for Marker {marker_id[0]} (mm): ")
                        if not true_dist_str.strip():
                             print("  Skipped.")
                             continue
                        true_dist = float(true_dist_str)
                        error = calc_dist - true_dist
                        error_percent = (abs(error) / true_dist) * 100
                        print(f"  Error: {error:.2f} mm ({error_percent:.2f}%)")
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
