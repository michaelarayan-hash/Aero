import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os

# ── Settings ────────────────────────────────────────────────────────────────
CALIB_FILE = "calibration.npz"
MARKER_SIZE_MM = 50.0  # Size of the ArUco marker in mm. CHANGE THIS TO MATCH YOUR MARKER!
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50  # CHANGE THIS TO MATCH YOUR MARKER DICT!
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

def main():
    # Load calibration
    mtx, dist, calib_size = load_calibration(CALIB_FILE)
    print(f"Calibration Matrix:\n{mtx}")
    print(f"Distortion Coefficients:\n{dist}")
    
    if calib_size:
        print(f"Calibration Resolution: {calib_size[0]}x{calib_size[1]}")

    # Initialize ArUco dictionary and parameters
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        parameters = cv2.aruco.DetectorParameters()
    except AttributeError:
        print("Error: cv2.aruco attributes not found. Ensure 'opencv-contrib-python' is installed.")
        sys.exit(1)

    # Start video capture
    cap = cv2.VideoCapture(0) # Adjust camera index if needed
    
    # Try to set resolution to match calibration
    if calib_size:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, calib_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calib_size[1])
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        sys.exit(1)

    # Read first frame to check actual resolution
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        sys.exit(1)
        
    h, w = frame.shape[:2]
    print(f"Camera Resolution: {w}x{h}")
    
    # Scale K if resolution differs
    if calib_size and (w != calib_size[0] or h != calib_size[1]):
        print("Warning: Camera resolution does not match calibration resolution.")
        print("Scaling calibration matrix...")
        
        scale_x = w / calib_size[0]
        scale_y = h / calib_size[1]
        
        mtx[0, 0] *= scale_x # fx
        mtx[1, 1] *= scale_y # fy
        mtx[0, 2] *= scale_x # cx
        mtx[1, 2] *= scale_y # cy
        
        print(f"New Camera Matrix:\n{mtx}")

    print("\nControls:")
    print("  'q' - Quit")
    print("  'v' - Verify distance (enter measured distance in console)")
    print(f"Using Marker Size: {MARKER_SIZE_MM} mm")

    while True:
        ret, frame = cap.read()
        if not ret:
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

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
