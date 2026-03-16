"""
Step 2: Run camera calibration
Raspberry Pi HQ Camera + 6mm CS-mount lens

Usage:
    python 2_calibrate.py

Outputs:
    calibration.npz  — camera matrix + distortion coefficients
    undistorted/     — undistorted sample images for visual check
"""

import cv2
import numpy as np
import glob
import os

# ── Settings ────────────────────────────────────────────────────────────────
IMAGE_DIR     = "calib_images"
OUTPUT_FILE   = "calibration.npz"
PREVIEW_DIR   = "undistorted"

# IMPORTANT: count the INNER corners of your checkerboard (not the squares)
# e.g. a 10x7 square board has 9x6 inner corners
CHECKERBOARD  = (10, 7)

# Real-world square size — measure your printed checkerboard with a ruler
# This only affects translation output, not the camera matrix itself
SQUARE_SIZE_MM = 23.0
# ────────────────────────────────────────────────────────────────────────────

os.makedirs(PREVIEW_DIR, exist_ok=True)

# Prepare object points: (0,0,0), (1,0,0), ... scaled to mm
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

obj_points = []   # 3D world points
img_points = []   # 2D image points
image_size = None

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

images = sorted(glob.glob(os.path.join(IMAGE_DIR, "*.jpg")))
print(f"\n🔍 Found {len(images)} images in '{IMAGE_DIR}/'\n")

good, bad = 0, 0

for fpath in images:
    img  = cv2.imread(fpath)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if image_size is None:
        image_size = gray.shape[::-1]  # (width, height)

    ret, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret:
        # Refine to sub-pixel accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners2)
        good += 1
        print(f"  ✓  {os.path.basename(fpath)}")
    else:
        bad += 1
        print(f"  ✗  {os.path.basename(fpath)}  ← corners not found (check board visibility)")

print(f"\n  {good} usable / {bad} rejected\n")

if good < 10:
    print("⚠️  Need at least 10 good images for reliable calibration. Capture more.")
    exit(1)

# ── Run calibration ──────────────────────────────────────────────────────────
print("⚙️  Running calibration...")
rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, image_size, None, None
)

print(f"\n{'='*50}")
print(f"  Reprojection error (RMS): {rms:.4f} px")
print(f"  {'✅ Excellent' if rms < 0.5 else '✅ Good' if rms < 1.0 else '⚠️  High — retake some images'}")
print(f"{'='*50}")
print(f"\n  Camera matrix (K):\n{K}")
print(f"\n  Distortion coefficients:\n{dist.ravel()}")
print(f"  (k1, k2, p1, p2, k3)")

fx, fy = K[0, 0], K[1, 1]
cx, cy = K[0, 2], K[1, 2]
print(f"\n  Focal length:  fx={fx:.1f}px  fy={fy:.1f}px")
print(f"  Principal pt:  cx={cx:.1f}  cy={cy:.1f}")

# ── Save calibration ─────────────────────────────────────────────────────────
np.savez(OUTPUT_FILE, K=K, dist=dist, rms=rms,
         image_size=image_size, checkerboard=CHECKERBOARD)
print(f"\n💾 Saved to '{OUTPUT_FILE}'")

# ── Visual check: undistort sample images ────────────────────────────────────
print(f"\n🖼  Saving undistorted samples to '{PREVIEW_DIR}/'...")

for fpath in images[:5]:
    img = cv2.imread(fpath)
    h, w = img.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(img, K, dist, None, new_K)

    # Crop to valid ROI
    x, y, rw, rh = roi
    if rw > 0 and rh > 0:
        undistorted = undistorted[y:y+rh, x:x+rw]

    out_path = os.path.join(PREVIEW_DIR, os.path.basename(fpath))
    cv2.imwrite(out_path, undistorted)

print(f"   Done. Compare originals vs undistorted/ to verify.")

# ── Usage snippet ─────────────────────────────────────────────────────────────
print(f"""
─────────────────────────────────────────────
To undistort frames in your own code:

    import cv2, numpy as np
    data  = np.load("calibration.npz")
    K     = data["K"]
    dist  = data["dist"]

    h, w  = frame.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w,h), 1, (w,h))
    frame = cv2.undistort(frame, K, dist, None, new_K)
─────────────────────────────────────────────
""")
