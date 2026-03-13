"""
Step 1: Capture calibration images
Raspberry Pi HQ Camera + 6mm CS-mount lens (rpicam version)

Usage:
    python3 1_capture_images.py

Controls:
    SPACE  - capture image
    D      - delete last image
    Q      - quit
"""

import cv2
import numpy as np
import subprocess
import os

# ── Settings ────────────────────────────────────────────────────────────────
SAVE_DIR      = "calib_images"
WIDTH, HEIGHT = 1280, 720
TARGET        = 25
# ────────────────────────────────────────────────────────────────────────────

os.makedirs(SAVE_DIR, exist_ok=True)

cmd = [
    "rpicam-vid",
    "--width",       str(WIDTH),
    "--height",      str(HEIGHT),
    "--framerate",   "30",
    "--codec",       "yuv420",
    "--timeout",     "0",
    "--nopreview",
    "-o", "-"
]

proc       = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
frame_size = WIDTH * HEIGHT * 3 // 2   # YUV420

count      = len(os.listdir(SAVE_DIR))
last_saved = None

print(f"\n HQ Camera ready — {WIDTH}x{HEIGHT}")
print(f"   Images already in folder: {count}")
print(f"   Target: {TARGET} images\n")
print("   SPACE = capture | D = delete last | Q = quit\n")

while True:
    raw = proc.stdout.read(frame_size)
    if len(raw) < frame_size:
        print("Camera stream ended unexpectedly.")
        break

    yuv   = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT * 3 // 2, WIDTH))
    frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

    # Overlay status
    status = f"Captured: {count}/{TARGET}"
    color  = (0, 255, 0) if count >= TARGET else (0, 200, 255)
    cv2.putText(frame, status, (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
    cv2.putText(frame, "SPACE=capture  D=delete  Q=quit",
                (10, HEIGHT - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    cv2.imshow("Calibration Capture", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord(' '):
        path = os.path.join(SAVE_DIR, f"calib_{count:03d}.jpg")
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        last_saved = path
        count += 1
        print(f"  ✓ Saved {path}")

        # White flash feedback
        flash = np.ones_like(frame) * 255
        cv2.imshow("Calibration Capture", flash)
        cv2.waitKey(80)

    elif key == ord('d') and last_saved and os.path.exists(last_saved):
        os.remove(last_saved)
        count -= 1
        print(f"  ✗ Deleted {last_saved}")
        last_saved = None

    elif key == ord('q'):
        break

proc.terminate()
cv2.destroyAllWindows()
print(f"\nDone. {count} images saved to '{SAVE_DIR}/'")
print("Next step: run  python 2_calibrate.py")
