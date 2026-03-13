"""
Step 1: Capture calibration images
Raspberry Pi HQ Camera + 6mm CS-mount lens

Usage:
    python 1_capture_images.py

Controls:
    SPACE  - capture image
    D      - delete last image
    Q      - quit
"""

import cv2
import os
import time
from picamera2 import Picamera2

# ── Settings ────────────────────────────────────────────────────────────────
SAVE_DIR       = "calib_images"
CAPTURE_WIDTH  = 2028   # Half-res mode, fast enough for live preview
CAPTURE_HEIGHT = 1520   #   (full res 4056x3040 also works, just slower)
TARGET_IMAGES  = 25     # Aim for at least 20 good ones
# ────────────────────────────────────────────────────────────────────────────

os.makedirs(SAVE_DIR, exist_ok=True)

picam2 = Picamera2()
config = picam2.create_still_configuration(
    main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"},
    lores={"size": (640, 480), "format": "YUV420"},   # fast preview stream
    display="lores"
)
picam2.configure(config)
picam2.start()
time.sleep(1)  # warm-up

count = len(os.listdir(SAVE_DIR))
print(f"\n📷 HQ Camera ready — {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}")
print(f"   Images already captured: {count}")
print(f"   Target: {TARGET_IMAGES} images\n")
print("   SPACE = capture | D = delete last | Q = quit\n")

last_saved = None

while True:
    # Grab low-res preview for display
    preview = picam2.capture_array("lores")
    preview_bgr = cv2.cvtColor(preview, cv2.COLOR_YUV420p2BGR)

    # Overlay status
    status = f"Captured: {count}/{TARGET_IMAGES}"
    color  = (0, 255, 0) if count >= TARGET_IMAGES else (0, 200, 255)
    cv2.putText(preview_bgr, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(preview_bgr, "SPACE=capture  D=delete  Q=quit",
                (10, preview_bgr.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    cv2.imshow("Calibration Capture — HQ Camera", preview_bgr)
    key = cv2.waitKey(1) & 0xFF

    if key == ord(' '):
        # Capture full-res still
        frame = picam2.capture_array("main")
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        path = os.path.join(SAVE_DIR, f"calib_{count:03d}.jpg")
        cv2.imwrite(path, frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
        last_saved = path
        count += 1
        print(f"  ✓ Saved {path}")

        # Flash feedback
        flash = preview_bgr.copy()
        flash[:] = (255, 255, 255)
        cv2.imshow("Calibration Capture — HQ Camera", flash)
        cv2.waitKey(100)

    elif key == ord('d') and last_saved and os.path.exists(last_saved):
        os.remove(last_saved)
        count -= 1
        print(f"  ✗ Deleted {last_saved}")
        last_saved = None

    elif key == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()

print(f"\nDone. {count} images saved to '{SAVE_DIR}/'")
print("Next step: run  python 2_calibrate.py")
