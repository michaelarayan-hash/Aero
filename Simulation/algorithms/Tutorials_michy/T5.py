"""
T5.py — run any task with a live camera feed.

Usage:
    python3 T5.py --task T2      # takeoff/land + camera
    python3 T5.py --task T3      # NED square  + camera
    python3 T5.py --task T4      # body square + camera (default)

Press Q to quit the camera window early.
"""

import argparse
import asyncio
import importlib
import sys
import threading
import cv2
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from gz_cam import SimCamera

_done = threading.Event()


def _run_mission(task_module):
    asyncio.run(task_module.run())
    _done.set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", default="T4", choices=["T2", "T3", "T4"],
                        help="Which task to run alongside the camera feed")
    args = parser.parse_args()

    task = importlib.import_module(args.task)

    cam = SimCamera()
    print(f"Waiting for camera on:\n  {cam.topic}")
    if not cam.wait_for_frame(timeout=15):
        print("[ERROR] No frames — is the simulation running?")
        sys.exit(1)

    print(f"Camera ready. Starting {args.task}...")
    threading.Thread(target=_run_mission, args=(task,), daemon=True).start()

    cv2.namedWindow(f"Camera — {args.task}", cv2.WINDOW_NORMAL)
    cv2.resizeWindow(f"Camera — {args.task}", 960, 720)

    while not _done.is_set():
        ok, frame = cam.read()
        if ok:
            cv2.imshow(f"Camera — {args.task}", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
