#!/usr/bin/env python3
"""
Generate PNG marker images for the tag_demo Gazebo world.

Outputs:
  Simulation/models/aruco_4x4_id{0,1,2}/marker.png
  Simulation/models/apriltag_36h11_id{0,1,2}/marker.png

Run from Simulation/:
    python3 tools/generate_markers.py
"""
from pathlib import Path
import cv2
import numpy as np

MODELS_DIR = Path(__file__).parent.parent / "models"
SIZE_PX = 500  # fill the full PNG; no extra border needed


def write_marker(img_gray: np.ndarray, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(str(out_path), bgr)
    print(f"  wrote {out_path}")


def main() -> None:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    april_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    print("ArUco 4x4_50:")
    for mid in range(3):
        img = cv2.aruco.generateImageMarker(aruco_dict, mid, SIZE_PX)
        write_marker(img, MODELS_DIR / f"aruco_4x4_id{mid}" / "marker.png")

    print("AprilTag 36h11:")
    for mid in range(3):
        img = cv2.aruco.generateImageMarker(april_dict, mid, SIZE_PX)
        write_marker(img, MODELS_DIR / f"apriltag_36h11_id{mid}" / "marker.png")

    print("Done.")


if __name__ == "__main__":
    main()
