#!/usr/bin/env python3
"""
Write model.config and model.sdf for each marker model directory.
Run AFTER generate_markers.py (PNG files must already exist).

Run from Simulation/:
    python3 tools/create_models.py
"""
from pathlib import Path

MODELS_DIR = Path(__file__).parent.parent / "models"

CONFIG_TEMPLATE = """\
<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
"""

SDF_TEMPLATE = """\
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>0 0 0.001 0 0 0</pose>
    <link name="base">
      <visual name="base_visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>0.5 0.5</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.4 0.4 0.4 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://{name}/marker.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MARKERS = [
    ("aruco_4x4_id0",        "ArUco 4x4_50 marker ID 0"),
    ("aruco_4x4_id1",        "ArUco 4x4_50 marker ID 1"),
    ("aruco_4x4_id2",        "ArUco 4x4_50 marker ID 2"),
    ("apriltag_36h11_id0",   "AprilTag 36h11 marker ID 0"),
    ("apriltag_36h11_id1",   "AprilTag 36h11 marker ID 1"),
    ("apriltag_36h11_id2",   "AprilTag 36h11 marker ID 2"),
]


def main() -> None:
    for name, desc in MARKERS:
        d = MODELS_DIR / name
        d.mkdir(parents=True, exist_ok=True)
        (d / "model.config").write_text(CONFIG_TEMPLATE.format(name=name, description=desc))
        (d / "model.sdf").write_text(SDF_TEMPLATE.format(name=name))
        print(f"  {name}/model.config + model.sdf")
    print("Done.")


if __name__ == "__main__":
    main()
