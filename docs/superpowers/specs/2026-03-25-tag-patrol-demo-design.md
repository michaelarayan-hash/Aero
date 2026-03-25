# Tag Patrol Demo — Design Spec
**Date:** 2026-03-25

## Overview

A simulation demo where a drone autonomously flies a waypoint circuit over a custom Gazebo world containing both ArUco and AprilTag markers laid flat on the ground. A downward-facing camera feed is displayed in real time with each detected tag annotated: type, family, ID, and distance in millimetres.

---

## Goals

- Drone takes off, flies a scripted waypoint loop, lands — fully autonomous.
- Simultaneously detect ArUco (cv2.aruco) and AprilTag (pupil_apriltags) markers in the downward camera feed.
- Annotate each detection on screen: `[TYPE/family] ID{n}  {dist:.0f}mm`, colour-coded by type.
- Show a status overlay: current waypoint index, total detections accumulated, drone altitude.
- Single command to launch; `Q` in the OpenCV window shuts everything down cleanly.

---

## Architecture

### New files

```
Simulation/
  algorithms/
    tag_detector.py      ← unified ArUco + AprilTag detector module
    tag_patrol.py        ← main demo script (flight + camera display)
  worlds/
    tag_demo.sdf         ← custom Gazebo world with mixed markers
  models/
    markers/             ← PNG marker images referenced by tag_demo.sdf
```

### Data flow

```
SimCamera (background thread)
    └─ frames ──► TagDetector.detect(frame) ──► List[Detection]
                                                      └─► TagDetector.draw() ──► cv2.imshow()

MAVSDK patrol (asyncio task)
    └─ arm → takeoff → waypoint[0..N] loop → land → disarm
```

- MAVSDK patrol runs as an `asyncio.Task`.
- Camera display runs on the main thread (OpenCV requirement).
- A thread-safe `detections` buffer shares the latest detection list between the two.

---

## Module: `tag_detector.py`

### `Detection` dataclass

```python
@dataclass
class Detection:
    type: str           # "aruco" or "apriltag"
    family: str         # e.g. "4x4_50" or "tag36h11"
    marker_id: int
    distance_mm: float
    corners: np.ndarray # shape (1, 4, 2), float32
    rvec: np.ndarray
    tvec: np.ndarray
    colour: tuple       # BGR, unique per family
```

### `TagDetector` class

**Constructor parameters:**
- `marker_size_mm: float` — physical marker side length (500mm to match world models)
- `camera_matrix: np.ndarray`
- `dist_coeffs: np.ndarray`
- `aruco_dicts: list[str]` — e.g. `["4x4_50"]`, names from the REGISTRY in `aruco_detect.py`
- `apriltag_families: list[str]` — e.g. `["tag36h11"]`

**Methods:**
- `detect(frame: np.ndarray) -> list[Detection]` — runs both detectors, returns unified list
- `draw(frame: np.ndarray, detections: list[Detection]) -> None` — annotates frame in-place

**Drawing style** (matches existing codebase):
- Corner brackets per tag
- 3D axis overlay (`cv2.drawFrameAxes`)
- Text label: `[TYPE/family] ID{n}  {dist:.0f}mm`
- Green border flash on frame when any tag detected
- Colour-coded: ArUco families in green palette, AprilTag families in cyan palette

---

## Script: `tag_patrol.py`

### Structure

```python
async def main():
    cam = SimCamera(world=args.world, vehicle=config.VEHICLE)
    cam.start()
    cam.wait_for_frame()

    async with connect_drone_ctx() as drone:
        asyncio.create_task(patrol_loop(drone, WAYPOINTS))
        display_loop(cam, detector)          # blocks on main thread

async def patrol_loop(drone, waypoints):
    # arm → takeoff → iterate waypoints via goto_location() → land → disarm

def display_loop(cam, detector):
    # cv2 window loop: read frame, detect, draw, imshow, handle Q to quit
```

### Waypoints

Defined as local NED offsets (metres from home position), forming a square that passes over the 6 marker positions:

```python
WAYPOINTS = [
    ( 10,  10, -3),   # NE corner
    ( 10, -10, -3),   # NW corner
    (-10, -10, -3),   # SW corner
    (-10,  10, -3),   # SE corner
]
```

Altitude 3m AGL (NED z = -3). Configurable via `--altitude` and `--spacing` CLI args.

### CLI

```bash
python3 algorithms/tag_patrol.py --world tag_demo --altitude 3.0 --spacing 10.0
```

### Shutdown

`Q` in the OpenCV window sets a `shutdown_event`. The patrol loop checks this event and issues `land` + `disarm` before exiting.

---

## World: `tag_demo.sdf`

A flat ground plane world. Six markers placed in a 2×3 grid, ~5m spacing, flat on the ground:

```
Ground layout (top-down):
  [ArUco 4x4 ID0]        [AprilTag 36h11 ID0]    [ArUco 4x4 ID1]
  [AprilTag 36h11 ID1]   [ArUco 4x4 ID2]         [AprilTag 36h11 ID2]
```

Each marker is a thin SDF box model (0.5m × 0.5m × 0.01m) with a PNG texture on its top face. Marker PNGs stored in `Simulation/models/markers/`.

### Running

```bash
PX4_GZ_WORLDS=~/Aero/Simulation/worlds python3 sim.py --world tag_demo --gui
# then in a second terminal:
python3 algorithms/tag_patrol.py --world tag_demo
```

---

## Dependencies

- `pupil-apriltags` — add to `Simulation/requirements.txt` and `pyproject.toml`/`uv` config
- `cv2.aruco` — already present
- `mavsdk` — already present
- `gz.transport13` / `gz.msgs10` — system packages, already used by `camera_feed.py`

---

## Out of Scope

- Logging detections to CSV (can be added later)
- Multiple vehicles
- Dynamic/moving markers
- Any real-camera (Raspberry Pi) integration
