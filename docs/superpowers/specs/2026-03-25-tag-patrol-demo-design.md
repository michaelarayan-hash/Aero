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
    tag_demo.sdf         ← custom Gazebo world with mixed markers on ground
  models/
    markers/             ← PNG marker images referenced by tag_demo.sdf
```

### Data flow

```
SimCamera (background thread)
    └─ frames ──► TagDetector.detect(frame) ──► List[Detection]
                                                      └─► TagDetector.draw() ──► cv2.imshow()

MAVSDK patrol (asyncio, background thread)
    └─ arm → takeoff → offboard NED waypoint loop → land → disarm
```

### Threading model

`asyncio` cannot share the main thread with a blocking OpenCV loop. The split is:

- **Main thread**: `display_loop()` — blocking `cv2.imshow` / `cv2.waitKey` loop (OpenCV requires main thread on most platforms).
- **Background thread**: `asyncio.run(patrol_loop(...))` — the entire MAVSDK event loop runs in a `threading.Thread`.
- **Shared state**: A `threading.Event` called `shutdown_event` (set by display loop on `Q`; checked by patrol loop to trigger landing). Latest detections stored in a `threading.Lock`-protected list updated by the display loop and read only for overlay.

---

## Module: `tag_detector.py`

### `Detection` dataclass

```python
@dataclass
class Detection:
    type: str           # "aruco" or "apriltag"
    family: str         # e.g. "4x4_50" or "tag36h11"
    marker_id: int
    distance_mm: float  # L2 norm of tvec (Euclidean distance camera → marker centre)
    corners: np.ndarray # shape (1, 4, 2), float32, order TL/TR/BR/BL; normalised for both backends
    rvec: np.ndarray    # (3,1) rotation vector
    tvec: np.ndarray    # (3,1) translation vector in mm
    colour: tuple       # BGR, unique per family
```

**Distance calculation:** `distance_mm = float(np.linalg.norm(tvec))` — L2 norm of the translation vector, consistent with the existing `camera_feed.py` and `aruco_detect.py` implementations.

**Corners normalisation:** `cv2.aruco` returns `(1, 4, 2)` directly. `pupil_apriltags` returns `(4, 2)` float64 — the backend reshapes with `corners.reshape(1, 4, 2).astype(np.float32)` before constructing the `Detection`.

### `TagDetector` class

**Constructor parameters:**
- `marker_size_mm: float` — physical marker side length in mm (500mm to match world models; see World section)
- `camera_matrix: np.ndarray`
- `dist_coeffs: np.ndarray`
- `aruco_dicts: list[str]` — dict names from the REGISTRY in `aruco_detect.py`, e.g. `["4x4_50"]`
- `apriltag_families: list[str]` — e.g. `["tag36h11"]`

`pupil_apriltags.Detector` takes `tag_size` in **metres** at detection time. The constructor converts internally: `tag_size_m = marker_size_mm / 1000.0` (500mm → 0.5m). One `Detector` instance is created per family at construction time; `tag_size_m` is passed to each `detector.detect(..., tag_size=tag_size_m)` call.

**Methods:**
- `detect(frame: np.ndarray) -> list[Detection]` — runs both detectors, returns unified list
- `draw(frame: np.ndarray, detections: list[Detection]) -> None` — annotates frame in-place

**Drawing style** (matches existing codebase):
- Corner brackets per tag
- 3D axis overlay (`cv2.drawFrameAxes`)
- Text label: `[TYPE/family] ID{n}  {dist:.0f}mm`
- Green border flash on frame when any tag detected
- Colour-coded: ArUco families in green palette, AprilTag families in cyan palette

### Camera intrinsics

`TagDetector` is constructed with intrinsics loaded via the same `load_calibration()` helper already in `camera_feed.py`:

```python
mtx, dist = load_calibration("calibration.npz")
# Falls back to SIM_CAMERA_MATRIX (derived from x500_mono_cam_down HFOV) if file missing
```

For the downward camera (`x500_mono_cam_down`, 1280×960, HFOV ~1.74 rad), the synthetic fallback values are:
```
fx = fy = 1280 / (2 * tan(1.74/2)) ≈ 664
cx = 640, cy = 480
dist = zeros
```

---

## Script: `tag_patrol.py`

### Structure

```python
# Main thread: display
def display_loop(cam, detector, shutdown_event):
    while not shutdown_event.is_set():
        ok, frame = cam.read()
        detections = detector.detect(frame)
        detector.draw(frame, detections)
        # overlay: waypoint index, altitude, detection count
        cv2.imshow(...)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            shutdown_event.set()
    cv2.destroyAllWindows()

# Background thread: asyncio + MAVSDK
async def patrol_loop(drone, waypoints, shutdown_event):
    await arm_and_takeoff(drone, altitude_m)
    for i, (n, e, d) in enumerate(waypoints):
        if shutdown_event.is_set():
            break
        await fly_to_ned(drone, n, e, d)   # offboard NED setpoint
        await asyncio.sleep(2)             # brief hover at each waypoint
    await drone.action.land()
    await drone.action.disarm()

def main():
    shutdown_event = threading.Event()
    cam = SimCamera(world=args.world, vehicle="x500_mono_cam_down")
    cam.start()
    cam.wait_for_frame()
    t = threading.Thread(target=lambda: asyncio.run(run_patrol(shutdown_event)), daemon=True)
    t.start()
    display_loop(cam, detector, shutdown_event)
    t.join(timeout=30)
```

### Flight: offboard NED, not goto_location()

`goto_location()` requires GPS coordinates. This demo uses **MAVSDK offboard mode** with `drone.offboard.set_position_ned()` which accepts local NED directly.

Full sequence:
1. `drone.action.set_takeoff_altitude(altitude_m)` — set to match `--altitude` (default 3.0)
2. `drone.action.arm()`
3. `drone.action.takeoff()` → wait until airborne
4. `drone.offboard.set_position_ned(PositionNedYaw(0, 0, -altitude_m, 0))` — hold current altitude
5. `drone.offboard.start()`
6. Waypoint loop via `fly_to_ned()`
7. `drone.offboard.stop()` → `drone.action.land()` → `drone.action.disarm()`

`set_takeoff_altitude` is set to the same value as `--altitude` so the drone reaches patrol altitude before offboard begins. The first offboard setpoint holds `z = -altitude_m`, matching the post-takeoff NED position, ensuring no altitude jump at offboard start.

A `fly_to_ned()` helper streams the target `PositionNedYaw` setpoint at ~10 Hz until the drone is within 0.5m acceptance radius, then returns.

### Waypoints

Local NED offsets from home (metres). A square at 3m AGL covering the 2×3 marker grid (~10m × 5m):

```python
WAYPOINTS = [   # (north, east, down)
    ( 7,  6, -3),
    ( 7, -6, -3),
    (-7, -6, -3),
    (-7,  6, -3),
]
```

Configurable via `--altitude` (metres AGL) and `--spacing` (metres) CLI args. `down = -altitude_m` for all waypoints.

### Altitude note

Offboard NED `down` is relative to the EKF origin (spawn point). `tag_demo.sdf` places the drone spawn at ground level (z=0), so `down = -3` is 3m AGL.

### SimCamera vehicle — explicit, not config default

`tag_patrol.py` constructs `SimCamera` with an explicit vehicle argument:

```python
cam = SimCamera(world=args.world, vehicle="x500_mono_cam_down")
```

This avoids breakage if `config.VEHICLE` is changed for another experiment.

### CLI

```bash
python3 algorithms/tag_patrol.py --world tag_demo --altitude 3.0 --spacing 10.0
```

### Shutdown

`Q` in the OpenCV window sets `shutdown_event` (a `threading.Event`). The patrol loop polls `shutdown_event.is_set()` between waypoints and in the setpoint stream loop. When set, it exits offboard mode and calls `land()` + `disarm()`.

---

## World: `tag_demo.sdf`

### Layout

A flat ground plane world with a directional sun light. Six markers in a 2×3 grid, 5m spacing, flush with the ground surface:

```
Top-down (x=north, y=east):
  ( 5,  5)  ArUco 4x4 ID0     ( 5,  0)  AprilTag 36h11 ID0    ( 5, -5)  ArUco 4x4 ID1
  (-5,  5)  AprilTag 36h11 ID1 (-5,  0)  ArUco 4x4 ID2         (-5, -5)  AprilTag 36h11 ID2
```

### Marker models

Each marker is an inline SDF `<model>` — a thin box `0.5m × 0.5m × 0.01m` with a PNG diffuse texture on the top face. PNG images stored in `Simulation/models/markers/`.

**Marker size / texture note:** The PNG images must fill the full 0.5m face with no white border padding, so that `marker_size_mm=500` in the detector exactly matches the textured area. ArUco and AprilTag generator tools should be run with `--border 0` or equivalent. If a border is unavoidable, the actual inner marker dimension must be measured and used as `marker_size_mm` instead.

### Lighting

`tag_demo.sdf` includes a `<light type="directional">` (sun) pointing downward at ~45°, sufficient for OpenCV texture detection. Without this, Gazebo renders textures as unlit black and detection fails.

### Running

```bash
# Terminal 1 — simulation
PX4_GZ_WORLDS=~/Aero/Simulation/worlds python3 sim.py --world tag_demo --gui

# Terminal 2 — patrol demo
cd ~/Aero/Simulation && source .venv/bin/activate
python3 algorithms/tag_patrol.py --world tag_demo
```

`config.py` `WORLD` default is `"default"` — the `--world` CLI arg overrides it by passing directly to `SimCamera` and the MAVLink connection; `config.py` does not need to be modified.

---

## Dependencies

- `pupil-apriltags` — add to `Simulation/requirements.txt` and `pyproject.toml`/`uv` config
- `cv2.aruco` — already present (opencv-contrib-python)
- `mavsdk` — already present
- `gz.transport13` / `gz.msgs10` — system packages, already used by `camera_feed.py`

---

## Out of Scope

- Logging detections to CSV
- Multiple vehicles
- Dynamic/moving markers
- Real-camera (Raspberry Pi) integration
