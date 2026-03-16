# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Camera calibration and ArUco marker verification suite targeting Raspberry Pi HQ Camera and standard USB cameras. Three-step pipeline: capture → calibrate → verify.

## Setup

```bash
# Option A: venv
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt

# Option B: Conda
conda env create -f environment.yml && conda activate aero_calib
```

## Workflow Commands

```bash
# Step 1: Capture checkerboard images (SPACE=capture, D=delete last, Q=quit)
python3 Code/Callibration/1_capture_images.py

# Step 2: Process images → calibration.npz
python Code/Callibration/2_calibrate.py

# Step 3: Live ArUco verification (Q=quit, V=manual distance verify)
python Code/Callibration/3_aruco_test.py
```

## Architecture

### Pipeline Data Flow

```
1_capture_images.py  →  calib_images/*.jpg
                                ↓
2_calibrate.py       →  calibration.npz (K matrix, distortion, RMS)
                                ↓
3_aruco_test.py      →  aruco_distance_results.csv (live distance measurements)
```

### Camera Backend Pattern

All three scripts share the same dual-backend pattern:
- **Primary**: `rpicam-vid` subprocess piping raw YUV420 frames (Raspberry Pi HQ Camera)
- **Fallback**: `cv2.VideoCapture(0)` for standard USB cameras

### Key Hardcoded Parameters (must match physical setup)

| File | Parameter | Default | Must match |
|------|-----------|---------|------------|
| `1_capture_images.py` | Resolution | 1280×720, 30fps | Camera capability |
| `2_calibrate.py` | Checkerboard | 10×7 inner corners, 23mm squares | Printed board |
| `3_aruco_test.py` | `MARKER_SIZE_MM` | 100mm | Printed marker |
| `3_aruco_test.py` | Dictionary | `DICT_4X4_50` | Printed marker type |

### Calibration Output (`calibration.npz`)

Contains: camera matrix `K`, distortion coefficients `dist`, RMS reprojection error, `image_size`, `checkerboard` dims. When resolution differs from calibration, `3_aruco_test.py` scales the camera matrix dynamically.

### Verification Logging

`3_aruco_test.py` saves CSV records to `aruco_distance_results.csv` when V is pressed, capturing measured vs. actual distance and error percentage.

---

## Simulation (PX4 + Gazebo Harmonic)

### Setup

```bash
cd Simulation
uv venv .venv && source .venv/bin/activate
uv sync
# or: pip install -r requirements.txt
```

### Running the simulation

#### Quickstart — `sim.py` (recommended)

`sim.py` handles everything in one command: kills any stray PX4/Gazebo/QGC processes, starts the
PX4 SITL server, launches QGroundControl, and optionally opens the Gazebo GUI. Ctrl-C stops
everything cleanly.

```bash
cd ~/Aero/Simulation
source .venv/bin/activate

# Basic — server + QGC, no GUI (headless)
python3 sim.py --world forest

# With Gazebo GUI
python3 sim.py --world forest --gui

# Different world and vehicle
python3 sim.py --world aruco --vehicle x500_depth --gui

# External world directory
PX4_GZ_WORLDS=~/PX4-gazebo-models/worlds python3 sim.py --world baylands --gui
```

Then in a separate terminal run algorithm scripts:

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
python3 algorithms/takeoff_land.py             # takeoff to 5 m, hover, land
python3 algorithms/takeoff_land.py --altitude 10.0
python3 connection_test.py                     # telemetry snapshot only
```

#### Manual launch (advanced)

Use this when you need direct control over each component, e.g. to restart just the GUI without
killing the server.

> **Order matters.** Always start the server first and wait for the line
> `INFO  [init] Gazebo world is ready` before opening `gz sim -g`. If `gz sim -g` is opened first
> it starts its own Gazebo server (default world), and PX4 will attach to that instead of your
> chosen world.

```bash
# Terminal 1 — PX4 SITL server (blocking, wait for "Gazebo world is ready")
cd ~/Aero/Simulation
./launch_sim.sh forest                         # forest world, default vehicle
./launch_sim.sh aruco x500_depth               # aruco world + depth camera
PX4_GZ_WORLDS=~/PX4-gazebo-models/worlds \
    ./launch_sim.sh baylands                   # external world

# Terminal 2 — Gazebo GUI (only after server prints "Gazebo world is ready")
gz sim -g

# Terminal 3 — QGroundControl
~/QGroundControl.AppImage

# Terminal 4 — algorithm scripts
cd ~/Aero/Simulation && source .venv/bin/activate
python3 algorithms/takeoff_land.py
```

### Worlds

#### Built-in worlds (bundled with PX4)

Located in `~/PX4-Autopilot/Tools/simulation/gz/worlds/`. Pass the name (no `.sdf`) as the first argument to `launch_sim.sh`.

| World | Description |
|-------|-------------|
| `default` | Empty grey plane |
| `aruco` | Contains ArUco markers (useful for vision testing) |
| `forest` | Tree-covered terrain |
| `baylands` | San Francisco Bay area terrain |
| `lawn` | Grass lawn |
| `windy` | Empty world with wind enabled |
| `walls` | World with obstacle walls |
| `moving_platform` | Moving takeoff/landing platform |
| `underwater` | Underwater environment |

#### External worlds (PX4-gazebo-models repo)

The [PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models) repository contains additional community worlds. Clone it once, then point `PX4_GZ_WORLDS` at its `worlds/` directory:

```bash
git clone https://github.com/PX4/PX4-gazebo-models.git ~/PX4-gazebo-models

# Use any world from that repo
PX4_GZ_WORLDS=~/PX4-gazebo-models/worlds ./launch_sim.sh <worldname>
```

#### Arbitrary / custom world files

`PX4_GZ_WORLDS` accepts any directory of `.sdf` files — including your own:

```bash
PX4_GZ_WORLDS=/path/to/my/worlds ./launch_sim.sh my_world
```

> **Note:** `PX4_GZ_WORLDS` sets the *directory*; `PX4_GZ_WORLD` (or the first CLI arg) is the
> *filename without `.sdf`*. There is no single-file path variable — the directory + name split
> is how PX4 resolves worlds internally.

### Vehicle Variants

Default is `x500_mono_cam` (forward-facing monocular camera, 1280×960, 30fps).

| Vehicle | Camera | Make target |
|---------|--------|-------------|
| `x500` | none | `gz_x500` |
| `x500_mono_cam` | forward mono | `gz_x500_mono_cam` |
| `x500_mono_cam_down` | downward mono | `gz_x500_mono_cam_down` |
| `x500_depth` | OakD-Lite depth+RGB | `gz_x500_depth` |

Override vehicle: `./launch_sim.sh default x500_depth`

### Architecture

```
config.py                — single source of truth (paths, ports, timeouts, vehicle)
sim.py                   — one-command launcher: kill strays → server → QGC → optional GUI
launch_sim.sh            — raw PX4 SITL server launcher (called by sim.py)
connection_test.py       — connect_drone / read_telemetry (standalone + importable)
algorithms/
  __init__.py            — documents sys.path import pattern
  takeoff_land.py        — arm → takeoff → hover → land → disarm
```

### MAVLink Ports

| Port | Purpose |
|------|---------|
| UDP 14540 | MAVSDK / offboard algorithm scripts |
| UDP 14550 | QGroundControl (auto-detected) |

### Adding Algorithm Scripts

```python
# algorithms/my_script.py
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import config
from connection_test import connect_drone_ctx, read_telemetry
```
