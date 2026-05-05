# Running the Simulation

After completing [SETUP_UBUNTU.md](SETUP_UBUNTU.md) (native) or [SETUP_WINDOWS.md](SETUP_WINDOWS.md) (Docker), use this guide to run scenarios and algorithm tests.

## Quick Start

```bash
cd ~/Aero/Simulation
source .venv/bin/activate              # or skip if using Docker
python3 sim.py --world forest --gui
```

Open a second terminal for algorithm scripts:

```bash
cd ~/Aero/Simulation && source .venv/bin/activate
python3 algorithms/takeoff_land.py
python3 algorithms/auto_land.py --world tag_demo
```

## Worlds

### Built-in worlds (bundled with PX4)

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

### External worlds (PX4-gazebo-models repo)

Clone once:
```bash
git clone https://github.com/PX4/PX4-gazebo-models.git ~/PX4-gazebo-models
```

Then use any world from that repo:
```bash
PX4_GZ_WORLDS=~/PX4-gazebo-models/worlds python3 sim.py --world baylands --gui
```

### Custom world files

```bash
PX4_GZ_WORLDS=/path/to/my/worlds python3 sim.py --world my_world --gui
```

## Vehicles

| Vehicle | Camera | Description |
|---------|--------|-------------|
| `x500` | none | Basic X500 quadrotor |
| `x500_mono_cam` | forward mono | 1280×960, 30fps forward-facing |
| `x500_mono_cam_down` | downward mono | 1280×960, 30fps downward-facing (vision landing) |
| `x500_depth` | OakD-Lite RGB-D | Depth + RGB camera |

Default is `x500_mono_cam`. Override:

```bash
python3 sim.py --world aruco --vehicle x500_mono_cam_down --gui
```

## Camera Feed

`x500_mono_cam` and `x500_mono_cam_down` publish video over gz-transport. `algorithms/camera_feed.py` exposes a `cv2.VideoCapture`-compatible interface:

```bash
# Standalone viewer (simulation must be running)
python3 algorithms/camera_feed.py --world forest

# Press Q to quit
```

> Camera packages (`gz.transport13`, `gz.msgs10`) are system-level; no venv install needed.

## Algorithm Scripts

### Basic Flight

```bash
python3 algorithms/takeoff_land.py             # takeoff 5m, hover, land
python3 algorithms/takeoff_land.py --altitude 10.0
```

### Precision Auto-Landing

Requires `tag_demo` world + `x500_mono_cam_down` vehicle:

```bash
python3 sim.py --world tag_demo --vehicle x500_mono_cam_down --gui

# In second terminal:
python3 algorithms/auto_land.py --world tag_demo
python3 algorithms/auto_land.py --world tag_demo --altitude 3.0
# Press Q to quit
```

### Telemetry

```bash
python3 connection_test.py              # snapshot of drone state
```

## Manual Launch (advanced)

Use when you need direct control over each component (e.g., restart GUI without restarting server).

> **Order matters:** Always start the server first and wait for `INFO  [init] Gazebo world is ready` before opening the GUI.

Terminal 1:
```bash
cd ~/Aero/Simulation
./launch_sim.sh forest                  # forest world, default vehicle
./launch_sim.sh aruco x500_mono_cam_down       # aruco world + specific vehicle
```

Terminal 2 (after server prints "Gazebo world is ready"):
```bash
gz sim -g
```

Terminal 3:
```bash
~/QGroundControl.AppImage              # or `docker exec aero qgroundcontrol` if using Docker
```

Terminal 4:
```bash
cd ~/Aero/Simulation && source .venv/bin/activate
python3 algorithms/takeoff_land.py
```

## Architecture

```
config.py                — single source of truth (paths, ports, timeouts, vehicle)
sim.py                   — one-command launcher: kill strays → server → QGC → optional GUI
launch_sim.sh            — raw PX4 SITL server launcher
connection_test.py       — connect_drone / read_telemetry (standalone + importable)
algorithms/
  __init__.py            — sys.path import pattern
  takeoff_land.py        — arm → takeoff → hover → land → disarm
  camera_feed.py         — SimCamera: gz-transport subscriber, cv2-compatible
  auto_land.py           — closed-loop precision landing via ArUco pose + velocity control
```

## MAVLink Ports

| Port | Purpose |
|------|---------|
| UDP 14540 | MAVSDK / offboard algorithm scripts |
| UDP 14550 | QGroundControl (auto-detected) |
