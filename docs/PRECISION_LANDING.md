# Precision Landing

End-to-end guide for running the automated ArUco precision landing system. The drone arms, takes off, locates a ground marker using its downward camera, centers over it, and lands autonomously.

---

## Overview

```
Sim container                                  AI container
─────────────────────────────────────          ──────────────────────────────────────────────
PX4 SITL  ←─── MAVLink ───→  mavsdk_server ←──── landing_node (MAVSDK gRPC client)
Gazebo    ──── camera topic ──────────────────── aruco_node  →  /aruco/pose  →  landing_node
```

Two nodes work in sequence:

- **`aruco_node`** — reads the downward camera, detects the ArUco marker, publishes its XYZ offset on `/aruco/pose`
- **`landing_node`** — reads `/aruco/pose` and sends velocity commands to PX4 via MAVSDK to align and descend

Both nodes run together from a single launch file. The `landing_node` drives the full flight — `aruco_node` only provides perception.

---

## Prerequisites

1. Both Docker containers running:
   ```bash
   docker compose up -d
   ```

2. Simulation running in the sim container with the `landing` world:
   ```bash
   python3 sim.py --world landing --gui
   ```
   Wait for `[OK] Simulation is ready.` before proceeding.

3. Start `mavsdk_server` manually in the sim container. The AI container connects to it over the Docker bridge network using the hostname `sim`:
   ```bash
   /home/dev/.local/lib/python3.12/site-packages/mavsdk/bin/mavsdk_server -p 50051 udpin://0.0.0.0:14540
   ```
   Leave this running in a dedicated terminal — `landing_node` will not connect without it.

4. In the AI container, build and source the workspace (first time or after code changes to `setup.py`):
   ```bash
   cd /workspace/AI
   colcon build --packages-select camera_feed --symlink-install
   source install/setup.bash
   ```
   With `--symlink-install`, Python source edits (including `landing_node.py` and `aruco_node.py`) are live immediately — no rebuild needed.

---

## Running

From a terminal in the AI container:

```bash
ros2 launch camera_feed precision_landing.launch.py display:=true
```

`display:=true` opens an OpenCV window showing the live camera feed with the detected marker highlighted. Omit it (or set `false`) for headless operation.

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `altitude` | `3.0` | Takeoff altitude in metres |
| `display` | `false` | Show ArUco detection window |
| `mavsdk_server_host` | `sim` | Hostname of `mavsdk_server` |

Example with custom altitude:
```bash
ros2 launch camera_feed precision_landing.launch.py altitude:=5.0 display:=true
```

---

## What happens

1. **Connect** — `landing_node` connects to `mavsdk_server` in the sim container and waits for a GPS fix
2. **Arm** — arms the drone and waits for telemetry confirmation
3. **Take off** — climbs to `altitude` metres and waits until 80% of the target altitude is reached
4. **Offset** — enters offboard mode and flies to a small offset from the spawn point, giving the control loop a non-trivial alignment task
5. **Control loop** (10 Hz):
   - If no fresh pose from `aruco_node`: hover in place (`SEARCH`)
   - If aligned within 15 cm: descend at 0.3 m/s (`DESCEND`)
   - Otherwise: fly laterally toward the marker (`ALIGN`)
6. **Land** — when altitude drops below 0.4 m AGL, PX4's built-in land command is triggered
7. **Disarm** — waits for touchdown confirmation, then disarms

---

## Monitoring

In a separate terminal in either container:

```bash
# Confirm both nodes are running
ros2 node list

# Confirm detections are flowing
ros2 topic hz /aruco/pose

# Watch state transitions from the landing node
ros2 topic echo /rosout | grep landing_node
```

The `landing_node` logs each state transition (SEARCH → ALIGN → DESCEND → LAND) with the current xy error and altitude.

---

## Resetting between runs

If the previous run ended without cleanly landing and disarming (e.g. crashed mid-flight), PX4 will deny the next `arm()` call with `COMMAND_DENIED`. Restart the simulation fully:

```bash
# In the sim container
python3 sim.py --world landing --gui
```

This resets PX4 and respawns the drone on the ground. Then re-run the launch file in the AI container.

---

## Tuning

All control parameters live at the top of `AI/src/camera_feed/camera_feed/landing_node.py`. The most commonly adjusted ones:

| Constant | Default | Effect |
|---|---|---|
| `KP_XY` | `0.5` | Lateral aggressiveness — higher = faster alignment, more overshoot |
| `XY_ALIGN_THRESH_M` | `0.15` | How close the drone must be before descending |
| `DESCENT_SPEED_MS` | `0.3` | Descent rate in m/s |
| `TAKEOFF_ALT_M` | `3.0` | Default altitude if not set via launch argument |

Because `--symlink-install` was used, edits to `landing_node.py` take effect on the next launch without rebuilding.

---

## Related docs

- [AI/docs/ARUCO_NODE.md](../AI/docs/ARUCO_NODE.md) — ArUco detection node internals
- [AI/docs/LANDING_NODE.md](../AI/docs/LANDING_NODE.md) — landing node internals, state machine, threading model
- [docs/WORKING_IN_CONTAINERS.md](WORKING_IN_CONTAINERS.md) — container workflow
