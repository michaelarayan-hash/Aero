# Working in Containers

Day-to-day workflow once the images are built. See [SETUP_DOCKER.md](SETUP_DOCKER.md) for first-time setup.

---

## 1. Start the containers

From the repo root:

```bash
docker compose up -d
```

Both `sim` and `ai` containers start in the background sharing the same ROS2 network.

---

## 2. Open VS Code windows

Open two separate VS Code windows — one for each container:

| Window | Open folder |
|---|---|
| Simulation | `Aero/Simulation` |
| AI | `Aero/AI` |

In each window, VS Code will show a prompt in the bottom-left corner:

> **Reopen in Container**

Click it (or use Command Palette → `Dev Containers: Reopen in Container`). Requires the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Each window is now attached to its respective container at `/workspace/Simulation` or `/workspace/AI`.

---

## 3. Sim container — run the simulation

Open a terminal in the Sim VS Code window and start the simulation:

```bash
python3 sim.py --world aruco --gui
```

This starts PX4 SITL, Gazebo, and the ROS2 bridge. Wait for:

```
[OK] Simulation is ready.
```

Open a **second terminal** in the same window for flight scripts (e.g. MAVSDK missions):

```bash
source .venv/bin/activate
python3 algorithms/square_mission.py
```

---

## 4. AI container — build and run nodes

Open a terminal in the AI VS Code window.

Build only the packages you need:

```bash
cd /workspace/AI
colcon build --packages-select camera_feed --symlink-install
source install/setup.bash
```

> `--symlink-install` means Python edits are live immediately without rebuilding.  
> `source install/setup.bash` must be run in every new terminal after a build.

Then run the node:

```bash
ros2 run camera_feed camera_node --ros-args -p display:=true
```

The node reads `/workspace/tmp/sim_state.json` written by `sim.py` and automatically subscribes to the correct world and vehicle topic — no manual parameters needed.

---

## Tip — verify topics are shared

In any terminal in either container:

```bash
ros2 topic list
```

Topics from the sim container (e.g. camera, telemetry) should be visible in the AI container and vice versa.
