# Camera Feed — Setup & Usage

The `camera_feed` package is a ROS 2 node that subscribes to the Gazebo camera topic and makes each frame available to the rest of the ROS 2 graph. It is the entry point for all vision work in the AI container.

---

## Prerequisites

- Both Docker containers running (`docker compose up -d` from repo root)
- Simulation container publishing a camera topic (i.e. `sim.py` is running with a vehicle that has a camera)
- X11 server configured if you want the display window (see [docs/SETUP_DOCKER.md](../../docs/SETUP_DOCKER.md))

---

## Package Structure

```
AI/
├── src/
│   └── camera_feed/
│       ├── camera_feed/
│       │   ├── __init__.py
│       │   └── camera_node.py     # the node
│       ├── package.xml            # ROS 2 manifest + dependencies
│       ├── setup.py               # colcon entry point registration
│       └── setup.cfg              # install path override for ros2 run
├── docker/
│   ├── Dockerfile                 # AI container image
│   └── entrypoint.sh             # sources ROS 2 + workspace on container start
└── pyproject.toml                 # AI/ML Python deps (torch, onnxruntime, numpy)
```

---

## Building the Package

All commands run **inside the AI container**.

```bash
# Open a shell in the AI container
docker compose exec ai bash

# Navigate to the workspace root (already set by devcontainer)
cd /workspace/AI

# Build
colcon build

# Source the result — required after every build
source install/setup.bash
```

`source install/setup.bash` is always a manual step after `colcon build`. The entrypoint only sources the install space if it already existed when the container started.

To build only the camera_feed package (faster during iteration):

```bash
colcon build --packages-select camera_feed
source install/setup.bash
```

---

## Running the Node

### Basic (no display)

```bash
ros2 run camera_feed camera_node
```

The node reads `/workspace/tmp/sim_state.json` (written by `sim.py` on startup) to automatically pick up the correct `world` and `vehicle` — no manual parameters needed. It logs a heartbeat every 30 frames:

```
[INFO] [camera_feed]: Subscribing to: /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
[INFO] [camera_feed]: Frame 1: 1280x960
[INFO] [camera_feed]: Frame 31: 1280x960
```

### With live display window

```bash
ros2 run camera_feed camera_node --ros-args -p display:=true
```

Opens a 960×720 OpenCV window titled **Aero Camera**. Press `q` to close and shut down the node.

### Override world or vehicle

Explicit parameters always take precedence over `sim_state.json`:

```bash
ros2 run camera_feed camera_node --ros-args \
  -p world:=aruco \
  -p vehicle:=x500_mono_cam_down
```

The node builds the topic string as:
```
/world/<world>/model/<vehicle>_0/link/camera_link/sensor/camera/image
```

---

## Automatic parameter discovery

When `sim.py` starts it writes:

```
/workspace/tmp/sim_state.json  →  {"world": "aruco", "vehicle": "x500_mono_cam_down"}
```

`camera_node` reads this file before declaring its ROS2 parameters so the defaults match whatever the sim is running. The `/workspace` volume is shared between both containers, so the file written by the sim container is immediately visible in the AI container.

If the file is missing (sim not started yet), the node falls back to `world=aruco` / `vehicle=x500_mono_cam_down`.

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `world` | string | from `sim_state.json` | Gazebo world name — must match the world loaded in the sim container |
| `vehicle` | string | from `sim_state.json` | Model name — the node appends `_0` (first spawned instance) |
| `display` | bool | `false` | Show a live OpenCV window |

Pass any parameter with `--ros-args -p name:=value`.

---

## Verifying the Feed

### Check the topic exists

```bash
# In either container
ros2 topic list | grep camera
```

Expected output:
```
/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

If the topic is missing, the sim container is not running or the world/vehicle name is wrong.

### Check frame rate

```bash
ros2 topic hz /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

Typical output at simulation speed:
```
average rate: 30
```

### Inspect message fields

```bash
ros2 topic echo /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image --no-arr
```

Prints width, height, encoding, and header without the raw pixel data.

---

## How the Node Works Internally

```
Gazebo (sim container)
  └── publishes sensor_msgs/Image on gz-transport topic
        └── ros-gz-bridge (sim container) bridges it to ROS 2
              └── camera_node._on_image() called per frame
                    └── CvBridge.imgmsg_to_cv2()  →  BGR numpy array
                          └── (display) cv2.imshow()
                          └── (future) pass frame to inference pipeline
```

`CvBridge` handles the conversion from the ROS `Image` message (raw bytes + metadata) to an OpenCV-compatible `(H, W, 3)` numpy array in BGR order. The queue depth of `10` means up to 10 frames can buffer if processing falls behind before dropping begins.

---

## VS Code Dev Container

To edit and run the node from VS Code:

1. Ensure both containers are running (`docker compose up -d`)
2. Open the `Aero/AI` folder in VS Code
3. Command Palette → `Dev Containers: Reopen in Container`
4. VS Code connects to the `ai` container and opens `/workspace/AI`
5. The Python interpreter is `/usr/bin/python3` (system ROS 2 Python)
6. The ROS 2 extension (`ms-iot.vscode-ros`) provides topic introspection and node graph views

---

## Troubleshooting

**Node starts but logs no frames**

The topic name does not match what Gazebo is publishing. Run `ros2 topic list` and compare against the topic the node is subscribed to (logged at startup). Adjust `world` or `vehicle` parameters.

**`cv_bridge` import error**

The workspace was not sourced. Run `source /opt/ros/jazzy/setup.bash && source /workspace/AI/install/setup.bash` and retry.

**Display window opens but is black**

X11 is not forwarded to the container. See [docs/SETUP_DOCKER.md](../../docs/SETUP_DOCKER.md) for platform-specific X11 setup. As a workaround, skip the display and use `ros2 topic echo` to confirm frames are flowing.

**`colcon build` fails with missing packages**

```bash
sudo rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

This installs any system dependencies declared in `package.xml` that are not yet present in the container.

**Node crashes immediately with `ModuleNotFoundError: rclpy`**

The ROS 2 environment is not sourced. The entrypoint does this automatically at container start, but if you started a new shell manually:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/AI/install/setup.bash
```
