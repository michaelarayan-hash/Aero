# Docker Setup

Run the full Aero stack — Simulation and AI — via a single `docker compose up` from the repo root.

## Containers

| Service | Image | Contents |
|---|---|---|
| `sim` | `aero-sim:latest` | Gazebo Harmonic, PX4 SITL, ROS2 Jazzy, ros-gz-bridge |
| `ai` | `aero-ai:latest` | ROS2 Jazzy, nav2, PyTorch, ONNX Runtime, cv-bridge |

Both containers share a Docker bridge network. ROS2 topics are visible across both automatically.

---

## Prerequisites

- **Docker Desktop** (Windows/macOS) or **Docker Engine** (Linux)
- **VS Code** + [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- **X11 server** for Gazebo/QGC GUI — see [Simulation/docs/SETUP_DOCKER.md](../Simulation/docs/SETUP_DOCKER.md)

---

## First-Time Setup

### 1. Clone and configure display

```bash
git clone <repo-url>
cd Aero
cp Simulation/.env.example Simulation/.env
```

Edit `Simulation/.env` and uncomment the line for your OS. See [Simulation/docs/SETUP_DOCKER.md](../Simulation/docs/SETUP_DOCKER.md) for platform-specific X11 setup.

### 2. Build both images

```bash
docker compose build
```

**Expected time:** 25–40 minutes. Only needed once per version change.

### 3. Start both containers

```bash
docker compose up -d
```

---

## Daily Workflow

### Open a shell in either container

```bash
docker compose exec sim bash
docker compose exec ai bash
```

### VS Code Dev Containers

With both containers running, open the relevant subfolder in VS Code:

| Container | Open folder | Then |
|---|---|---|
| Simulation | `Aero/Simulation` | Command Palette → `Dev Containers: Reopen in Container` |
| AI | `Aero/AI` | Command Palette → `Dev Containers: Reopen in Container` |

VS Code opens at `/workspace/Simulation` or `/workspace/AI` respectively.

### Build and source a ROS2 workspace (AI container)

```bash
colcon build
source install/setup.bash
```

`source install/setup.bash` is always manual after a colcon build — this is standard ROS2 workflow.

### Run the simulation

```bash
# In sim container
python3 sim.py --world forest --gui
```

### Verify ROS2 topic visibility across containers

```bash
# In either container
ros2 topic list
```

Topics published in `sim` (e.g. from ros-gz-bridge) should be visible in `ai` and vice versa.

---

## Stop / Reset

```bash
docker compose down          # stop and remove containers
docker compose build --no-cache  # full rebuild of both images
```

To rebuild a single image:

```bash
docker compose build sim
docker compose build ai
```

---

## Deploying AI to Jetson Orin Nano

The AI image is built for x86 during development. For deployment to the Jetson, build the same Dockerfile for ARM64:

```bash
docker buildx build --platform linux/arm64 -t aero-ai:arm64 AI/
```

Push to a registry and pull on the Jetson. The Simulation container is never deployed to the Jetson.
