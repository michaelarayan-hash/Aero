# Simulation Setup: Native & Docker

PX4 SITL + Gazebo Harmonic + QGroundControl

## Platform Notes

| Platform | Guide | Notes |
|----------|-------|-------|
| **Ubuntu 22.04 / 24.04** | Below (native) | Recommended; fastest |
| **Ubuntu 26.04+** | Below (Docker) | Native PX4 deps not yet packaged; Docker gives full GPU speed |
| **Ubuntu 20.04 / older** | Use Docker | [SETUP_DOCKER.md](SETUP_DOCKER.md) |
| **macOS** | Use Docker | [SETUP_DOCKER.md](SETUP_DOCKER.md) with colima/OrbStack instead of Docker Desktop |
| **Windows** | See [SETUP_DOCKER.md](SETUP_DOCKER.md) | Docker Desktop + WSL 2 + VcXsrv |

---

## Ubuntu 26.04+ Docker Setup

Ubuntu 26.04 ships with ROS 2 and Gazebo versions that conflict with the PX4 native setup script. Use Docker instead — the container runs Ubuntu 24.04 internally and is functionally identical to a native install.

GPU passthrough gives the container direct hardware OpenGL access, so Gazebo runs at full speed (not the ~17% RTF you'd see with Windows software rendering).

### 1. Create GPU passthrough override

Create `docker-compose.override.yml` once in the `Simulation/` directory (it is gitignored — local to your machine):

```bash
cat > ~/Aero/Simulation/docker-compose.override.yml << 'EOF'
services:
  sim:
    devices:
      - /dev/dri:/dev/dri
    group_add:
      - video
      - render
EOF
```

Docker Compose automatically merges this file — no extra flags needed.

### 2. Configure display

```bash
cp ~/Aero/Simulation/.env.example ~/Aero/Simulation/.env
```

Open `.env` and uncomment the Linux line. Verify `echo $DISPLAY` on your host matches — on Ubuntu 26 with Wayland + XWayland it is typically `:0`.

### 3. Allow container X11 access (once per session)

```bash
xhost +local:
```

Add to `~/.bashrc` to avoid repeating this on every login.

### 4. Build and run

Follow [SETUP_DOCKER.md](SETUP_DOCKER.md) from **"Build the Docker Image"** onward. The GPU override is applied automatically.

---

## Ubuntu 22.04 / 24.04 Native Setup

### Prerequisites (one-time installation)

### 1. Install PX4 Autopilot

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh          # ~10 min, installs all deps including Gazebo
make px4_sitl gz_x500                 # first build ~5–15 min
```

> Reboot or re-source `~/.bashrc` after setup completes.

### 2. Install QGroundControl

```bash
wget -O ~/QGroundControl.AppImage \
  https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ~/QGroundControl.AppImage

# Ubuntu 22.04+ also needs:
sudo apt-get install -y libfuse2
```

### 3. Install uv (Python package manager)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Python Environment Setup

```bash
cd ~/Aero/Simulation
uv venv .venv
source .venv/bin/activate
uv sync
```

Alternative (without uv):
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Next Steps

See [RUNNING_SIMULATION.md](RUNNING_SIMULATION.md) for:
- Worlds (built-in, external, custom)
- Vehicle variants
- Camera feed setup
- Algorithm scripts (takeoff, auto-landing, etc.)
- MAVLink ports
