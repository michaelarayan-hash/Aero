# Simulation Setup: Native & Docker

PX4 SITL + Gazebo Harmonic + QGroundControl

## Platform Notes

| Platform | Guide | Notes |
|----------|-------|-------|
| **Ubuntu 22.04 / 24.04** | Below (native) | Recommended; fastest |
| **Ubuntu 20.04 / older** | Use Docker | `SETUP_WINDOWS.md` docker-compose works on Linux too |
| **macOS** | Use Docker | `SETUP_WINDOWS.md` docker-compose with colima/OrbStack instead of Docker Desktop |
| **Windows** | See `SETUP_WINDOWS.md` | Docker Desktop + WSL 2 + VcXsrv |

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
