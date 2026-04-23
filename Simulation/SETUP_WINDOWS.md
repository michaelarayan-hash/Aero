# Windows Setup Guide

Run the Aero simulation stack on Windows using Docker Desktop and VcXsrv. The image is self-contained — PX4, Gazebo Harmonic, QGroundControl and all Python deps are baked in at `docker compose build` time, so `docker compose up` is instant.

## Prerequisites

### 1. Docker Desktop
Download and install from https://www.docker.com/products/docker-desktop/

During installation:
- Select **WSL 2** backend (recommended over Hyper-V)
- Enable "Use WSL 2 based engine" in Docker Desktop → Settings → General

### 2. VcXsrv (X11 server for Windows)
Download and install from https://sourceforge.net/projects/vcxsrv/

VcXsrv lets GUI apps running inside the container (Gazebo, QGroundControl) display windows on your Windows desktop.

### 3. Git line-ending config
Shell scripts in this repo must be checked out with LF endings. The `.gitattributes` handles this automatically, **but only if you clone with a sane config**. Before cloning, run once:

```bash
git config --global core.autocrlf input
```

If you already cloned with `core.autocrlf=true` and the container crashes with `/usr/bin/env: 'bash\r': No such file or directory`, re-normalize the working tree:

```bash
git add --renormalize .
```

## First-Time Setup

### Step 1: Start VcXsrv

Open **XLaunch** from the Start menu and configure it:

1. Display settings → **Multiple windows** → Display number: **0** → Next
2. Client startup → **Start no client** → Next
3. Extra settings → check **Disable access control** → Next → Finish

> "Disable access control" allows the Docker container to connect to VcXsrv.
> You can save the configuration as a shortcut for future launches.

### Step 2: Allow VcXsrv through Windows Firewall

When VcXsrv first launches, Windows will show a firewall prompt — click **Allow access** for both private and public networks.

If you missed the prompt, add the rule manually:
1. Open **Windows Defender Firewall** → Advanced Settings
2. Inbound Rules → New Rule → Program
3. Browse to `C:\Program Files\VcXsrv\vcxsrv.exe`
4. Allow the connection → apply to all profiles → Finish

### Step 3: Clone the repository

```bash
git clone <repo-url>
cd Aero/Simulation
```

### Step 4: Configure your .env file

```bash
cp .env.example .env
```

Open `.env` — the first uncommented line should already be:

```
DISPLAY=host.docker.internal:0.0
```

Leave it as-is. This tells the container to forward its display to your VcXsrv instance.

### Step 5: Build the image

```bash
docker compose build
```

This installs Ubuntu 24.04, Gazebo Harmonic, the PX4 toolchain, clones PX4-Autopilot, runs its setup script, builds the SITL binary, downloads QGroundControl, and installs the project Python deps — all into a single image.

**Expected time:** 25–35 minutes. **Image size:** ~8–12 GB.

You only do this once per PX4 version.

To pin a specific PX4 release (recommended for reproducibility):

```bash
docker compose build --build-arg PX4_REF=v1.15.2
```

## Running the Simulation

### Step 1: Start the container

```bash
docker compose up -d
```

Starts instantly. The container stays up in the background.

### Step 2: Open a shell inside the container

```bash
docker compose exec sim bash
```

You are now inside the container at `/workspace/Simulation`.

### Step 3: Launch the simulation

```bash
python3 sim.py --world forest --gui
```

- PX4 SITL server starts, then QGroundControl opens as a window via VcXsrv
- `--gui` adds the Gazebo 3D view (also via VcXsrv)
- First launch of a specific vehicle (default `x500_mono_cam_down`) does a small incremental compile (~30–60s); later launches are instant
- Ctrl-C in the shell stops everything

### Step 4: Run algorithms and camera feed

In the same container shell (or another `docker compose exec sim bash` terminal):

```bash
python3 algorithms/takeoff_land.py
python3 algorithms/camera_feed.py --world forest
```

Any algorithm in `algorithms/` works the same way.

## Stopping / Resetting

```bash
docker compose down       # stop and remove the container (image stays)
docker compose down -v    # also remove volumes (there are none, but future-proof)
```

To update PX4 or rebuild from scratch:

```bash
docker compose build --no-cache
```

## Troubleshooting

### Gazebo / QGC window doesn't appear

- Check VcXsrv is running (system tray icon)
- Verify "Disable access control" was checked in XLaunch
- Check Windows Firewall allows VcXsrv (see Step 2)
- Confirm `.env` contains `DISPLAY=host.docker.internal:0.0`

### "Cannot connect to display" error

Inside the container:
```bash
echo $DISPLAY       # should print host.docker.internal:0.0
xauth list          # should not error
```

If `DISPLAY` is empty, `.env` was not loaded — confirm the file exists next to `docker-compose.yml`.

### Container exits with `/usr/bin/env: 'bash\r'`

Windows line endings on shell scripts. See the "Git line-ending config" prerequisite above — run `git add --renormalize .` and commit, or re-clone with `core.autocrlf=input`.

### Docker Desktop WSL 2 performance

For best disk performance, clone the repo inside WSL 2 (`\\wsl$\Ubuntu\home\...`) rather than on the Windows filesystem (`C:\...`). Docker Desktop accesses WSL 2 paths much faster.

## VS Code Dev Container (optional)

Install the **Dev Containers** extension, then:

1. Open the `Aero` folder in VS Code
2. Command Palette → **Dev Containers: Reopen in Container**
3. VS Code attaches to the running container with full IntelliSense

The devcontainer config opens at `/workspace`; `cd Simulation` before running `sim.py`.
