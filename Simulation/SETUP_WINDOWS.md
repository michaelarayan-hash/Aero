# Windows Setup Guide

This guide sets up the Aero simulation stack on Windows using Docker Desktop and VcXsrv for X11 display forwarding.

## Prerequisites

### 1. Docker Desktop
Download and install from https://www.docker.com/products/docker-desktop/

During installation:
- Select **WSL 2** backend (recommended over Hyper-V)
- Enable "Use WSL 2 based engine" in Docker Desktop → Settings → General

### 2. VcXsrv (X11 server for Windows)
Download and install from https://sourceforge.net/projects/vcxsrv/

VcXsrv lets GUI apps running inside the container (Gazebo, QGroundControl) display windows on your Windows desktop.

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

Open a terminal (PowerShell, Git Bash, or WSL):

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

### Step 5: Build the Docker image

```bash
docker compose build
```

This downloads Ubuntu 24.04, Gazebo Harmonic, the PX4 toolchain, and Python dependencies.
**Expected time:** 5–10 minutes (image is ~3–4 GB).

## Running the Simulation

### Step 1: Start the container

```bash
docker compose up
```

**First run only:** the container will automatically clone PX4-Autopilot and compile the SITL binaries.
This takes **~20–30 minutes** and only happens once. Output streams to your terminal so you can follow progress.

Subsequent runs start instantly.

### Step 2: Open a shell inside the container

In a second terminal:

```bash
docker compose exec sim bash
```

You are now inside the container at `/workspace/Simulation`.

### Step 3: Launch the simulation

```bash
python3 sim.py --world tag_demo --gui
```

- The Gazebo GUI window should appear on your Windows desktop (via VcXsrv)
- QGroundControl opens as a second window
- Press Ctrl-C in the first terminal to stop everything

### Step 4: Run algorithm scripts

In the same container shell (or a third terminal with `docker compose exec sim bash`):

```bash
python3 algorithms/takeoff_land.py
python3 algorithms/tag_patrol.py
```

## Troubleshooting

### Gazebo window doesn't appear

- Check VcXsrv is running (system tray icon)
- Verify "Disable access control" was checked in XLaunch
- Check Windows Firewall allows VcXsrv (see Step 2)
- Confirm `.env` contains `DISPLAY=host.docker.internal:0.0`

### "Cannot connect to display" error

```bash
# Inside the container, test X11 connectivity:
xauth list
echo $DISPLAY
```

If `DISPLAY` is empty, `.env` was not loaded — confirm the file exists and is not empty.

### First-run setup fails mid-way

```bash
# Inside the container:
bash /workspace/Simulation/docker/setup.sh --reset
```

This wipes `/root/PX4-Autopilot` and reruns the full setup.

### Docker Desktop WSL 2 performance

For best disk performance, clone the repo inside WSL 2 (`\\wsl$\Ubuntu\home\...`) rather than on the Windows filesystem (`C:\...`). Docker Desktop accesses WSL 2 paths much faster.

## VS Code Dev Container (optional)

Install the **Dev Containers** extension, then:

1. Open the `Aero` folder in VS Code
2. Command Palette → **Dev Containers: Reopen in Container**
3. VS Code attaches to the running container with full IntelliSense

No additional configuration needed — `.devcontainer/devcontainer.json` is already set up.
