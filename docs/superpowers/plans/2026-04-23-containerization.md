# Containerization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Package the full Aero simulation stack (PX4 SITL + Gazebo Harmonic + QGroundControl + Python algorithms) into a Docker image so teammates on Windows, macOS, and Linux can run `docker compose up` and get a working environment.

**Architecture:** Single Ubuntu 24.04 container with Gazebo Harmonic pre-installed and the PX4 toolchain baked in. PX4 source is cloned and compiled on first run into a named volume (`px4-build`) so the 5 GB build is never part of the image. The repo is bind-mounted at `/workspace` so code edits on the host take effect immediately. X11 is forwarded to the host display via VcXsrv (Windows) or XQuartz (macOS).

**Tech Stack:** Docker 24+, Docker Compose v2, Ubuntu 24.04, Gazebo Harmonic (`gz-harmonic` apt), PX4-Autopilot (SITL), uv, Python 3.12, MAVSDK, OpenCV

---

## File Map

| Path | Action | Responsibility |
|---|---|---|
| `Simulation/.dockerignore` | Create | Excludes venv, pycache, calib data from build context |
| `Simulation/docker/Dockerfile` | Create | Ubuntu 24.04 + gz-harmonic + build toolchain + Python deps |
| `Simulation/docker/setup.sh` | Create | First-run: clone PX4, build, download QGC |
| `Simulation/docker/entrypoint.sh` | Create | Calls setup.sh if needed; symlinks worlds/models; drops to bash |
| `Simulation/docker-compose.yml` | Create | Volume, DISPLAY, ports, bind-mount |
| `Simulation/.env.example` | Create | DISPLAY template for each OS |
| `Simulation/.devcontainer/devcontainer.json` | Create | VS Code "Reopen in Container" support |
| `Simulation/SETUP_WINDOWS.md` | Create | Step-by-step Windows onboarding guide |

`.gitignore` already excludes `.env` — no change needed.

---

## Task 1: .dockerignore

**Files:**
- Create: `Simulation/.dockerignore`

- [ ] **Step 1: Create `.dockerignore`**

```
# Python
**/__pycache__/
**/*.pyc
**/*.pyo

# Virtual environments
.venv/
venv/

# Test cache
.pytest_cache/

# Calibration data (large, not needed for sim)
Code/Callibration/calib_images/
Code/Callibration/undistorted/

# Git
.git/
.worktrees/

# OS
.DS_Store
Thumbs.db

# Secrets
.env
```

Save to `Simulation/.dockerignore`.

- [ ] **Step 2: Verify syntax is valid (no leading `/` on patterns)**

```bash
cat Simulation/.dockerignore
```

Expected: file prints cleanly, no `/`-prefixed lines.

- [ ] **Step 3: Commit**

```bash
git add Simulation/.dockerignore
git commit -m "feat(docker): add .dockerignore"
```

---

## Task 2: docker/setup.sh

First-run script. Runs once inside the container when `px4-build` volume is empty.

**Files:**
- Create: `Simulation/docker/setup.sh`

- [ ] **Step 1: Create `Simulation/docker/setup.sh`**

```bash
#!/usr/bin/env bash
# First-run setup: clone PX4, build SITL, download QGC.
# Called automatically by entrypoint.sh when /root/PX4-Autopilot/.git is absent.
# Safe to re-run: skips steps that are already done.
# --reset flag: wipes /root/PX4-Autopilot and starts over.
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

PX4_DIR="/root/PX4-Autopilot"
QGC_PATH="/root/QGroundControl.AppImage"
QGC_URL="https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage"

if [[ "${1:-}" == "--reset" ]]; then
    echo "[INFO] --reset: removing $PX4_DIR"
    rm -rf "$PX4_DIR"
fi

echo ""
echo "================================================"
echo "  Aero first-time setup"
echo "  This will take ~20-30 minutes."
echo "================================================"
echo ""

# 1. Clone PX4
if [[ ! -d "$PX4_DIR/.git" ]]; then
    echo "[1/4] Cloning PX4-Autopilot (shallow + shallow submodules)..."
    git clone --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
else
    echo "[1/4] PX4 source already present — skipping clone."
fi

# 2. PX4 runtime deps (ubuntu.sh installs remaining apt packages)
echo "[2/4] Running PX4 ubuntu.sh (--no-nuttx)..."
bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx

# 3. Build default vehicle
echo "[3/4] Building PX4 SITL (gz_x500_mono_cam)..."
cd "$PX4_DIR"
make px4_sitl gz_x500_mono_cam

# 4. QGroundControl AppImage
if [[ ! -f "$QGC_PATH" ]]; then
    echo "[4/4] Downloading QGroundControl..."
    wget -q --show-progress -O "$QGC_PATH" "$QGC_URL"
    chmod +x "$QGC_PATH"
else
    echo "[4/4] QGroundControl already present — skipping download."
fi

echo ""
echo "================================================"
echo "[OK] Setup complete!"
echo ""
echo "  Start the simulation:"
echo "    cd /workspace/Simulation"
echo "    python3 sim.py --world tag_demo --gui"
echo "================================================"
echo ""
```

- [ ] **Step 2: Verify shell syntax**

```bash
bash -n Simulation/docker/setup.sh
```

Expected: no output (no syntax errors).

- [ ] **Step 3: Make executable**

```bash
chmod +x Simulation/docker/setup.sh
```

- [ ] **Step 4: Commit**

```bash
git add Simulation/docker/setup.sh
git commit -m "feat(docker): add first-run setup.sh"
```

---

## Task 3: docker/entrypoint.sh

Runs on every `docker compose up`. Calls `setup.sh` on first start; refreshes world/model symlinks on every start; then drops to bash.

**Files:**
- Create: `Simulation/docker/entrypoint.sh`

- [ ] **Step 1: Create `Simulation/docker/entrypoint.sh`**

```bash
#!/usr/bin/env bash
set -euo pipefail

# ── First-time PX4 setup ──────────────────────────────────────────────────────
if [[ ! -d /root/PX4-Autopilot/.git ]]; then
    echo "[entrypoint] PX4 not found — running first-time setup (~20-30 min)..."
    bash /workspace/Simulation/docker/setup.sh
fi

# ── Refresh world/model symlinks on every start ───────────────────────────────
# Custom .sdf worlds and model directories in the bind-mounted repo are symlinked
# into PX4's directories so Gazebo can find them without PX4_GZ_WORLDS overrides.
PX4_WORLDS="/root/PX4-Autopilot/Tools/simulation/gz/worlds"
PX4_MODELS="/root/PX4-Autopilot/Tools/simulation/gz/models"

if [[ -d "$PX4_WORLDS" && -d /workspace/Simulation/worlds ]]; then
    for world in /workspace/Simulation/worlds/*.sdf; do
        [[ -f "$world" ]] && ln -sf "$world" "$PX4_WORLDS/$(basename "$world")"
    done
fi

if [[ -d "$PX4_MODELS" && -d /workspace/Simulation/models ]]; then
    for model_dir in /workspace/Simulation/models/*/; do
        [[ -d "$model_dir" ]] && \
            ln -sf "$model_dir" "$PX4_MODELS/$(basename "$model_dir")"
    done
fi

exec "$@"
```

- [ ] **Step 2: Verify shell syntax**

```bash
bash -n Simulation/docker/entrypoint.sh
```

Expected: no output.

- [ ] **Step 3: Make executable**

```bash
chmod +x Simulation/docker/entrypoint.sh
```

- [ ] **Step 4: Commit**

```bash
git add Simulation/docker/entrypoint.sh
git commit -m "feat(docker): add entrypoint with world/model symlinking"
```

---

## Task 4: docker/Dockerfile

**Files:**
- Create: `Simulation/docker/Dockerfile`

- [ ] **Step 1: Create `Simulation/docker/Dockerfile`**

```dockerfile
FROM ubuntu:24.04

# Suppress interactive apt prompts throughout build and in setup.sh
ENV DEBIAN_FRONTEND=noninteractive
# Makes QGroundControl AppImage bypass FUSE (required in Docker without --privileged)
ENV APPIMAGE_EXTRACT_AND_RUN=1
# uv install dir
ENV PATH="/root/.local/bin:$PATH"

# ── 1. Base packages ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git gnupg lsb-release wget sudo \
    xauth \
    libfuse2 \
    python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── 2. Gazebo Harmonic ────────────────────────────────────────────────────────
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ── 3. PX4 build toolchain ────────────────────────────────────────────────────
# Installs cmake, ninja, gcc/g++ and other build essentials.
# PX4's ubuntu.sh (run by setup.sh on first container start) installs remaining
# runtime deps (Python packages, simulation-specific tools) after cloning PX4.
RUN apt-get update && apt-get install -y --no-install-recommends \
    astyle build-essential cmake cppcheck \
    file g++ gcc gdb \
    make ninja-build \
    python3-dev python3-setuptools \
    rsync unzip xxd libxml2-utils \
    && rm -rf /var/lib/apt/lists/*

# ── 4. uv ─────────────────────────────────────────────────────────────────────
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# ── 5. Python project deps ────────────────────────────────────────────────────
# Pre-install from requirements.txt so algorithm scripts work immediately.
# Build context is Simulation/ so requirements.txt is at the root of context.
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# ── 6. Entrypoint ─────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace/Simulation
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

Note: `sudo` is installed because PX4's `ubuntu.sh` script uses it internally. Running as root, `sudo` passes through transparently.

Note: Build context is `Simulation/` (set in `docker-compose.yml` with `context: .`), so `COPY requirements.txt` copies `Simulation/requirements.txt` and `COPY docker/entrypoint.sh` copies `Simulation/docker/entrypoint.sh`.

- [ ] **Step 2: Verify Dockerfile syntax with hadolint (if available), else visually inspect**

```bash
docker run --rm -i hadolint/hadolint < Simulation/docker/Dockerfile 2>/dev/null \
    || echo "hadolint not available — visually checked"
```

- [ ] **Step 3: Commit**

```bash
git add Simulation/docker/Dockerfile
git commit -m "feat(docker): add Dockerfile (Ubuntu 24.04 + gz-harmonic + toolchain)"
```

---

## Task 5: Build the image and verify tools

- [ ] **Step 1: Build the image**

```bash
cd Simulation
docker compose build
```

Expected: build succeeds, final line is `=> => writing image sha256:...`.
This will take 5–10 minutes on first build (gz-harmonic is large).

- [ ] **Step 2: Verify Gazebo is installed**

```bash
docker run --rm aero-sim:latest gz sim --versions
```

Expected output contains: `8.x.x`

- [ ] **Step 3: Verify Python deps are available**

```bash
docker run --rm aero-sim:latest python3 -c "import cv2, mavsdk, numpy; print('OK', cv2.__version__)"
```

Expected: `OK 4.x.x`

- [ ] **Step 4: Verify gz-transport Python bindings**

```bash
docker run --rm aero-sim:latest python3 -c "
import sys; sys.path.insert(0, '/usr/lib/python3/dist-packages')
from gz.transport13 import Node
print('gz-transport13 OK')
"
```

Expected: `gz-transport13 OK`

- [ ] **Step 5: Verify uv is on PATH**

```bash
docker run --rm aero-sim:latest uv --version
```

Expected: `uv 0.x.x`

- [ ] **Step 6: Commit (nothing new — this is a verification task)**

No commit needed. If any step above failed, fix `Dockerfile` and re-run `docker compose build` before proceeding.

---

## Task 6: docker-compose.yml and .env.example

**Files:**
- Create: `Simulation/docker-compose.yml`
- Create: `Simulation/.env.example`

- [ ] **Step 1: Create `Simulation/docker-compose.yml`**

```yaml
services:
  sim:
    build:
      context: .
      dockerfile: docker/Dockerfile
    image: aero-sim:latest
    volumes:
      - px4-build:/root/PX4-Autopilot        # PX4 source + build (persisted across runs)
      - ..:/workspace                         # repo bind-mount (live edits from host IDE)
      - /tmp/.X11-unix:/tmp/.X11-unix         # X11 socket — Linux only, ignored on Win/Mac
    environment:
      - DISPLAY=${DISPLAY}
      - APPIMAGE_EXTRACT_AND_RUN=1            # QGC AppImage runs without FUSE
      - GZ_SIM_RESOURCE_PATH=/workspace/Simulation/models
    ports:
      - "14540:14540/udp"    # MAVSDK offboard scripts (if run on host)
      - "14550:14550/udp"    # QGroundControl (if run on host)
    stdin_open: true
    tty: true

volumes:
  px4-build:
```

- [ ] **Step 2: Create `Simulation/.env.example`**

```dotenv
# Copy this file to .env and uncomment the line for your OS.
# .env is gitignored — never commit it.

# ── Windows (VcXsrv) ─────────────────────────────────────────────────────────
# 1. Install VcXsrv: https://sourceforge.net/projects/vcxsrv/
# 2. Run XLaunch → Multiple windows → Display 0 → check "Disable access control"
# 3. Allow VcXsrv through Windows Firewall when prompted
DISPLAY=host.docker.internal:0.0

# ── macOS (XQuartz) ───────────────────────────────────────────────────────────
# 1. Install XQuartz: https://www.xquartz.org/
# 2. XQuartz Preferences → Security → check "Allow connections from network clients"
# 3. Log out and back in (or restart) after enabling
# DISPLAY=host.docker.internal:0

# ── Linux (native X11) ────────────────────────────────────────────────────────
# DISPLAY=:0
```

- [ ] **Step 3: Validate compose config**

```bash
cd Simulation
docker compose config
```

Expected: prints the resolved YAML with no errors. `DISPLAY` will show a warning if `.env` doesn't exist yet — that's fine at this stage.

- [ ] **Step 4: Commit**

```bash
git add Simulation/docker-compose.yml Simulation/.env.example
git commit -m "feat(docker): add docker-compose.yml and .env.example"
```

---

## Task 7: .devcontainer/devcontainer.json

Enables VS Code "Reopen in Container". References the same compose service — no second image.

**Files:**
- Create: `Simulation/.devcontainer/devcontainer.json`

- [ ] **Step 1: Create `Simulation/.devcontainer/devcontainer.json`**

```json
{
  "name": "Aero Simulation",
  "dockerComposeFile": "../docker-compose.yml",
  "service": "sim",
  "workspaceFolder": "/workspace",
  "remoteUser": "root",
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-python.python",
        "ms-python.pylance"
      ],
      "settings": {
        "python.defaultInterpreterPath": "/usr/bin/python3"
      }
    }
  }
}
```

- [ ] **Step 2: Verify JSON is valid**

```bash
python3 -c "import json; json.load(open('Simulation/.devcontainer/devcontainer.json')); print('JSON OK')"
```

Expected: `JSON OK`

- [ ] **Step 3: Commit**

```bash
git add Simulation/.devcontainer/devcontainer.json
git commit -m "feat(docker): add devcontainer.json for VS Code support"
```

---

## Task 8: SETUP_WINDOWS.md

**Files:**
- Create: `Simulation/SETUP_WINDOWS.md`

- [ ] **Step 1: Create `Simulation/SETUP_WINDOWS.md`**

```markdown
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
```

- [ ] **Step 2: Commit**

```bash
git add Simulation/SETUP_WINDOWS.md
git commit -m "docs: add Windows setup guide for Docker + VcXsrv"
```

---

## Task 9: End-to-end smoke test

Verifies the full stack works together before declaring done.

- [ ] **Step 1: Copy .env and start container**

```bash
cd Simulation
cp .env.example .env
# Edit .env: uncomment the line for your OS
docker compose up -d
```

Expected: container starts, exits to bash (or waits — depends on TTY).

- [ ] **Step 2: Verify container is running**

```bash
docker compose ps
```

Expected: `sim` service shows `running`.

- [ ] **Step 3: Verify workspace bind-mount**

```bash
docker compose exec sim ls /workspace/Simulation/algorithms/
```

Expected: lists `takeoff_land.py`, `tag_patrol.py`, `camera_feed.py`, etc.

- [ ] **Step 4: Verify PX4 worlds directory will be populated after setup**

```bash
docker compose exec sim bash -c "ls /root/PX4-Autopilot/ 2>/dev/null || echo 'not built yet — setup.sh will run on next interactive start'"
```

Expected: either lists PX4 source files (if setup ran), or prints "not built yet".

- [ ] **Step 5: Verify gz-sim is callable**

```bash
docker compose exec sim gz sim --versions
```

Expected: `8.x.x`

- [ ] **Step 6: Verify Python algorithm imports**

```bash
docker compose exec sim python3 -c "
import sys, os
sys.path.insert(0, '/workspace/Simulation')
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import config
print('config OK — PX4_ROOT:', config.PX4_ROOT)
import cv2, mavsdk, numpy
print('deps OK')
"
```

Expected:
```
config OK — PX4_ROOT: /root/PX4-Autopilot
deps OK
```

- [ ] **Step 7: Stop and clean up test container**

```bash
docker compose down
```

- [ ] **Step 8: Final commit**

```bash
git add .
git commit -m "feat(docker): complete containerization setup

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>"
```
