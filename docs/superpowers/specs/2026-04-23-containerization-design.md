# Containerization Design: Aero Simulation Stack

**Date:** 2026-04-23
**Status:** Approved

## Overview

Containerize the full simulation stack (PX4 SITL + Gazebo Harmonic + QGroundControl + Python algorithms) into a single Docker image based on Ubuntu 24.04, so team members on Windows, macOS, and Linux can run an identical environment via Docker Desktop. GUI apps (Gazebo, QGC) are forwarded to the host display via X11 (VcXsrv on Windows, XQuartz on macOS, native on Linux).

## Goals

- Any teammate runs `docker compose up` and has a working sim in one command
- No host Python setup required — algorithm scripts run inside the container
- Repo is bind-mounted so edits in the host IDE take effect immediately
- Easy upgrade path to VS Code Dev Container (Option C) with no second image

## Non-Goals

- GPU acceleration / hardware rendering (software rendering via llvmpipe is sufficient for SITL)
- CI/headless-only pipeline (not in scope for this iteration)
- Raspberry Pi (arm64) build of the container (SITL is x86_64 only)

---

## Repository Layout

All new files live under `Simulation/`:

```
Simulation/
  docker/
    Dockerfile          # Ubuntu 24.04 + gz-harmonic + toolchain + uv
    setup.sh            # first-run: clone PX4, make, download QGC
    entrypoint.sh       # checks if setup needed, then drops to bash
  docker-compose.yml    # volume, DISPLAY, ports, repo bind-mount
  .env.example          # DISPLAY template — teammates copy to .env
  .dockerignore
  SETUP_WINDOWS.md      # step-by-step Windows setup guide
  .devcontainer/
    devcontainer.json   # VS Code "Reopen in Container" — references compose service
```

---

## Dockerfile

Base image: `ubuntu:24.04`

Build layers (in order):

1. **System packages**
   - Gazebo Harmonic via the official `packages.osrfoundation.org` apt repo: `gz-harmonic`
   - PX4 build dependencies via `PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx` (run inside the build — keeps deps in sync with upstream automatically)
   - Supporting packages: `libfuse2` (QGC AppImage), `git`, `curl`, `python3`, `python3-pip`, `xauth`

2. **uv** — installed via `https://astral.sh/uv/install.sh`, added to `PATH`

3. **Python venv** — `COPY pyproject.toml uv.lock` then `uv sync` baked in so all Python deps are pre-installed. The repo itself is not copied into the image; it arrives via bind-mount at runtime.

4. **PX4 source / build** — intentionally excluded from the image. Handled by `setup.sh` on first run into the `px4-build` named volume.

5. **Entrypoint** — `ENTRYPOINT ["/entrypoint.sh"]`

Resulting image size: ~3–4 GB (toolchain + gz-harmonic + Python deps).

---

## docker-compose.yml

```yaml
services:
  sim:
    build: ./docker
    image: aero-sim:latest
    volumes:
      - px4-build:/root/PX4-Autopilot   # PX4 source + build artifacts (persisted)
      - ..:/workspace                    # repo bind-mount (live edits)
      - /tmp/.X11-unix:/tmp/.X11-unix    # X11 socket (Linux only, ignored on Win/Mac)
    environment:
      - DISPLAY=${DISPLAY}
      - GZ_SIM_RESOURCE_PATH=/workspace/Simulation/models
    ports:
      - "14540:14540/udp"   # MAVSDK / offboard scripts (external access)
      - "14550:14550/udp"   # QGroundControl (external access)
    stdin_open: true
    tty: true

volumes:
  px4-build:
```

PX4, QGC, and algorithm scripts all run inside the container and connect via `localhost`. The port mappings expose 14540/14550 externally for anyone running QGC or algorithm scripts on the host instead of inside the container. `network_mode: host` is intentionally omitted — it doesn't work cross-platform on Docker Desktop (on Windows/Mac it refers to the Linux VM's network, not the host machine).

---

## .env.example

Teammates copy this to `.env` and uncomment their OS line:

```dotenv
# Windows — VcXsrv running with "Disable access control" checked
DISPLAY=host.docker.internal:0.0

# macOS — XQuartz running with "Allow connections from network clients" enabled
# DISPLAY=host.docker.internal:0

# Linux — native X11
# DISPLAY=:0
```

---

## setup.sh (first-run script)

Runs automatically on first `docker compose up` when `/root/PX4-Autopilot/.git` is absent. Steps:

1. `git clone --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git /root/PX4-Autopilot`
2. `bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx` (installs any remaining runtime deps)
3. `cd /root/PX4-Autopilot && make px4_sitl gz_x500_mono_cam` (default vehicle; additional vehicles build on demand)
4. Download `QGroundControl.AppImage` to `/root/QGroundControl.AppImage`
5. Print `[OK] Setup complete — run: python3 sim.py --world tag_demo --gui`

**Recovery flag:** `bash setup.sh --reset` wipes `/root/PX4-Autopilot` and re-runs all steps. Useful if a build is corrupted or a new PX4 version is needed.

All output is streamed to stdout so teammates can follow progress during the ~20–30 min first run.

---

## entrypoint.sh

```bash
#!/usr/bin/env bash
set -euo pipefail
if [[ ! -d /root/PX4-Autopilot/.git ]]; then
    echo "[INFO] PX4 not built yet — running setup (this takes ~20-30 min)..."
    bash /workspace/Simulation/docker/setup.sh
fi
exec "$@"
```

Fast path (every run after first): `.git` present → drops straight to bash. No delay.

---

## .devcontainer/devcontainer.json (Option C)

```json
{
  "name": "Aero Simulation",
  "dockerComposeFile": "../docker-compose.yml",
  "service": "sim",
  "workspaceFolder": "/workspace",
  "remoteUser": "root"
}
```

VS Code "Reopen in Container" uses the same image and compose service. No second Dockerfile or image. IntelliSense works because the Python venv is at `/workspace/Simulation/.venv` (bind-mounted from the repo).

---

## Windows Setup (SETUP_WINDOWS.md)

Step-by-step guide covering:

1. **Install VcXsrv** from [sourceforge.net/projects/vcxsrv](https://sourceforge.net/projects/vcxsrv/)
2. **Launch XLaunch** — select "Multiple windows", display number `0`, check "Disable access control"
3. **Windows Firewall** — allow VcXsrv through the firewall when prompted (or add a manual inbound rule for TCP/UDP port 6000)
4. **Install Docker Desktop** — enable WSL2 backend
5. **Clone repo and configure .env**:
   ```
   cp .env.example .env
   # DISPLAY=host.docker.internal:0.0 is already uncommented
   ```
6. **First run**:
   ```
   cd Simulation
   docker compose up
   # Wait ~20-30 min for PX4 to build
   ```
7. **Open a second terminal**:
   ```
   docker compose exec sim bash
   cd /workspace/Simulation
   python3 sim.py --world tag_demo --gui
   ```
8. **Gazebo GUI** should open in a VcXsrv window on the host desktop.

---

## Data Flow

```
Host (Windows/Mac/Linux)
  └── Docker Desktop
        └── aero-sim container
              ├── /root/PX4-Autopilot  ← named volume px4-build
              ├── /workspace           ← bind-mount of repo
              ├── PX4 SITL process     → MAVLink UDP :14540, :14550
              ├── gz sim (headless server)
              └── gz sim -g / QGC      → X11 → VcXsrv/XQuartz → host display
```

---

## Key Constraints

| Constraint | Decision |
|---|---|
| GUI on Windows/Mac | X11 forwarding via VcXsrv/XQuartz — no VNC/noVNC |
| PX4 build size (~5 GB) | Named volume, built once on first run, never in image |
| QGC AppImage + FUSE | Run with `--appimage-extract-and-run` flag, no `--privileged` needed |
| MAVLink UDP routing | Bridge mode + port mappings — `network_mode: host` skipped (not cross-platform) |
| arm64 (Raspberry Pi) | Out of scope — SITL is x86_64 only |
| VS Code Dev Container | `.devcontainer/devcontainer.json` references existing compose service |
