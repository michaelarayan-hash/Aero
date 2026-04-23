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
