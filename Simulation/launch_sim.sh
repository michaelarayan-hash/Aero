#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Harmonic.
#
# Usage:
#   ./launch_sim.sh [world] [vehicle]
#
# World directory resolution (first match wins):
#   1. PX4_GZ_WORLDS env var (point at any directory of .sdf files)
#   2. Default: ~/PX4-Autopilot/Tools/simulation/gz/worlds/
#
# Examples:
#   ./launch_sim.sh                                        # default world, default vehicle
#   ./launch_sim.sh aruco                                  # aruco world
#   ./launch_sim.sh forest x500_depth                     # world + vehicle
#   PX4_GZ_WORLDS=~/PX4-gazebo-models/worlds ./launch_sim.sh baylands
#   PX4_GZ_WORLDS=/my/worlds ./launch_sim.sh my_world

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Resolve defaults from config.py ──────────────────────────────────────────
PX4_ROOT="$(python3 -c "import sys; sys.path.insert(0,'$SCRIPT_DIR'); import config; print(config.PX4_ROOT)")"
DEFAULT_WORLD="$(python3 -c "import sys; sys.path.insert(0,'$SCRIPT_DIR'); import config; print(config.WORLD)")"
DEFAULT_VEHICLE="$(python3 -c "import sys; sys.path.insert(0,'$SCRIPT_DIR'); import config; print(config.VEHICLE)")"
MAVSDK_PORT="$(python3 -c "import sys; sys.path.insert(0,'$SCRIPT_DIR'); import config; print(config.MAVSDK_PORT)")"
QGC_PORT="$(python3 -c "import sys; sys.path.insert(0,'$SCRIPT_DIR'); import config; print(config.QGC_PORT)")"

WORLD="${1:-${PX4_GZ_WORLD:-$DEFAULT_WORLD}}"
VEHICLE="${2:-$DEFAULT_VEHICLE}"

# ── Resolve worlds directory ──────────────────────────────────────────────────
# PX4_GZ_WORLDS env var overrides the default bundled worlds directory.
DEFAULT_WORLDS_DIR="$PX4_ROOT/Tools/simulation/gz/worlds"
WORLDS_DIR="${PX4_GZ_WORLDS:-$DEFAULT_WORLDS_DIR}"
WORLDS_DIR="${WORLDS_DIR/#\~/$HOME}"       # expand leading ~ safely (no eval)
WORLD_SDF="$WORLDS_DIR/${WORLD}.sdf"

# ── Validate world ────────────────────────────────────────────────────────────
if [[ ! -f "$WORLD_SDF" ]]; then
    echo "[ERROR] World SDF not found: $WORLD_SDF"
    echo ""
    if [[ -d "$WORLDS_DIR" ]]; then
        echo "Available worlds in $WORLDS_DIR:"
        for f in "$WORLDS_DIR"/*.sdf; do
            echo "  - $(basename "$f" .sdf)"
        done
    else
        echo "[ERROR] Worlds directory does not exist: $WORLDS_DIR"
    fi
    echo ""
    echo "To use an external worlds directory:"
    echo "  PX4_GZ_WORLDS=/path/to/worlds ./launch_sim.sh <world>"
    echo "  PX4_GZ_WORLDS=~/PX4-gazebo-models/worlds ./launch_sim.sh <world>"
    exit 1
fi

# ── Pre-launch summary ────────────────────────────────────────────────────────
echo "================================================"
echo "  PX4 SITL + Gazebo Harmonic"
echo "================================================"
echo "  PX4 root   : $PX4_ROOT"
echo "  Worlds dir : $WORLDS_DIR"
echo "  World      : $WORLD  ($WORLD_SDF)"
echo "  Vehicle    : $VEHICLE"
echo "  Make       : px4_sitl gz_${VEHICLE}"
echo ""
echo "  MAVLink ports:"
echo "    MAVSDK / offboard scripts : UDP $MAVSDK_PORT"
echo "    QGroundControl broadcast  : UDP $QGC_PORT"
echo ""
echo "  Once running, open Gazebo GUI in a new terminal:"
echo "    gz sim -g"
echo ""
echo "  Connect QGroundControl in another terminal:"
echo "    ~/QGroundControl.AppImage"
echo ""
echo "  Press Ctrl-C to stop simulation."
echo "================================================"
echo ""

# ── Launch (exec = clean Ctrl-C forwarding) ───────────────────────────────────
cd "$PX4_ROOT"
exec env PX4_GZ_WORLD="$WORLD" PX4_GZ_WORLDS="$WORLDS_DIR" make px4_sitl "gz_${VEHICLE}"
