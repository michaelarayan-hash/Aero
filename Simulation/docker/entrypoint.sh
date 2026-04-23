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
