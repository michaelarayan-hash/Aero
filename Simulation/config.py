"""Central configuration for Aero simulation stack.

Every other file imports from here. Override WORLD via PX4_GZ_WORLD env var.
"""
import os
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────
PX4_ROOT = Path("~/PX4-Autopilot").expanduser()
QGC_APPIMAGE = Path("~/QGroundControl.AppImage").expanduser()
WORLDS_DIR = PX4_ROOT / "Tools/simulation/gz/worlds"

# ── Vehicle ───────────────────────────────────────────────────────────────────
# Camera variants: x500_mono_cam, x500_mono_cam_down, x500_depth
VEHICLE = "x500_mono_cam_down"
MAKE_TARGET = f"gz_{VEHICLE}"

# ── World (overridable via env var) ───────────────────────────────────────────
WORLD = os.environ.get("PX4_GZ_WORLD", "default")

# ── MAVLink ports ─────────────────────────────────────────────────────────────
MAVSDK_PORT = 14540       # offboard / algorithm scripts
QGC_PORT = 14550          # QGroundControl broadcast
MAVSDK_ADDRESS = f"udpin://0.0.0.0:{MAVSDK_PORT}"

# ── Timeouts ──────────────────────────────────────────────────────────────────
CONNECT_TIMEOUT_SEC = 30
TELEMETRY_TIMEOUT_SEC = 5


def validate() -> list[str]:
    """Check that required paths exist. Returns list of error strings (empty = OK)."""
    errors = []
    if not PX4_ROOT.exists():
        errors.append(f"PX4_ROOT not found: {PX4_ROOT}")
    if not WORLDS_DIR.exists():
        errors.append(f"Worlds directory not found: {WORLDS_DIR}")
    world_sdf = WORLDS_DIR / f"{WORLD}.sdf"
    if WORLDS_DIR.exists() and not world_sdf.exists():
        available = [p.stem for p in WORLDS_DIR.glob("*.sdf")]
        errors.append(
            f"World SDF not found: {world_sdf}\n"
            f"  Available worlds: {', '.join(sorted(available))}"
        )
    return errors


if __name__ == "__main__":
    print("=== Aero Simulation Config ===")
    print(f"  PX4_ROOT        : {PX4_ROOT}")
    print(f"  WORLDS_DIR      : {WORLDS_DIR}")
    print(f"  QGC_APPIMAGE    : {QGC_APPIMAGE}")
    print(f"  VEHICLE         : {VEHICLE}")
    print(f"  MAKE_TARGET     : {MAKE_TARGET}")
    print(f"  WORLD           : {WORLD}")
    print(f"  MAVSDK_ADDRESS  : {MAVSDK_ADDRESS}  (port {MAVSDK_PORT})")
    print(f"  QGC_PORT        : {QGC_PORT}")
    print(f"  CONNECT_TIMEOUT : {CONNECT_TIMEOUT_SEC}s")
    print(f"  TELEMETRY_TIMEOUT: {TELEMETRY_TIMEOUT_SEC}s")
    print()

    errors = validate()
    if errors:
        print("[VALIDATION ERRORS]")
        for e in errors:
            print(f"  ✗ {e}")
    else:
        print("[OK] All paths validated successfully.")
