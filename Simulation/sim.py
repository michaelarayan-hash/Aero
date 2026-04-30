"""
Simulation launcher — kills stray processes, then starts PX4 server, QGC, and optionally Gazebo GUI.

Usage:
    python3 sim.py [--world WORLD] [--vehicle VEHICLE] [--gui]

Examples:
    python3 sim.py --world forest --gui
    python3 sim.py --world aruco --vehicle x500_depth --gui
    python3 sim.py --world baylands          # server + QGC only, no GUI
"""

import argparse
import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))
import config

# ── Process patterns to kill when cleaning up stray sims ─────────────────────
STRAY_PATTERNS = [
    "gz sim",
    "ruby.*gz",
    "px4",
    "QGroundControl",
]


def kill_stray():
    """Kill stray simulation processes and wait until they are gone."""
    print("Killing stray simulation processes...")
    killed = 0
    for pattern in STRAY_PATTERNS:
        result = subprocess.run(["pkill", "-f", pattern], capture_output=True)
        if result.returncode == 0:
            killed += 1

    if not killed:
        print("  No stray processes found.")
        return

    print(f"  Sent SIGTERM to processes matching {killed} pattern(s). Waiting...")

    # Poll until all patterns are gone or force-kill after 5s
    deadline = time.time() + 5
    while time.time() < deadline:
        still_alive = [
            p for p in STRAY_PATTERNS
            if subprocess.run(["pgrep", "-f", p], capture_output=True).returncode == 0
        ]
        if not still_alive:
            break
        time.sleep(0.3)
    else:
        # Force-kill anything still alive
        for pattern in STRAY_PATTERNS:
            subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)
        time.sleep(0.5)

    print("  All stray processes stopped.")


def wait_for_ready(proc: subprocess.Popen, timeout: float = 90) -> bool:
    """Stream server output until 'Gazebo world is ready' or timeout.

    Uses select() so the timeout is enforced even if the server stalls
    without producing output or exiting.
    """
    deadline = time.time() + timeout

    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            print(f"[ERROR] Timed out waiting for sim to be ready ({timeout}s).")
            return False

        ready, _, _ = select.select([proc.stdout], [], [], min(remaining, 1.0))

        if ready:
            line = proc.stdout.readline()
            if not line:  # EOF — process closed its stdout
                print("[ERROR] Server process ended unexpectedly.")
                return False
            sys.stdout.write(f"  [server] {line}")
            sys.stdout.flush()
            if "Gazebo world is ready" in line:
                return True

        if proc.poll() is not None:
            # Drain remaining output
            for line in proc.stdout:
                sys.stdout.write(f"  [server] {line}")
            print("[ERROR] Server process exited early.")
            return False


def _gz_clean_env() -> dict:
    """Return a minimal env for gz sim -g, stripping snap library paths.

    When launched from the VS Code snap, LD_LIBRARY_PATH contains snap paths
    that override the system glibc and crash gz with a symbol lookup error.
    """
    clean = {
        k: os.environ[k]
        for k in (
            "HOME", "USER", "DISPLAY", "WAYLAND_DISPLAY",
            "XDG_RUNTIME_DIR", "DBUS_SESSION_BUS_ADDRESS",
            "XAUTHORITY", "LANG", "LC_ALL",
        )
        if k in os.environ
    }
    clean["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    return clean


def main():
    parser = argparse.ArgumentParser(description="Launch PX4 SITL simulation stack")
    parser.add_argument("--world", default=config.WORLD,
                        help=f"Gazebo world name (default: {config.WORLD})")
    parser.add_argument("--vehicle", default=config.VEHICLE,
                        help=f"Vehicle model (default: {config.VEHICLE})")
    parser.add_argument("--gui", action="store_true",
                        help="Open Gazebo GUI (gz sim -g)")
    parser.add_argument("--camera", action="store_true",
                        help="Open simulated camera feed with ArUco detection")
    args = parser.parse_args()

    launch_sh = ROOT / "launch_sim.sh"
    if not launch_sh.exists():
        print(f"[ERROR] launch_sim.sh not found at {launch_sh}")
        sys.exit(1)

    # ── Track spawned processes for cleanup on Ctrl-C ────────────────────────
    procs: list[subprocess.Popen] = []

    def shutdown(sig=None, frame=None):
        print("\nShutting down...")
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ── Step 1: kill stray processes ─────────────────────────────────────────
    kill_stray()

    # ── Step 2: launch PX4 SITL server ───────────────────────────────────────
    print(f"\nStarting PX4 SITL server — world={args.world}  vehicle={args.vehicle}")
    # Automatically add the local models/ directory to GZ_SIM_RESOURCE_PATH so
    # Gazebo can resolve model:// URIs in custom world files without the user
    # having to set this env var manually before every launch.
    server_env = os.environ.copy()
    models_dir = str(ROOT / "models")
    existing_resource_path = server_env.get("GZ_SIM_RESOURCE_PATH", "")
    server_env["GZ_SIM_RESOURCE_PATH"] = (
        f"{models_dir}:{existing_resource_path}" if existing_resource_path else models_dir
    )
    server = subprocess.Popen(
        ["bash", str(launch_sh), args.world, args.vehicle],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        cwd=ROOT,
        env=server_env,
    )
    procs.append(server)

    if not wait_for_ready(server, timeout=90):
        shutdown()

    print("[OK] Simulation is ready.\n")

    # ── Step 3: launch QGroundControl ────────────────────────────────────────
    qgc = config.QGC_APPIMAGE
    qgc_log = Path("/tmp/qgc.log")
    if qgc.exists():
        print(f"Starting QGroundControl ({qgc})... (stderr → {qgc_log})")
        qgc_log_fh = open(qgc_log, "w")
        procs.append(subprocess.Popen(
            [str(qgc)], stdout=qgc_log_fh, stderr=subprocess.STDOUT
        ))
    else:
        print(f"[WARN] QGroundControl not found at {qgc} — skipping.")

    # ── Step 4: optionally launch Gazebo GUI ─────────────────────────────────
    if args.gui:
        time.sleep(2)  # let server finish initialising before GUI connects
        print("Starting Gazebo GUI (gz sim -g)...")
        gui_proc = subprocess.Popen(
            ["gz", "sim", "-g"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=_gz_clean_env(),
        )
        procs.append(gui_proc)
        time.sleep(2)
        if gui_proc.poll() is not None:
            out, _ = gui_proc.communicate()
            print(f"[WARN] Gazebo GUI exited early:\n{out}")
        else:
            print("[OK] Gazebo GUI running.")

    # ── Step 5: optionally launch camera feed ────────────────────────────────
    if args.camera:
        print("Starting camera feed (ArUco detection)...")
        cam_proc = subprocess.Popen(
            [sys.executable, str(ROOT / "algorithms" / "camera_feed.py"),
             "--world", args.world, "--vehicle", args.vehicle],
            cwd=ROOT,
        )
        procs.append(cam_proc)

    # ── Step 6: keep alive, stream remaining server output ───────────────────
    print(f"\nAll processes running.  Ctrl-C to stop everything.\n")
    print(f"  MAVSDK (offboard scripts) : udp://:{config.MAVSDK_PORT}")
    print(f"  QGroundControl            : UDP {config.QGC_PORT} (auto-detected)")
    if args.gui:
        print(f"  Gazebo GUI                : running")
    if args.camera:
        print(f"  Camera feed (ArUco)       : running  [Q=quit, V=log distance]")
    print()

    for line in server.stdout:
        sys.stdout.write(f"  [server] {line}")
        sys.stdout.flush()

    server.wait()
    print("\n[INFO] Server process ended.")
    shutdown()


if __name__ == "__main__":
    main()
