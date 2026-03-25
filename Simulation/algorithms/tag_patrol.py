"""
Tag patrol demo — autonomous waypoint circuit with ArUco + AprilTag detection.

Usage:
    cd ~/Aero/Simulation && source .venv/bin/activate

    # Start sim first (separate terminal):
    GZ_SIM_RESOURCE_PATH=$HOME/Aero/Simulation/models \\
    PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds \\
    python3 sim.py --world tag_demo

    # Then run patrol:
    python3 algorithms/tag_patrol.py --world tag_demo

Controls:
    Q  in OpenCV window — graceful shutdown (lands drone)
"""
import asyncio
import sys
import threading
import time
from pathlib import Path

import argparse
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

import config
from connection_test import connect_drone_ctx
from algorithms.camera_feed import SimCamera, load_calibration
from algorithms.tag_detector import TagDetector

from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw

# ── Constants ─────────────────────────────────────────────────────────────────
VEHICLE          = "x500_mono_cam_down"
MARKER_SIZE_MM   = 500.0
ARUCO_DICTS      = ["4x4_50"]
APRILTAG_FAMILIES = ["tag36h11"]
SETPOINT_HZ      = 10
ACCEPTANCE_M     = 0.5
HOVER_SEC        = 2.0
IN_AIR_TIMEOUT   = 30
LAND_TIMEOUT     = 60


# ── Waypoints ─────────────────────────────────────────────────────────────────

def make_waypoints(altitude_m: float, spacing_m: float) -> list[tuple]:
    """Square circuit at `altitude_m` AGL covering the 2×3 marker grid."""
    s, d = spacing_m, -altitude_m
    return [
        ( s,  s, d),   # NE
        ( s, -s, d),   # NW
        (-s, -s, d),   # SW
        (-s,  s, d),   # SE
    ]


# ── Flight helpers ────────────────────────────────────────────────────────────

async def _wait_in_air(drone, state: bool, timeout: float, label: str) -> None:
    async def _watch():
        async for in_air in drone.telemetry.in_air():
            if in_air == state:
                return
    try:
        await asyncio.wait_for(_watch(), timeout=timeout)
    except asyncio.TimeoutError:
        raise TimeoutError(f"Timed out waiting for {label} after {timeout}s")


async def _get_ned_position(drone):
    """Return (north_m, east_m, down_m) from the first telemetry sample."""
    async for pv in drone.telemetry.position_velocity_ned():
        return pv.position.north_m, pv.position.east_m, pv.position.down_m


async def fly_to_ned(
    drone, n: float, e: float, d: float, shutdown_event: threading.Event
) -> None:
    """Stream NED setpoint at SETPOINT_HZ until within ACCEPTANCE_M or shutdown."""
    target = PositionNedYaw(n, e, d, 0.0)
    period = 1.0 / SETPOINT_HZ
    while not shutdown_event.is_set():
        await drone.offboard.set_position_ned(target)
        try:
            cn, ce, cd = await asyncio.wait_for(_get_ned_position(drone), timeout=2.0)
        except asyncio.TimeoutError:
            await asyncio.sleep(period)
            continue
        dist = ((cn - n) ** 2 + (ce - e) ** 2 + (cd - d) ** 2) ** 0.5
        if dist < ACCEPTANCE_M:
            return
        await asyncio.sleep(period)


# ── Patrol coroutine ──────────────────────────────────────────────────────────

async def run_patrol(
    args: argparse.Namespace,
    shutdown_event: threading.Event,
    waypoint_idx: list,
) -> None:
    waypoints = make_waypoints(args.altitude, args.spacing)

    try:
        async with connect_drone_ctx() as drone:
            print("[patrol] Waiting for GPS fix...")
            async for health in drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    break

            print(f"[patrol] Arming + taking off to {args.altitude}m AGL...")
            await drone.action.set_takeoff_altitude(args.altitude)
            await drone.action.arm()
            await drone.action.takeoff()
            await _wait_in_air(drone, True, IN_AIR_TIMEOUT, "airborne")
            print("[patrol] Airborne. Entering offboard mode.")

            hold = PositionNedYaw(0.0, 0.0, -args.altitude, 0.0)
            await drone.offboard.set_position_ned(hold)
            await drone.offboard.start()

            for i, (n, e, d) in enumerate(waypoints):
                if shutdown_event.is_set():
                    break
                waypoint_idx[0] = i
                print(f"[patrol] Waypoint {i + 1}/{len(waypoints)}: N={n} E={e} D={d}")
                await fly_to_ned(drone, n, e, d, shutdown_event)
                await asyncio.sleep(HOVER_SEC)

            print("[patrol] Circuit complete. Landing...")
            try:
                await drone.offboard.stop()
            except OffboardError:
                pass
            await drone.action.land()
            await _wait_in_air(drone, False, LAND_TIMEOUT, "landed")
            try:
                await drone.action.disarm()
            except ActionError:
                pass
            print("[patrol] Landed and disarmed.")

    except Exception as e:
        print(f"[patrol] ERROR: {e}")
    finally:
        shutdown_event.set()


# ── Display loop ──────────────────────────────────────────────────────────────

def display_loop(
    cam: SimCamera,
    detector: TagDetector,
    shutdown_event: threading.Event,
    waypoint_idx: list,
    n_waypoints: int,
) -> None:
    cv2.namedWindow("Tag Patrol", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Tag Patrol", 1280, 960)

    frame_budget_ms = 1000.0 / 30

    while not shutdown_event.is_set():
        t0 = time.monotonic()
        ok, frame = cam.read()
        if not ok:
            time.sleep(0.01)
            continue

        detections = detector.detect(frame)
        detector.draw(frame, detections)

        wp = waypoint_idx[0] + 1
        cv2.putText(frame, f"Waypoint {wp}/{n_waypoints}",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(frame, f"Detections this frame: {len(detections)}",
                    (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        elapsed_ms = (time.monotonic() - t0) * 1000
        lag_col = (0, 60, 255) if elapsed_ms > frame_budget_ms else (120, 120, 120)
        cv2.putText(frame, f"{elapsed_ms:.1f}ms",
                    (12, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, lag_col, 1)

        cv2.imshow("Tag Patrol", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("[display] Q pressed — shutting down.")
            shutdown_event.set()

    cv2.destroyAllWindows()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--world",    default="tag_demo",
                        help="Gazebo world name (default: tag_demo)")
    parser.add_argument("--altitude", type=float, default=3.0,
                        help="Patrol altitude in metres AGL (default: 3.0)")
    parser.add_argument("--spacing",  type=float, default=7.0,
                        help="Waypoint square half-side in metres (default: 7.0)")
    parser.add_argument("--calib",    default="calibration.npz",
                        help="Path to calibration.npz (default: calibration.npz)")
    args = parser.parse_args()

    mtx, dist = load_calibration(args.calib)

    detector = TagDetector(
        marker_size_mm=MARKER_SIZE_MM,
        camera_matrix=mtx,
        dist_coeffs=dist,
        aruco_dicts=ARUCO_DICTS,
        apriltag_families=APRILTAG_FAMILIES,
    )

    cam = SimCamera(world=args.world, vehicle=VEHICLE)
    cam.start()
    print(f"[main] Waiting for first frame from {cam.topic} ...")
    if not cam.wait_for_frame(timeout=15):
        print("[ERROR] No frames received — is the simulation running?")
        print(f"  Start with: GZ_SIM_RESOURCE_PATH=$HOME/Aero/Simulation/models "
              f"PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds python3 sim.py --world {args.world}")
        cam.stop()
        sys.exit(1)

    shutdown_event = threading.Event()
    waypoint_idx = [0]
    waypoints = make_waypoints(args.altitude, args.spacing)

    patrol_thread = threading.Thread(
        target=lambda: asyncio.run(
            run_patrol(args, shutdown_event, waypoint_idx)
        ),
        daemon=True,
        name="patrol",
    )
    patrol_thread.start()

    display_loop(cam, detector, shutdown_event, waypoint_idx, len(waypoints))

    patrol_thread.join(timeout=30)
    cam.stop()
    print("[main] Done.")


if __name__ == "__main__":
    main()
