"""
Precision auto-landing demo — aligns over an ArUco marker and descends.

Usage:
    cd ~/Aero/Simulation && source .venv/bin/activate

    # Terminal 1 — start sim:
    GZ_SIM_RESOURCE_PATH=$HOME/Aero/Simulation/models \\
    PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds \\
    python3 sim.py --world tag_demo --gui

    # Terminal 2 — run landing:
    python3 algorithms/auto_land.py --world tag_demo

Controls:
    Q  in OpenCV window — graceful shutdown (lands drone)
"""
import argparse
import asyncio
import enum
import sys
import threading
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).parent.parent))

import config
from connection_test import connect_drone_ctx
from algorithms.camera_feed import SimCamera, load_calibration
from algorithms.tag_detector import TagDetector

from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed

# ── Constants ─────────────────────────────────────────────────────────────────
VEHICLE           = "x500_mono_cam_down"
MARKER_SIZE_MM    = 500.0     # large ground markers in tag_demo (real camera: 100 mm)
ARUCO_DICTS       = ["4x4_50"]
APRILTAG_FAMILIES = []
TAKEOFF_ALT_M     = 2.0
DETECT_HZ         = 10
DESCENT_SPEED_MS  = 0.3       # m/s downward
KP_XY             = 0.5       # proportional gain for XY centering
MAX_XY_VEL_MS     = 1.0       # velocity clamp
XY_ALIGN_THRESH_M = 0.15      # centering radius before descent begins
LAND_ALT_M        = 0.4       # altitude at which action.land() is called
GPS_TIMEOUT_SEC   = 60
IN_AIR_TIMEOUT    = 30
LAND_TIMEOUT      = 60


# ── State machine ─────────────────────────────────────────────────────────────
class LandState(enum.Enum):
    SEARCH  = "SEARCH"
    ALIGN   = "ALIGN"
    DESCEND = "DESCEND"
    LAND    = "LAND"


# ── Helpers ───────────────────────────────────────────────────────────────────
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


async def _wait_in_air(drone, state: bool, timeout: float, label: str) -> None:
    async def _watch():
        async for in_air in drone.telemetry.in_air():
            if in_air == state:
                return
    try:
        await asyncio.wait_for(_watch(), timeout=timeout)
    except asyncio.TimeoutError:
        raise TimeoutError(f"Timed out waiting for {label} after {timeout}s")


async def _get_altitude(drone) -> float:
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m


# ── Flight coroutine ──────────────────────────────────────────────────────────
async def run_landing(
    args: argparse.Namespace,
    shutdown_event: threading.Event,
    shared: dict,
    lock: threading.Lock,
) -> None:
    period = 1.0 / DETECT_HZ

    try:
        async with connect_drone_ctx() as drone:
            print("[landing] Waiting for GPS fix...")

            async def _gps_ready():
                async for health in drone.telemetry.health():
                    if health.is_global_position_ok and health.is_home_position_ok:
                        return

            await asyncio.wait_for(_gps_ready(), timeout=GPS_TIMEOUT_SEC)

            print(f"[landing] Arming and taking off to {args.altitude} m...")
            await drone.action.set_takeoff_altitude(args.altitude)
            await drone.action.arm()
            await drone.action.takeoff()
            await _wait_in_air(drone, True, IN_AIR_TIMEOUT, "airborne")
            print("[landing] Airborne. Entering offboard mode.")

            seed = PositionNedYaw(0.0, 0.0, -args.altitude, 0.0)
            await drone.offboard.set_position_ned(seed)
            await drone.offboard.start()
            print("[landing] Offboard active. Starting control loop.")

            lstate = LandState.SEARCH

            while not shutdown_event.is_set():
                t0 = asyncio.get_running_loop().time()

                altitude_m = await _get_altitude(drone)
                with lock:
                    shared["altitude_m"] = altitude_m

                if altitude_m < LAND_ALT_M:
                    lstate = LandState.LAND
                    with lock:
                        shared["lstate"] = lstate
                    print(f"[landing] {altitude_m:.2f} m AGL — commanding land.")
                    break

                with lock:
                    detections = list(shared["detections"])

                v_fwd, v_rgt, v_down = 0.0, 0.0, 0.0
                xy_err_m = 0.0

                if not detections:
                    lstate = LandState.SEARCH
                else:
                    det = min(detections, key=lambda d: d.distance_mm)

                    if det.tvec[2][0] >= 100.0:
                        # Camera frame → body frame
                        # tvec[1] down-in-image = drone backward → negate for forward
                        forward_err_m = -(det.tvec[1][0] / 1000.0)
                        right_err_m   =  (det.tvec[0][0] / 1000.0)
                        xy_err_m = (forward_err_m ** 2 + right_err_m ** 2) ** 0.5

                        v_fwd = clamp(KP_XY * forward_err_m, -MAX_XY_VEL_MS, MAX_XY_VEL_MS)
                        v_rgt = clamp(KP_XY * right_err_m,   -MAX_XY_VEL_MS, MAX_XY_VEL_MS)

                        if xy_err_m <= XY_ALIGN_THRESH_M:
                            lstate = LandState.DESCEND
                            v_down = DESCENT_SPEED_MS
                        else:
                            lstate = LandState.ALIGN
                            v_down = 0.0
                    else:
                        lstate = LandState.SEARCH

                with lock:
                    shared["lstate"]   = lstate
                    shared["xy_err_m"] = xy_err_m

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(v_fwd, v_rgt, v_down, 0.0)
                )

                elapsed = asyncio.get_running_loop().time() - t0
                await asyncio.sleep(max(0.0, period - elapsed))

            # Landing sequence
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
            print("[landing] Landed and disarmed.")

    except Exception as e:
        print(f"[landing] ERROR: {e}")
    finally:
        shutdown_event.set()


# ── Display loop ──────────────────────────────────────────────────────────────
def display_loop(
    cam: SimCamera,
    detector: TagDetector,
    shutdown_event: threading.Event,
    shared: dict,
    lock: threading.Lock,
) -> None:
    cv2.namedWindow("Auto Land", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Auto Land", 1280, 960)

    frame_budget_ms = 1000.0 / 30

    _state_colours = {
        LandState.SEARCH:  (0, 100, 255),
        LandState.ALIGN:   (0, 220, 255),
        LandState.DESCEND: (50, 200, 50),
        LandState.LAND:    (255, 80, 80),
    }

    while not shutdown_event.is_set():
        t0 = time.monotonic()
        ok, frame = cam.read()
        if not ok:
            time.sleep(0.01)
            continue

        detections = detector.detect(frame)

        with lock:
            shared["detections"] = detections
            lstate     = shared["lstate"]
            xy_err_m   = shared["xy_err_m"]
            altitude_m = shared["altitude_m"]

        detector.draw(frame, detections)

        colour = _state_colours.get(lstate, (200, 200, 200))
        cv2.putText(frame, f"State: {lstate.value}",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 2)
        cv2.putText(frame, f"XY err: {xy_err_m:.3f} m  (thresh {XY_ALIGN_THRESH_M} m)",
                    (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, f"Alt: {altitude_m:.2f} m AGL  (land < {LAND_ALT_M} m)",
                    (12, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, f"Detections: {len(detections)}",
                    (12, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        proc_ms = (time.monotonic() - t0) * 1000
        lag_col = (0, 60, 255) if proc_ms > frame_budget_ms else (120, 120, 120)
        cv2.putText(frame, f"{proc_ms:.1f}ms",
                    (12, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, lag_col, 1)

        cv2.imshow("Auto Land", frame)
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
    parser.add_argument("--altitude", type=float, default=TAKEOFF_ALT_M,
                        help=f"Takeoff altitude in metres AGL (default: {TAKEOFF_ALT_M})")
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
              f"PX4_GZ_WORLDS=$HOME/Aero/Simulation/worlds "
              f"python3 sim.py --world {args.world} --gui")
        cam.stop()
        sys.exit(1)

    shared = {
        "detections": [],
        "lstate":     LandState.SEARCH,
        "xy_err_m":   0.0,
        "altitude_m": 0.0,
    }
    lock           = threading.Lock()
    shutdown_event = threading.Event()

    landing_thread = threading.Thread(
        target=lambda: asyncio.run(
            run_landing(args, shutdown_event, shared, lock)
        ),
        daemon=True,
        name="landing",
    )
    landing_thread.start()

    display_loop(cam, detector, shutdown_event, shared, lock)

    shutdown_event.set()
    landing_thread.join(timeout=30)
    if landing_thread.is_alive():
        print("[WARN] landing thread did not exit within 30s")
    cam.stop()
    print("[main] Done.")


if __name__ == "__main__":
    main()
