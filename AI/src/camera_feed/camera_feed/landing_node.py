import asyncio
import enum
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed

# ── Constants ─────────────────────────────────────────────────────────────────
MAVSDK_ADDRESS    = "udpin://sim:14540"  # resolved at runtime via socket.getaddrinfo
TAKEOFF_ALT_M     = 2.0
DETECT_HZ         = 10
DESCENT_SPEED_MS  = 0.3
KP_XY             = 0.5
MAX_XY_VEL_MS     = 1.0
XY_ALIGN_THRESH_M = 0.15
LAND_ALT_M        = 0.4
POSE_TIMEOUT_SEC  = 1.0
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
def _clamp(value: float, low: float, high: float) -> float:
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
    altitude_m: float,
    shutdown_event: threading.Event,
    shared: dict,
    lock: threading.Lock,
    logger,
) -> None:
    period = 1.0 / DETECT_HZ

    try:
        import socket
        sim_ip = socket.gethostbyname('sim')
        address = f"udpout://{sim_ip}:14540"

        drone = System()
        await drone.connect(system_address=address)

        logger.info(f"Connecting to PX4 at {address}...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                logger.info("Connected.")
                break

        logger.info("Waiting for GPS fix...")

        async def _gps_ready():
            async for health in drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    return

        await asyncio.wait_for(_gps_ready(), timeout=GPS_TIMEOUT_SEC)

        logger.info(f"Arming and taking off to {altitude_m} m...")
        await drone.action.set_takeoff_altitude(altitude_m)
        await drone.action.arm()
        await drone.action.takeoff()
        await _wait_in_air(drone, True, IN_AIR_TIMEOUT, "airborne")
        logger.info("Airborne. Entering offboard mode.")

        seed = PositionNedYaw(0.0, 2.0, -altitude_m, 0.0)
        await drone.offboard.set_position_ned(seed)
        await drone.offboard.start()
        logger.info("Offboard active. Starting control loop.")

        lstate = LandState.SEARCH
        last_log_state = None

        while not shutdown_event.is_set():
            t0 = asyncio.get_running_loop().time()

            alt = await _get_altitude(drone)
            with lock:
                shared["altitude_m"] = alt

            if alt < LAND_ALT_M:
                lstate = LandState.LAND
                logger.info(f"{alt:.2f} m AGL — commanding land.")
                break

            with lock:
                pose = shared["pose"]
                pose_time = shared["pose_time"]

            v_fwd, v_rgt, v_down = 0.0, 0.0, 0.0
            xy_err_m = 0.0

            pose_stale = (pose is None) or ((time.monotonic() - pose_time) > POSE_TIMEOUT_SEC)

            if pose_stale:
                lstate = LandState.SEARCH
            else:
                x_mm = pose.pose.position.x
                y_mm = pose.pose.position.y
                z_mm = pose.pose.position.z

                if z_mm >= 100.0:
                    forward_err_m = -(y_mm / 1000.0)
                    right_err_m   =  (x_mm / 1000.0)
                    xy_err_m = (forward_err_m ** 2 + right_err_m ** 2) ** 0.5

                    v_fwd = _clamp(KP_XY * forward_err_m, -MAX_XY_VEL_MS, MAX_XY_VEL_MS)
                    v_rgt = _clamp(KP_XY * right_err_m,   -MAX_XY_VEL_MS, MAX_XY_VEL_MS)

                    if xy_err_m <= XY_ALIGN_THRESH_M:
                        lstate = LandState.DESCEND
                        v_down = DESCENT_SPEED_MS
                    else:
                        lstate = LandState.ALIGN
                else:
                    lstate = LandState.SEARCH

            with lock:
                shared["lstate"]   = lstate
                shared["xy_err_m"] = xy_err_m

            if lstate != last_log_state:
                logger.info(f"State: {lstate.value}  xy_err={xy_err_m:.3f} m  alt={alt:.2f} m")
                last_log_state = lstate

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(v_fwd, v_rgt, v_down, 0.0)
            )

            elapsed = asyncio.get_running_loop().time() - t0
            await asyncio.sleep(max(0.0, period - elapsed))

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
        logger.info("Landed and disarmed.")

    except Exception as e:
        logger.error(f"Landing error: {e}")
    finally:
        shutdown_event.set()


# ── ROS2 node ─────────────────────────────────────────────────────────────────
class LandingNode(Node):
    def __init__(self, shared: dict, lock: threading.Lock):
        super().__init__('landing_node')

        self.declare_parameter('altitude', TAKEOFF_ALT_M)
        self._altitude = self.get_parameter('altitude').get_parameter_value().double_value

        self._shared = shared
        self._lock   = lock

        self.create_subscription(PoseStamped, '/aruco/pose', self._on_pose, 10)
        self.get_logger().info(f"Subscribed to /aruco/pose. Takeoff altitude: {self._altitude} m")

    def _on_pose(self, msg: PoseStamped):
        with self._lock:
            self._shared["pose"]      = msg
            self._shared["pose_time"] = time.monotonic()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    shared = {
        "pose":      None,
        "pose_time": 0.0,
        "lstate":    LandState.SEARCH,
        "xy_err_m":  0.0,
        "altitude_m": 0.0,
    }
    lock           = threading.Lock()
    shutdown_event = threading.Event()

    node = LandingNode(shared, lock)

    landing_thread = threading.Thread(
        target=lambda: asyncio.run(
            run_landing(node._altitude, shutdown_event, shared, lock, node.get_logger())
        ),
        daemon=True,
        name="landing",
    )
    landing_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_event.set()
        landing_thread.join(timeout=30)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
