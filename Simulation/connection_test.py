"""
PX4/MAVSDK connection test — standalone tester AND importable library.

Standalone:
    python3 connection_test.py [--address udp://:14540]

Importable:
    from connection_test import connect_drone, read_telemetry, connect_drone_ctx
"""
import asyncio
import sys
from contextlib import asynccontextmanager
from pathlib import Path

# Allow running from any working directory
sys.path.insert(0, str(Path(__file__).parent))
import config

try:
    from mavsdk import System
except ImportError:
    print("[ERROR] mavsdk not installed. Run: pip install mavsdk>=2.0.0")
    sys.exit(1)


# ── Public API ────────────────────────────────────────────────────────────────

async def connect_drone(
    address: str = config.MAVSDK_ADDRESS,
    timeout: float = config.CONNECT_TIMEOUT_SEC,
) -> "System":
    """Connect to PX4 and wait for heartbeat.

    Args:
        address: MAVLink connection string, e.g. "udp://:14540"
        timeout: Seconds to wait for heartbeat before raising TimeoutError

    Returns:
        Connected mavsdk.System instance

    Raises:
        TimeoutError: If no heartbeat received within timeout
    """
    drone = System()
    await drone.connect(system_address=address)

    try:
        await asyncio.wait_for(_wait_for_connection(drone), timeout=timeout)
    except asyncio.TimeoutError:
        raise TimeoutError(
            f"No heartbeat from PX4 within {timeout}s on {address}.\n"
            "  Is the simulation running?  Try: ./launch_sim.sh"
        )
    return drone


async def _wait_for_connection(drone: "System") -> None:
    async for state in drone.core.connection_state():
        if state.is_connected:
            return


async def read_telemetry(drone: "System") -> dict:
    """Read first value from each telemetry stream with per-field timeout.

    Returns a dict with keys: gps, battery, armed, flight_mode, health.
    Fields that time out return None rather than hanging.
    """
    timeout = config.TELEMETRY_TIMEOUT_SEC

    async def _first(aiter, field_name):
        try:
            async for val in aiter:
                return val
        except Exception as exc:
            print(f"  [warn] {field_name}: {exc}")
            return None

    async def _timed(coro, field_name):
        try:
            return await asyncio.wait_for(coro, timeout=timeout)
        except asyncio.TimeoutError:
            print(f"  [warn] {field_name}: timed out after {timeout}s")
            return None

    gps, battery, armed, flight_mode, health = await asyncio.gather(
        _timed(_first(drone.telemetry.position(), "gps"), "gps"),
        _timed(_first(drone.telemetry.battery(), "battery"), "battery"),
        _timed(_first(drone.telemetry.armed(), "armed"), "armed"),
        _timed(_first(drone.telemetry.flight_mode(), "flight_mode"), "flight_mode"),
        _timed(_first(drone.telemetry.health(), "health"), "health"),
    )

    return {
        "gps": gps,
        "battery": battery,
        "armed": armed,
        "flight_mode": flight_mode,
        "health": health,
    }


@asynccontextmanager
async def connect_drone_ctx(
    address: str = config.MAVSDK_ADDRESS,
    timeout: float = config.CONNECT_TIMEOUT_SEC,
):
    """Async context manager wrapping connect_drone.

    Usage:
        async with connect_drone_ctx() as drone:
            telem = await read_telemetry(drone)
    """
    drone = await connect_drone(address, timeout)
    try:
        yield drone
    finally:
        pass  # mavsdk System has no explicit close


# ── Pretty printer ────────────────────────────────────────────────────────────

def _fmt_telemetry(t: dict) -> str:
    lines = []

    gps = t.get("gps")
    if gps is not None:
        lines += [
            f"  GPS lat          : {gps.latitude_deg:.6f}°",
            f"  GPS lon          : {gps.longitude_deg:.6f}°",
            f"  Altitude (AMSL)  : {gps.absolute_altitude_m:.2f} m",
            f"  Altitude (AGL)   : {gps.relative_altitude_m:.2f} m",
        ]
    else:
        lines.append("  GPS              : unavailable")

    battery = t.get("battery")
    if battery is not None:
        lines.append(
            f"  Battery          : {battery.remaining_percent:.1f}%"
            f"  ({battery.voltage_v:.2f} V)"
        )
    else:
        lines.append("  Battery          : unavailable")

    armed = t.get("armed")
    lines.append(f"  Armed            : {armed}")

    fm = t.get("flight_mode")
    lines.append(f"  Flight mode      : {fm}")

    health = t.get("health")
    if health is not None:
        lines += [
            f"  Gyro calibrated  : {health.is_gyrometer_calibration_ok}",
            f"  Accel calibrated : {health.is_accelerometer_calibration_ok}",
            f"  Mag calibrated   : {health.is_magnetometer_calibration_ok}",
            f"  GPS fix          : {health.is_global_position_ok}",
            f"  Home position    : {health.is_home_position_ok}",
            f"  Armable          : {health.is_armable}",
        ]
    else:
        lines.append("  Health           : unavailable")

    return "\n".join(lines)


# ── Standalone entry point ────────────────────────────────────────────────────

async def _main():
    import argparse

    parser = argparse.ArgumentParser(description="Test MAVSDK connection to PX4 SITL")
    parser.add_argument(
        "--address",
        default=config.MAVSDK_ADDRESS,
        help=f"MAVLink address (default: {config.MAVSDK_ADDRESS})",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=config.CONNECT_TIMEOUT_SEC,
        help=f"Connection timeout in seconds (default: {config.CONNECT_TIMEOUT_SEC})",
    )
    args = parser.parse_args()

    print(f"Connecting to PX4 at {args.address} (timeout {args.timeout}s)...")

    try:
        drone = await connect_drone(args.address, args.timeout)
    except TimeoutError as exc:
        print(f"\n[FAILED] {exc}")
        sys.exit(1)

    print("[OK] Heartbeat received — drone connected.\n")
    print("Reading telemetry...")

    telem = await read_telemetry(drone)

    print("\n=== Telemetry Snapshot ===")
    print(_fmt_telemetry(telem))

    print("\n=== QGroundControl ===")
    print(f"  Connect QGC to UDP port {config.QGC_PORT} (broadcast, auto-detected)")
    print(f"  Or launch: {config.QGC_APPIMAGE}")


if __name__ == "__main__":
    asyncio.run(_main())
