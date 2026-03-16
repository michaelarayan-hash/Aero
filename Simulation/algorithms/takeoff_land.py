"""
Takeoff and landing test.

Usage:
    cd Simulation && source .venv/bin/activate
    python3 algorithms/takeoff_land.py [--altitude 5.0]

Requires PX4 SITL to be running:
    python3 sim.py --world default
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import config
from connection_test import connect_drone_ctx, read_telemetry

from mavsdk.action import ActionError

TAKEOFF_ALTITUDE_M = 5.0
IN_AIR_TIMEOUT_SEC = 30   # max wait to become airborne after takeoff command
LAND_TIMEOUT_SEC = 60     # max wait to touch down after land command


async def _wait_in_air(drone, state: bool, timeout: float, label: str) -> None:
    """Wait until drone.telemetry.in_air() == state, with timeout."""
    async def _watch():
        async for in_air in drone.telemetry.in_air():
            if in_air == state:
                return
    try:
        await asyncio.wait_for(_watch(), timeout=timeout)
    except asyncio.TimeoutError:
        raise TimeoutError(f"Timed out waiting for {label} after {timeout}s")


async def main(altitude: float = TAKEOFF_ALTITUDE_M):
    print(f"Connecting to {config.MAVSDK_ADDRESS}...")

    async with connect_drone_ctx() as drone:
        await drone.action.set_takeoff_altitude(altitude)

        telem = await read_telemetry(drone)
        print(f"Pre-flight: armed={telem['armed']}  mode={telem['flight_mode']}")

        health = telem["health"]
        if health and not health.is_armable:
            print("[ERROR] Drone is not armable. Check health:")
            print(f"  GPS fix       : {health.is_global_position_ok}")
            print(f"  Home position : {health.is_home_position_ok}")
            print(f"  Gyro cal      : {health.is_gyrometer_calibration_ok}")
            sys.exit(1)

        print("Arming...")
        try:
            await drone.action.arm()
        except ActionError as e:
            print(f"[ERROR] Arm failed: {e}")
            sys.exit(1)

        print(f"Taking off to {altitude} m AGL...")
        try:
            await drone.action.takeoff()
        except ActionError as e:
            print(f"[ERROR] Takeoff failed: {e}")
            sys.exit(1)

        try:
            await _wait_in_air(drone, True, IN_AIR_TIMEOUT_SEC, "airborne")
        except TimeoutError as e:
            print(f"[ERROR] {e}")
            sys.exit(1)
        print("Airborne.")

        # Hold altitude briefly
        await asyncio.sleep(5)

        async for pos in drone.telemetry.position():
            print(f"Hovering at {pos.relative_altitude_m:.2f} m AGL")
            break

        print("Landing...")
        try:
            await drone.action.land()
        except ActionError as e:
            print(f"[ERROR] Land failed: {e}")
            sys.exit(1)

        try:
            await _wait_in_air(drone, False, LAND_TIMEOUT_SEC, "landed")
        except TimeoutError as e:
            print(f"[ERROR] {e}")
            sys.exit(1)
        print("Landed.")

        try:
            await drone.action.disarm()
        except ActionError:
            pass  # PX4 auto-disarms after landing; ignore if already disarmed
        print("Disarmed. Done.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Takeoff and landing test")
    parser.add_argument(
        "--altitude",
        type=float,
        default=TAKEOFF_ALTITUDE_M,
        help=f"Target takeoff altitude in metres AGL (default: {TAKEOFF_ALTITUDE_M})",
    )
    args = parser.parse_args()
    asyncio.run(main(args.altitude))
