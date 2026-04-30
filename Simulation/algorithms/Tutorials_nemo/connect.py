import sys
from pathlib import Path
import asyncio
from mavsdk import System
import time

sys.path.insert(0, str(Path(__file__).parent.parent.parent))
import config

async def connect_drone(timeout=30):
    print(f"Connecting to drone at {config.MAVSDK_ADDRESS}")
    drone = System()
    await drone.connect(config.MAVSDK_ADDRESS)
    try:
        await asyncio.wait_for(_wait_for_connection(drone), timeout=timeout)
        print("Connected to drone!")

    except asyncio.TimeoutError:
        raise TimeoutError(
            f"No heartbeat from PX4 within {timeout}s on {config.MAVSDK_ADDRESS}.\n"
            "  Is the simulation running?  Try: ./launch_sim.sh"
        )
    
    return drone

async def _wait_for_connection(drone: "System"):
    async for state in drone.core.connection_state():
        if state.is_connected:
            return
