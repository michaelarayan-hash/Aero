import asyncio
import sys
import logging
from pathlib import Path
from mavsdk import System

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import config
from drone_logger import DroneLogger

logging.basicConfig(level=logging.INFO)

async def run():
    drone = System()
    await drone.connect(config.MAVSDK_ADDRESS)
    logging.info(f"Connecting to drone at {config.MAVSDK_ADDRESS}...")

    try:
        await asyncio.wait_for(_wait_for_connection(drone), timeout=config.CONNECT_TIMEOUT_SEC)
        logging.info("Drone connected!")
        return drone
    
    except asyncio.TimeoutError:
        logging.error("Failed to connect to drone within the timeout period.")

async def _wait_for_connection(drone: "System"):
    async for state in drone.core.connection_state():
        if state.is_connected:
            return
        
if __name__ == "__main__":
    asyncio.run(run())