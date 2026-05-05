
from connect import run as connect
from drone_logger import DroneLogger
import asyncio


async def telemetry_loop(logger):
    try:
        while True:
            await logger.log_all()
            print("=======================================================\n")
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        return


async def run():
    drone = await connect()
    if drone is None:
        print("Exiting due to connection failure.")
        return
    logger = DroneLogger(drone=drone)

    print("-- Checking drone health")
    async for health in logger.get_health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    task = asyncio.ensure_future(telemetry_loop(logger))

    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(10)  # small delay to ensure arming is processed
    
    await drone.action.set_takeoff_altitude(5)
    print("-- Taking off")
    await drone.action.takeoff()

    try:
        await asyncio.wait_for(_wait_for_altitude(logger, target_altitude=5), timeout=30)

    except asyncio.TimeoutError:
        print("Timeout reached while waiting for altitude.")

    print("-- Hovering")
    await asyncio.sleep(10)

    print("-- Landing")
    await drone.action.land()

    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("-- Landed and disarmed")
            return
        
    task.cancel()

    await asyncio.gather(task, return_exceptions=True)

async def _wait_for_altitude(logger: DroneLogger, target_altitude: float, threshold: float = 0.2):
    while True:
        position = await logger.get_position()
        if position.relative_altitude_m >= target_altitude - threshold:
            print(f"-- Reached target altitude: {position.relative_altitude_m:.2f}m")
            return
        
if __name__ == "__main__":
    asyncio.run(run())
