
from connect import run as connect
from drone_logger import DroneLogger
import asyncio
import sys
from mavsdk.offboard import VelocityNedYaw, VelocityBodyYawspeed

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
    await asyncio.sleep(5)
    print("--Square mission")
    try:
        await _square_mission(drone)
    except Exception as e:
        print(f"Error occurred during square mission: {e}")
        await drone.offboard.stop()

    print("-- Landing")
    await drone.action.return_to_launch()

    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("-- Landed and disarmed")
            break
        
    task.cancel()

    await asyncio.gather(task, return_exceptions=True)

    sys.exit()

async def _wait_for_altitude(logger: DroneLogger, target_altitude: float, threshold: float = 0.2):
    while True:
        position = await logger.get_position()
        if position.relative_altitude_m >= target_altitude - threshold:
            print(f"-- Reached target altitude: {position.relative_altitude_m:.2f}m")
            return

async def _square_mission(drone):
    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))  # hover in place
    await asyncio.sleep(1)  # small delay to ensure setpoint is processed
    print("-- Starting square mission")
    await drone.offboard.start()


    moves = [
    ("Go Forward",  (2.5, 0, 0, 90, 8)),
    ("Go Right",    (0, 2.5, 0, 90, 5)) 
    ]


    for label, body in moves:
        print(f"-- {label}")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(body[0], body[1], body[2], body[3]))
        await asyncio.sleep(body[4])

    print("-- Mission complete, stopping offboard")
    await drone.offboard.stop()

if __name__ == "__main__":
    asyncio.run(run())
