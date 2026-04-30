from connect import connect_drone
from logger import DroneLogger
import asyncio
from mavsdk.action import ActionError
from mavsdk import System
import time
from mavsdk.offboard import VelocityNedYaw, OffboardError
import sys

async def main():
    drone: System = await connect_drone()
    logger = DroneLogger(drone=drone)


    print("[PRE-ARM] Waiting for drone to be armable...")
    await asyncio.wait_for(_armable(drone, logger), timeout=5)
    print("[PRE-ARM] Drone is ready to arm.")


    try:
        await drone.action.arm()
        print("[ARM] Arming drone...")
        if await logger.log_armed():
            print("[ARM] Drone armed successfully.")
            await asyncio.sleep(1)
        else:
            raise ActionError
        
    except ActionError as e:
        print(f"[ERROR] Arm failed: {e}")
        sys.exit(1)    


    alt = 5
    await drone.action.set_takeoff_altitude(alt)

    curr = await logger.log_position()
    if abs(curr.relative_altitude_m - alt) < 0.3:
        print("[TAKEOFF] Already at desired height")

    else:
        try:    
            print(f"[TAKEOFF] Taking off to {alt} m AGL...")
            await drone.action.takeoff()
            await asyncio.wait_for(_reached_heigth(alt, logger), timeout=15)

        except asyncio.TimeoutError as e:
            print('[ERROR] Failed to reach takeoff altittude')
            await drone.action.land()
            sys.exit(1)

    print(f"[TAKEOFF] Reached {alt} m.")

    print("[MISSION] flying")
    try:
        await asyncio.wait_for(fly_square(drone=drone), timeout=10)
    except asyncio.TimeoutError as e:
        print('[MISSION] Done Flying')

    await drone.offboard.stop()
    await drone.action.land()  
    await asyncio.sleep(3)
    sys.exit()

async def fly_square(drone: System):
        
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(
            f"Starting offboard mode failed with error code: \
            {error._result.result}"
        )
        print("-- Landing")
        await drone.action.land()
        return
    
    print(" [MISSIION] flying square")

    ned_map ={
        'North': (2,0,0,90),
        'East': (0,2,0,180),
        'South': (-2,0,0,270),
        'West': (0,-2,0,0)
    }
    while True:
        for name, NED in ned_map.items():
            print(f'Flying {name}')

            print(NED)
            await drone.offboard.set_velocity_ned(VelocityNedYaw(NED[0], NED[1], NED[2], NED[3]))
            await asyncio.sleep(2)
    

async def _reached_heigth(target, logger: DroneLogger):
    while True:
        curr = await logger.log_position()

        print(f"[CLIMB] Current altitude: {curr.relative_altitude_m} m")
        if abs(curr.relative_altitude_m - target) < 0.3:
            return
        await asyncio.sleep(1)
        
    
async def _armable(drone: System, logger: DroneLogger):
    while True:
        print(f"[HEALTH] {await logger.log_health()}")

        async for ok in drone.telemetry.health_all_ok():
            if ok:
                return
            break

        
        await asyncio.sleep(1)

asyncio.run(main())