import connect
from logger import DroneLogger
import asyncio
import time

async def main():
    drone =  await connect.connect_drone()
    logger = DroneLogger(drone=drone)

    for i in range(15):
        asyncio.gather()
        pos = await logger.log_position()
        att = await logger.log_attitude_euler()
        vel = await logger.log_velocity_ned()

        
        print(f'POS: {pos}\n')
        print(f'Attitude: {att}\n')
        print(f'Velocity:{vel}\n')
        print(f'=======================================================\n')

        await asyncio.sleep(1)



asyncio.run(main())