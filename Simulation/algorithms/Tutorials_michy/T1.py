from connect import run as connect
from drone_logger import DroneLogger
import asyncio

async def main():
    drone = await connect()
    if drone is None:
        print("Exiting due to connection failure.")
        return
    
    logger = DroneLogger(drone=drone)

    for i in range(15):
        print(f"Logging telemetry data... ({i+1}/15)")
        await logger.log_all()
        print(f'=======================================================\n')

        await asyncio.sleep(1)

    print("Finished logging telemetry data.")

if __name__ == "__main__":
    asyncio.run(main())