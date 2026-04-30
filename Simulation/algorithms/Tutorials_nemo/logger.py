import sys
from pathlib import Path
import asyncio
from mavsdk import System
from mavsdk.telemetry import Position, VelocityNed, EulerAngle, Health

sys.path.insert(0, str(Path(__file__).parent.parent.parent))
import config

class DroneLogger:
    def __init__(self, drone):
        self.drone: System = drone

    async def log_position(self) -> Position:
        async for pos in self.drone.telemetry.position():
            return pos

    async def log_velocity_ned(self) -> VelocityNed:
        async for vel in self.drone.telemetry.velocity_ned():
            return vel

    async def log_attitude_euler(self) -> EulerAngle:
        async for att in self.drone.telemetry.attitude_euler():
            return att

    async def log_health(self) -> Health:
        async for health in self.drone.telemetry.health():
            return health

    async def log_armed(self) -> bool:
        async for armed in self.drone.telemetry.armed():
            return armed


