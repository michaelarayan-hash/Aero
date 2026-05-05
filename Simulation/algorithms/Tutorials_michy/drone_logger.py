import sys
import asyncio
from pathlib import Path
from mavsdk import System
from mavsdk.telemetry import Battery, GpsInfo, Position, Health, VelocityNed

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import config


class DroneLogger:
    def __init__(self, drone: System):
        self.drone = drone

    async def get_battery(self) -> Battery:
        async for battery in self.drone.telemetry.battery():
            return battery

    async def get_gps(self) -> GpsInfo:
        async for gps in self.drone.telemetry.gps_info():
            return gps

    async def get_position(self) -> Position:
        async for pos in self.drone.telemetry.position():
            return pos
    
    async def get_health(self) -> Health:
        async for h in self.drone.telemetry.health():
            yield h
        
    async def get_velocity_ned(self) -> VelocityNed:
        async for vel in self.drone.telemetry.velocity_ned():
            return vel
        
    async def log_all(self) -> None:
        async def _one_health():
            async for h in self.get_health():
                return h

        battery, gps, position, health, velocity_ned = await asyncio.gather(
            self.get_battery(),
            self.get_gps(),
            self.get_position(),
            _one_health(),
            self.get_velocity_ned()
        )
        print(f"[Battery]  {battery.remaining_percent * 100:.1f}%  {battery.voltage_v:.2f}V")
        print(f"[Position] lat={position.latitude_deg:.6f}  lon={position.longitude_deg:.6f}"
              f"  alt={position.relative_altitude_m:.2f}m AGL")
        print(f"[Velocity] N={velocity_ned.north_m_s:.2f}  E={velocity_ned.east_m_s:.2f}  D={velocity_ned.down_m_s:.2f}")