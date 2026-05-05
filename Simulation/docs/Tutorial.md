# Simulation Tutorial

Getting started with offboard control using MAVSDK.

## Resources

- [MAVSDK Python API Reference](https://mavsdk.mavlink.io/main/en/python/api_reference/telemetry.html)
- [MAVSDK Python Quickstart](https://mavsdk.mavlink.io/main/en/python/quickstart.html)
- [PX4 Flight Modes Reference](https://docs.px4.io/main/en/getting_started/flight_modes.html)
- [MAVSDK Python Examples](https://github.com/mavlink/MAVSDK-Python/tree/main/examples)

## Tasks

### Task 1: Connect and Read Telemetry

Connect to the drone and log telemetry for 15 seconds:
- Position
- Velocity  
- NED Euler angles

### Task 2: Basic Takeoff and Landing

Arm the drone, take off, hover, and land:
1. Connect to drone
2. Check health (global position and home position OK)
3. Arm
4. Set takeoff altitude to 5m and take off
5. Wait until target altitude is reached (30s timeout)
6. Hover for 10 seconds
7. Land and wait until disarmed

### Task 3: Square Flight in NED Frame (Velocity Commands)

Fly in a square using NED velocity commands:
1. Connect to drone
2. Check health
3. Arm and take off to 5m
4. Switch to offboard mode using `VelocityNedYaw`
5. Fly each leg at 2.5 m/s for 5 seconds — north (0°), east (90°), south (180°), west (270°) — with a hover pause between each leg
6. Return to launch

### Task 4: Square Flight in Body Frame (Velocity Commands)

Fly using body frame velocity commands:
1. Connect to drone
2. Check health
3. Arm and take off to 5m
4. Switch to offboard mode using `VelocityBodyYawspeed`
5. Fly forward then right at 2.5 m/s for 10 seconds each, with a 90°/s yaw rate

**Key learning:** Understand the difference between body and NED frames.

### Task 5: AprilTag Detection and Landing

Detect and land on an AprilTag:
1. Write a camera feed display loop
2. Apply OpenCV tag detection
3. Takeoff above the tag and observe detection
4. Land
