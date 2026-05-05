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

### Task 2: Square Flight in NED Frame (Velocity Commands)

Fly in a square using NED velocity commands:
1. Connect to drone
2. Arm
3. Takeoff
4. Switch to offboard mode (using `offboard_velocity_ned`)
5. Fly 2 m/s in each direction (north, east, south, west) for 2 seconds per leg
6. Total flight time: 20 seconds
7. Land

### Task 3: Square Flight in Body Frame (GPS Coordinates)

Fly in a square using body frame and GPS coordinates.

**Key learning:** Understand the difference between body and NED frames.

### Task 4: AprilTag Detection and Landing

Detect and land on an AprilTag:
1. Write a camera feed display loop
2. Apply OpenCV tag detection
3. Takeoff above the tag and observe detection
4. Land
