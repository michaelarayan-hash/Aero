* 
   Docs:                                                                           
  - MAVSDK Python telemetry API: https://mavsdk.mavlink.io/main/en/python/api_reference/telemetry.html           
  - 
  - MAVSDK Python quickstart: https://mavsdk.mavlink.io/main/en/python/quickstart.html                        
  - PX4 flight modes reference: https://docs.px4.io/main/en/getting_started/flight_modes.html


EXAMPLES: https://github.com/mavlink/MAVSDK-Python/tree/main/examples


TASK1: Connect to the drone and read telemetry
- connect to drone
- Log position, velocity and NED_euler for 15 secs

TASK2: Fly in a square using NED frame and Velocity commands
- connect to drone
- arm
- takeoff
- go into offboard mode -(offboard_velocity_ned to command)
- fly 2ms north, 2ms east, south, west, in a loop 2 secs per direction for 20 secs total, then land
  
  

TASK 3: Fly in a square using Body Frame and GPS coordinates
  

=========================
NOW WE UNDERSTAND BODY AND NED FRAME!!
=========================

TASK4: Recognise april tag
- Write a camera feed display loop.
- apply OpenCV tag detection
- takeoff above the tag and see the tag get detected
- land



  *


