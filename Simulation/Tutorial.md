* 
   Docs:                                                                           
  - MAVSDK Python telemetry API: https://mavsdk.mavlink.io/main/en/python/api_reference/telemetry.html           
  - 
  - MAVSDK Python quickstart: https://mavsdk.mavlink.io/main/en/python/quickstart.html                        
  - PX4 flight modes reference: https://docs.px4.io/main/en/getting_started/flight_modes.html


EXAMPLES: https://github.com/mavlink/MAVSDK-Python/tree/main/examples


TASK1: 
- connect to drone
- Log position, velocity and NED_euler for 15 secs

TASK2:
- connect to drone
- arm
- takeoff
- go into offboard mode -(offboard_velocity_ned to command)
- fly 2ms north, 2ms east, south, west, in a loop 2 secs per direction for 20 secs total, then land
-  
  *