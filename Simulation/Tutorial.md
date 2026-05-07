Docs:
  - MAVSDK Python telemetry API: https://mavsdk.mavlink.io/main/en/python/api_reference/telemetry.html
  - MAVSDK Python quickstart: https://mavsdk.mavlink.io/main/en/python/quickstart.html
  - PX4 flight modes reference: https://docs.px4.io/main/en/getting_started/flight_modes.html

EXAMPLES: https://github.com/mavlink/MAVSDK-Python/tree/main/examples

=========================
GOAL: Learn how connection, logging, and moving the drone works
=========================

TASK 1: Connect to the drone and read telemetry
- Connect to the drone
- Log position, velocity and NED euler for 15 seconds


TASK 2: Take off and land
- Connect to the drone
- Arm the drone
- Take off to a target altitude
- Wait a few seconds
- Land


TASK 3: Fly in a square using NED frame
- Connect to the drone
- Arm and take off
- Enter offboard mode (use offboard_velocity_ned to command)
- Fly 2 m/s north, east, south, west in a loop — 2 seconds per direction for 20 seconds total
- Land

=========================
NOW WE UNDERSTAND NED FRAME!!
=========================

TASK 4: Fly in a square using Body frame
- Connect to the drone
- Arm and take off
- Enter offboard mode (use offboard_velocity_body to command)
- Fly forward, right, backward, left in a loop — 2 seconds per direction for 20 seconds total
- Land

=========================
NOW WE UNDERSTAND BODY FRAME!!
=========================


TASK 5: SIM CAMERA
- Subscribe to gazebo camera topic
- CONVERT IMage to BGR (opencv readable)
- Im.show(frames)


TASK 6: Aruco tag detection

- Use Sim camera frames to detect aruco markers
- Draw aruco markers on fram
- Publish to aruco detected frame with x,y,z distance to marker center to a ROS topic.
