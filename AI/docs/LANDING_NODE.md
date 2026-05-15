# Landing Node

## What it does

`landing_node` is a ROS2 node that closes the loop between ArUco detection and physical flight. It:

1. Arms the drone and takes off to a configurable altitude
2. Enters PX4 offboard mode and positions to an offset location
3. Runs a control loop that reads `/aruco/pose` detections and sends velocity commands to center the drone over the marker
4. Descends once aligned and triggers a PX4 land command when below the landing threshold

---

## How it fits into the ROS2 graph

```
aruco_node  →  /aruco/pose  →  landing_node  →  MAVSDK gRPC  →  mavsdk_server (sim container)  →  PX4
```

`landing_node` does not interact with the camera or OpenCV at all — it only reads the structured pose output from `aruco_node`. The MAVSDK connection goes out of the AI container over Docker networking to the `mavsdk_server` process running in the sim container alongside PX4.

---

## State machine

The control loop runs at `DETECT_HZ` (10 Hz) and cycles through four states:

| State | Condition | Action |
|---|---|---|
| `SEARCH` | No fresh pose from `/aruco/pose` (timeout > 1s) or z < 100mm | Hover in place — zero velocity |
| `ALIGN` | Marker detected, xy error > 15 cm | Lateral velocity toward marker centre |
| `DESCEND` | Marker detected, xy error ≤ 15 cm | Hold lateral position, descend at 0.3 m/s |
| `LAND` | Altitude < 0.4 m AGL | Break loop, call `drone.action.land()` |

State transitions are logged only when the state changes, so the log stays readable.

---

## Threading model

The node uses two concurrent execution contexts:

```
main thread      asyncio.run(run_landing(...))   — MAVSDK async flight coroutine
ros_thread       rclpy.spin(node)                — ROS2 callback processing
```

They share state through a `dict` + `threading.Lock`. The ROS2 callback (`_on_pose`) writes the latest pose and timestamp into the shared dict. The asyncio loop reads from it inside the control loop. The lock is always held for the minimum time needed.

This split is necessary because MAVSDK is async-only and `rclpy.spin` is blocking — they cannot run on the same thread.

---

## Startup sequence

```
connect to mavsdk_server (sim:50051)
  └── wait for GPS fix
        └── arm()
              └── wait for armed telemetry confirmation
                    └── sleep 3s (motor stabilisation)
                          └── takeoff()
                                └── wait for in_air == True
                                      └── wait for altitude ≥ 80% of target
                                            └── set_position_ned(offset seed)
                                                  └── offboard.start()
                                                        └── sleep 5s (fly to offset)
                                                              └── control loop begins
```

The altitude wait before offboard (`_wait_for_altitude`) prevents the control loop from running while the drone is still climbing — which would read 0 m AGL and immediately trigger the land branch.

The `set_position_ned` seed must be set and `offboard.start()` called immediately after — MAVSDK times out the registered setpoint if there is a delay between the two calls.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `altitude` | `3.0` | Takeoff altitude in metres |
| `mavsdk_server_host` | `sim` | Hostname of the `mavsdk_server` process |

Pass via `--ros-args -p altitude:=5.0` or through the launch file.

---

## Constants (tuning)

All control constants are at the top of `landing_node.py`:

| Constant | Value | Description |
|---|---|---|
| `KP_XY` | `0.5` | Proportional gain for lateral velocity |
| `MAX_XY_VEL_MS` | `1.0` | Maximum lateral speed (m/s) |
| `XY_ALIGN_THRESH_M` | `0.15` | Alignment radius before descending (m) |
| `DESCENT_SPEED_MS` | `0.3` | Downward speed while descending (m/s) |
| `LAND_ALT_M` | `0.4` | AGL altitude at which land is commanded (m) |
| `POSE_TIMEOUT_SEC` | `1.0` | Seconds without a pose before entering SEARCH |
| `DETECT_HZ` | `10` | Control loop rate (Hz) |

---

## Running standalone

The node is normally run via the `precision_landing.launch.py` launch file (see [PRECISION_LANDING.md](../../docs/PRECISION_LANDING.md)). To run it alone for testing:

```bash
# In the AI container, after sourcing install/setup.bash
ros2 run camera_feed landing_node --ros-args \
  -p altitude:=3.0 \
  -p mavsdk_server_host:=sim
```

`mavsdk_server` must already be running in the sim container — it does not start automatically. See [PRECISION_LANDING.md](../../docs/PRECISION_LANDING.md) for the command.

---

## Expected log output

```
[landing_node]: Subscribed to /aruco/pose. Takeoff altitude: 3.0 m  mavsdk_server: sim:50051
[landing_node]: Connecting to PX4 via mavsdk_server at sim:50051...
[landing_node]: Connected.
[landing_node]: Waiting for GPS fix...
[landing_node]: Arming and taking off to 3.0 m...
[landing_node]: Armed confirmed. Waiting before takeoff...
[landing_node]: Climbing to target altitude...
[landing_node]: Reached 2.4 m AGL.
[landing_node]: Entering offboard mode.
[landing_node]: Offboard active. Starting control loop.
[landing_node]: State: SEARCH   xy_err=0.000 m  alt=3.01 m
[landing_node]: State: ALIGN    xy_err=0.312 m  alt=3.01 m
[landing_node]: State: DESCEND  xy_err=0.041 m  alt=2.88 m
[landing_node]: 0.38 m AGL — commanding land.
[landing_node]: Landed and disarmed.
```

---

## Troubleshooting

**`COMMAND_DENIED` on `arm()`**

The drone from a previous run is still armed or airborne. Reset the simulation (`python3 sim.py --world landing --gui`) so PX4 starts clean with the drone on the ground.

**`NO_SETPOINT_SET` on `offboard.start()`**

`set_position_ned` was not called immediately before `start()`. There must be no `asyncio.sleep` between the two calls — MAVSDK times out the registered setpoint.

**State stays `SEARCH` indefinitely**

`aruco_node` is not detecting the marker. Check:
- `ros2 topic hz /aruco/pose` — if rate is 0, no detections
- The drone may have drifted off the marker during the offset phase
- The `z_mm >= 100.0` guard filters detections below 10 cm — ensure altitude is reasonable

**`TimeoutError: Timed out waiting to reach X m`**

PX4 takeoff did not reach the expected altitude within `IN_AIR_TIMEOUT` (30s). Usually means the sim is not running or the drone spawned underground. Restart the simulation.
