# ArUco Detection Node

## What it does

`aruco_node` is a ROS2 node that subscribes to the camera topic, detects ArUco markers in each frame, and publishes their 3D position offsets. Those offsets are what a landing controller will use — x/y to center the drone over the marker, z to control descent.

---

## How it fits into the ROS2 graph

In ROS2, nodes are independent processes that communicate only through topics. No node calls another directly — they just publish and subscribe.

```
sim container                          AI container
ros_gz_bridge  →  /world/.../image  →  aruco_node  →  /aruco/pose  →  (future) landing_node
```

`aruco_node` sits in the middle: it consumes raw camera frames and produces structured detection data. The landing controller doesn't need to know anything about cameras or OpenCV — it just reads `/aruco/pose`.

This separation is the point of ROS2. Each node has one job, and you can swap or extend either side without touching the other.

---

## Message types

**Input:** `sensor_msgs/msg/Image`  
The standard ROS2 image message. `cv_bridge` converts it to a numpy array OpenCV can work with.

**Output:** `geometry_msgs/msg/PoseStamped`  
A standard ROS2 pose message containing:
- `header.stamp` — timestamp of the detection
- `header.frame_id` — the marker ID (as a string, e.g. `"0"`)
- `pose.position.x/y/z` — offset from the camera in mm

Only published when a marker is detected. If multiple markers are visible, the closest one (smallest z) is published.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `world` | from `sim_state.json` | Auto-detected from running sim |
| `vehicle` | from `sim_state.json` | Auto-detected from running sim |
| `display` | `false` | Show annotated OpenCV window |
| `marker_size_mm` | `100.0` | Physical size of marker in mm |
| `dict_name` | `4x4_1000` | ArUco dictionary |

For the `landing` world: `dict_name:=4x4_50` and `marker_size_mm:=500.0`.

---

## The landing world

The PX4 built-in `aruco` world uses a model called `arucotag` whose definition isn't accessible in the repo — so you can't know the exact marker size or dictionary. Instead, use the `landing` world defined in `Simulation/worlds/landing.sdf`, which has a single `aruco_4x4_id0` marker (500mm, `4x4_50` dict) placed at the world origin directly below the drone spawn point.

### How custom worlds are found

PX4 normally only looks for worlds in its own internal directory. `sim.py` handles this automatically: if the world name matches a `.sdf` file in `Simulation/worlds/`, it sets `PX4_GZ_WORLDS` before launching, which tells `launch_sim.sh` to symlink the world and its models into PX4's search path. Built-in worlds like `aruco` or `baylands` are unaffected.

---

## Running

Start the sim in the sim container:
```bash
python3 sim.py --world landing --gui
```

In the AI container:
```bash
ros2 run camera_feed aruco_node --ros-args -p dict_name:=4x4_50 -p marker_size_mm:=500.0
```

With display (requires X11):
```bash
ros2 run camera_feed aruco_node --ros-args -p dict_name:=4x4_50 -p marker_size_mm:=500.0 -p display:=true
```

---

## Verifying detections

In a second terminal (after sourcing `install/setup.bash`):

```bash
# Confirm topic is live
ros2 topic list | grep aruco

# Stream detections
ros2 topic echo /aruco/pose

# Check publish rate (only publishes when marker is visible)
ros2 topic hz /aruco/pose
```

Expected output when the drone is over the marker:
```
header:
  frame_id: '0'
pose:
  position:
    x: -12.3     # mm left/right
    y: 8.1       # mm forward/back
    z: 842.0     # mm above marker
```

---

## What's next

A `landing_node` subscribes to `/aruco/pose` and:
- Sends lateral velocity commands to zero out `x` and `y`
- Reduces descent rate as `z` decreases
- Triggers land once within threshold distance
