# Random ArUco World

A Gazebo world with a single ArUco 4x4_50 ID0 marker placed at a random position around the drone spawn point each run. Useful for testing detection and target-approach logic without a fixed marker location.

---

## Marker

| Model | Dictionary | ID |
|---|---|---|
| `aruco_4x4_id0` | ArUco 4x4_50 | 0 |

Your detection node needs `DICT_4X4_50`.

---

## Randomization

Each run, `world_gen.py` generates a fresh layout before PX4 starts:

- The marker is placed at a random distance between **2 m and 8 m** from the origin
- The angle around the origin is fully random (0°–360°)
- The marker gets a random yaw so its face isn't always aligned north

The generated SDF is written to `/tmp/px4_worlds/random_aruco.sdf` and `PX4_GZ_WORLDS` is pointed there. The static `worlds/random_aruco.sdf` is used as a reference only — it is not loaded at runtime.

---

## Running

```bash
# Random layout (new seed each time)
python3 sim.py --world random_aruco --gui

# Reproduce a specific layout
python3 sim.py --world random_aruco --seed 918092314 --gui

# Skip randomization, use the static .sdf as-is
python3 sim.py --world random_aruco --no-randomize --gui
```

The seed is printed at startup:
```
World randomized  seed=918092314  (rerun with --seed 918092314 to reproduce)
```

It is also saved to `/workspace/tmp/sim_state.json` so AI nodes can read which seed was used:
```json
{"world": "random_aruco", "vehicle": "x500_mono_cam_down", "seed": 918092314}
```

---

## Verifying the layout

After launch, inspect the generated position inside the sim container:

```bash
cat /tmp/px4_worlds/random_aruco.sdf | grep pose
```

The `<pose>` line shows `x y z roll pitch yaw` for the marker.

---

## Tuning spawn range

The randomization lives in `Simulation/world_gen.py` under `_random_aruco()`. To adjust the spawn range, edit:

```python
dist  = rng.uniform(2, 8)   # distance from origin (metres)
angle = rng.uniform(0, 2 * math.pi)  # angle around origin (radians)
```
