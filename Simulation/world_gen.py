"""
Procedural Gazebo world generator.

Usage (from sim.py):
    import world_gen
    sdf = world_gen.generate("multi_aruco", seed=42)  # str or None

Returns None for worlds with no generator (static .sdf is used as-is).
"""

import math
import random


# ── SDF boilerplate ───────────────────────────────────────────────────────────

def _header(world_name: str) -> str:
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{world_name}">

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>1 1</size></plane>
          </geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
      </link>
      <pose>0 0 -0.01 0 0 0</pose>
      <self_collide>false</self_collide>
    </model>

    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range><linear>0</linear>
        <constant>1</constant><quadratic>0</quadratic>
      </attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>"""


def _footer() -> str:
    return "\n\n  </world>\n</sdf>\n"


def _include(uri: str, name: str, x: float, y: float, yaw: float) -> str:
    return (
        f"\n\n    <include>"
        f"\n      <uri>model://{uri}</uri>"
        f"\n      <name>{name}</name>"
        f"\n      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>"
        f"\n    </include>"
    )


# ── Generators ────────────────────────────────────────────────────────────────

def _random_aruco(rng: random.Random) -> str:
    """One ArUco 4x4_50 ID0 marker at a random position around the drone spawn."""
    dist  = rng.uniform(2, 5)
    angle = rng.uniform(0, 2 * math.pi)
    x     = dist * math.cos(angle)
    y     = dist * math.sin(angle)
    yaw   = rng.uniform(0, 2 * math.pi)

    body = _header("random_aruco")
    body += _include("aruco_4x4_id0", "aruco_4x4_id0", x, y, yaw)
    return body + _footer()


# ── Public API ────────────────────────────────────────────────────────────────

_GENERATORS = {
    "random_aruco": _random_aruco,
}


def generate(world_name: str, seed: int) -> "str | None":
    """Return a randomized SDF string, or None if this world has no generator."""
    gen = _GENERATORS.get(world_name)
    if gen is None:
        return None
    return gen(random.Random(seed))
