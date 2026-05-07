# ROS2 Architecture Design

**Date:** 2026-05-07
**Status:** In Progress

## Overview

Extend the existing simulation stack with a separate AI container built on ROS2 Jazzy. Both containers run on developer x86 machines for development and testing. The AI image is also built for ARM64 and deployed to a Jetson Orin Nano on the physical drone.

---

## Repository Layout

```
Aero/
├── Simulation/               # Existing — Gazebo + PX4 SITL
│   └── docker/
│       ├── Dockerfile
│       └── entrypoint.sh
├── AI/                       # New — ROS2 nodes, vision, planning
│   └── docker/
│       ├── Dockerfile
│       └── entrypoint.sh
└── docker-compose.yml        # Root compose — wires Simulation + AI together
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Dev Machine (x86)                     │
│                                                             │
│  ┌──────────────────────┐      ┌──────────────────────────┐ │
│  │   Simulation         │      │   AI                     │ │
│  │                      │      │                          │ │
│  │  Gazebo Harmonic     │      │  ROS2 Jazzy nodes        │ │
│  │  PX4 SITL            │      │  Vision + Planning       │ │
│  │  ros-gz-bridge       │      │  px4_msgs                │ │
│  │  micro-ros-agent     │◄────►│                          │ │
│  └──────────────────────┘      └──────────────────────────┘ │
│            ROS2 DDS (shared Docker network)                  │
└─────────────────────────────────────────────────────────────┘

                    same AI image, ARM64 build
                              │
                              ▼
                   ┌──────────────────────┐
                   │   Jetson Orin Nano   │
                   │   AI container       │
                   │   + TensorRT/CUDA    │
                   └──────────────────────┘
                              │
                         real PX4 FC
                         (uXRCE-DDS over UART)
```

---

## Container Breakdown

### Simulation container

| Component | Role |
|---|---|
| Gazebo Harmonic | Physics simulation |
| PX4 SITL | Autopilot running in software |
| `ros-gz-bridge` | Translates Gazebo gz-transport topics → ROS2 topics |
| `micro-ros-agent` | Bridges PX4 SITL ↔ ROS2 via uXRCE-DDS (UDP) |

ROS2 in this container is infrastructure only — no custom nodes live here.

### AI container

| Component | Role |
|---|---|
| ROS2 Jazzy | Middleware — all nodes communicate via topics |
| `px4_msgs` | PX4 message types for telemetry and commands |
| PyTorch + OpenCV | Vision inference and image processing |
| nav2 | Path planning and navigation |

All custom code lives here. This is the only image the AI team edits.

---

## Communication

### Simulation ↔ AI (development)

Both containers share a Docker bridge network. ROS2 DDS discovery works across containers on the same network automatically. No port mapping or special config needed.

```
Gazebo sensor data:
  Gazebo → gz-transport → ros-gz-bridge → ROS2 topic → AI nodes

Flight commands:
  AI nodes → ROS2 topic → micro-ros-agent → uXRCE-DDS → PX4 SITL

Telemetry:
  PX4 SITL → uXRCE-DDS → micro-ros-agent → ROS2 topic → AI nodes
```

### AI ↔ Real Hardware (deployment)

The same AI nodes run unchanged. Only the transport layer differs:

```
AI nodes → ROS2 topic → micro-ros-agent → uXRCE-DDS → PX4 FC (UART)
```

AI nodes never know if they are talking to SITL or a real flight controller.

---

## Python Environments

| Container | Python env | Used for |
|---|---|---|
| Simulation | `uv` venv (`/home/dev/.venv`) | Standalone scripts: MAVSDK, OpenCV, AprilTags |
| Simulation | System Python (ROS2) | `ros-gz-bridge`, `micro-ros-agent` |
| AI | System Python (ROS2) | All ROS2 nodes — colcon manages this |
| AI | `uv` venv | Non-ROS utilities if needed |

**Rule:** if it is a ROS2 node, it is a colcon package. If it is a standalone script, it goes in the uv venv.

---

## Development Workflow

1. Run `docker compose up` from repo root — starts both containers
2. Edit AI node code on the host — bind-mounted into the AI container
3. Inside the AI container: `colcon build && source install/setup.bash`
4. Test against the running Simulation container via ROS2 topics
5. `ros2 topic list` to verify topic visibility across containers

`source install/setup.bash` after every `colcon build` is always manual — this is standard ROS2 workflow.

---

## Deployment to Jetson Orin Nano

The AI Dockerfile targets x86 for development. For Jetson deployment, build the same Dockerfile for ARM64:

```bash
docker buildx build --platform linux/arm64 -t aero-ai:arm64 AI/
```

Push to a registry and pull on the Jetson. The Jetson runs the container with CUDA/TensorRT available for full inference performance.

The Simulation container is never deployed to the Jetson — it is a development tool only.

---

## Key Decisions

| Decision | Reason |
|---|---|
| Separate Simulation and AI containers | AI image must deploy to Jetson; Simulation is x86 only |
| ROS2 in both containers | Simulation needs `ros-gz-bridge`; AI needs it for all nodes |
| uXRCE-DDS as PX4 bridge | Works identically over UDP (SITL) and UART (real hardware) |
| Single AI Dockerfile for x86 + ARM64 | One codebase, built with `docker buildx` for Jetson |
| Jetson Orin Nano as deployment target | CUDA cores support PyTorch/TensorRT inference on-drone |
| uv venv for non-ROS scripts | Keeps sim scripts isolated from system Python that ROS2 depends on |
