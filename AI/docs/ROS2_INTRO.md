# ROS2 Intro

## How the structure works

```
AI/
  src/                        ← all your ROS2 source code lives here
    camera_feed/              ← one package
      camera_feed/
        camera_node.py        ← one node inside the package
      package.xml             ← declares the package name and dependencies
      setup.py                ← tells colcon how to install it
```

A **package** is the unit of build and distribution. A **node** is a running Python process inside a package. One package can contain multiple nodes.

When you run `colcon build`, it reads every package under `src/` and installs them to `install/`. Sourcing `install/setup.bash` makes ROS2 aware of them.

To run a node:
```bash
ros2 run <package_name> <node_name>
# e.g.
ros2 run camera_feed camera_node
```

Nodes communicate by publishing and subscribing to **topics** — named channels with a fixed message type. The camera node subscribes to the image topic that the sim container publishes via the ROS2 bridge.

```
sim container                          AI container
ros_gz_bridge  →  /world/.../image  →  camera_node
```

---

## Useful links

- [Understanding nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) — they use turtlesim, we use Gazebo, same idea
- [Understanding topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) — how nodes communicate
- [Building a package with colcon](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Writing a publisher and subscriber](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) — how to build a node and use topics
