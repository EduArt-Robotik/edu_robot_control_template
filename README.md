# Edu Robot Control Template
This repository serves as an easy introduction template to the programming of the EduArt's robots like IoTBot or IPCBot.

# ROS Node Examples

## Building Package

Before the example nodes can be used it is required to build this packages by following command:

```bash
colcon build --event-handlers console_direct+ --symlink-install
```

> **Note:** Please be aware of that the dependency edu_robot must be located in the same workspace or must be installed.

## Don't hit the wall

This node controls the robot in the scenario and moves forward until it comes close to a wall (or another object), then it stops in front of the wall. This example shows how to use the basic interfaces of the robot, such as setting the mode, sending velocity commands (twist) and receiving sensor data (here retrieving distance measurements from one of the distance sensors).

* [C++ Implementation](src/dont_hit_the_wall_node.cpp)
* [Python Implementation](scripts/dont_hit_the_wall_node.py)

### Launching C++ Node

```bash
ros2 run edu_robot_control_template dont-hit-the-wall-node --ros-args -r __ns:=<put here the namespace of your robot, like /eduard/blue>
```

> **Note**: if your robot does not have any namespace please remove it from the command starting from "--ros-args".

### Launching Python Node

```bash
ros2 run edu_robot_control_template dont_hit_the_wall_node.py --ros-args -r __ns:=<put here the namespace of your robot, like /eduard/blue>
```

# Coming Soon

Following is in progress and will be added to this repository soon:

* (C++) Provide GPIO state via ROS service.
* (Python) Provide GPIO state via ROS service.
