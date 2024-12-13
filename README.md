# Edu Robot Control Template
This repository serves as an easy introduction template to the programming of the EduArt's robots like IoTBot or IPCBot.

# ROS Node Examples

## Building Package

Before the example nodes can be used it is required to build this packages by following command:

```bash
colcon build --event-handlers console_direct+ --symlink-install
```

> **Note:** Please be aware of that the dependency [edu_robot](https://github.com/EduArt-Robotik/edu_robot) must be located in the same workspace or must be installed.

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

## Enable Robot by Button

### GPIO

Make sure you setted up the IOT GPIOs right:
- iot2050setups
- Peripherals
- Configure Arduino I/O
- Enable GPIO
- Arrow keys to your PIN (in this case GPIO4 and GPIO5)
- Tap to "Direction" -> Arrow keys to "Input" Tap to "Pull-Mode"
- "Pull - up" for low active -> Tap to <OK> -> Enter -> Esc for exit

### Launching

```bash
ros2 run edu_robot_control_template enable-robot-by-button --ros-args -r __ns:=/eduard/blue
```

# Deploying Using Docker

Unfortunately, it is necessary to run ROS2 applications in a Docker container on the IoT2050. This is due to the fact that no ROS2 packages are available for Debian 11. The following describes how to deploy a ROS2 package into a Docker image and start ROS2 nodes from it.

## Building Docker Image

Please execute in the top level folder of your ROS2 package following command to build an Docker image of it:

```bash
docker build -t edu-robot-control-template .
```

# Coming Soon

Following is in progress and will be added to this repository soon:

* Deploying your code via Docker image on your robot.
* (C++) Provide GPIO state via ROS service.
* (Python) Provide GPIO state via ROS service.
