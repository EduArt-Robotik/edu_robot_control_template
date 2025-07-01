# Overview
This example uses the Simple Commander API for NAV2.
It specifies 3 Waypoints the robot has to reach. It automatically activates the robot (sets mode to autonomous).
NAV2 and a localization must be running.

The package can be copied directly into a local ROS2 Jazzy workspace.

# Dependencies
- NAV2
- edu_robot

# Build the example
```bash
colcon build --packages-select edu_nav2_example_py --symlink-install
```

# Run the example
This command executes the launch file:
```bash
ros2 launch edu_nav2_example_py nav2_controller_launch.py
```
This is how the output should look like:
```bash
ros2 launch edu_nav2_example_py nav2_controller_launch.py
[INFO] [launch]: All log files can be found below /home/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [nav2_py_controller-1]: process started with pid [69604]
[nav2_py_controller-1] [...]: Robot Namespace: eduard/simulation
[nav2_py_controller-1] [...]: Nav2 is ready for use!
[nav2_py_controller-1] [...]: Following 3 goals....
[nav2_py_controller-1] [...]: Service call successful - Autonomous mode
activated
[nav2_py_controller-1] [...]: Executing current waypoint: 1/3
...
```