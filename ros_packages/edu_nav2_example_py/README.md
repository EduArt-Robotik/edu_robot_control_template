# Overview
This example uses the Simple Commander API for NAV2.
It specifies 3 Waypoints the robot has to reach. It automatically activates the robot (sets mode to autonomous).
NAV2 and a localization must be running.

# Dependencies
- NAV2
- edu_robot

# Build the example
```bash
colcon build --packages-select edu_nav2_example_py --symlink-install
```

# Run the example
```bash
ros2 launch edu_nav2_example_py nav2_controller_launch.py
```