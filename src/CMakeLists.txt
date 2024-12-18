# Don't hit the wall example.
add_executable(dont-hit-the-wall-node
  dont_hit_the_wall_node.cpp
)
# Non ROS Dependencies
target_link_libraries(dont-hit-the-wall-node

)
# ROS Dependencies
ament_target_dependencies(dont-hit-the-wall-node
  "rclcpp"
  "edu_robot"
  "geometry_msgs"
)

# Enable Robot by Button
add_executable(enable-robot-by-button
  enable_robot_by_button.cpp
)
# Non ROS Dependencies
target_link_libraries(enable-robot-by-button
  mraa
)
# ROS Dependencies
ament_target_dependencies(enable-robot-by-button
  "rclcpp"
  "edu_robot"
)

# Toggle LED
add_executable(toggle_led
  toggle_led_node.cpp
)
# Non ROS Dependencies
target_link_libraries(toggle_led
  mraa
)
# ROS Dependencies
ament_target_dependencies(toggle_led
  "rclcpp"
  "edu_robot"
)