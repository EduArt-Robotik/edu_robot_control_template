# Don't hit the wall example.
add_executable(dont-hit-the-wall-node
  dont_hit_the_wall_node.cpp
)
target_link_libraries(dont-hit-the-wall-node

)
ament_target_dependencies(dont-hit-the-wall-node
  "rclcpp"
  "edu_robot"
)
