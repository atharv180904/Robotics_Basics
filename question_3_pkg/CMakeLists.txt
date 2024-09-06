cmake_minimum_required(VERSION 3.5)
project(question_3_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare a C++ executable
add_executable(move_program src/move.cpp)

# Specify libraries to link against
ament_target_dependencies(move_program
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# Install targets
install(TARGETS
  move_program
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package configuration
ament_package()
