cmake_minimum_required(VERSION 3.8)
project(ur3e_motion_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Build executables
add_executable(move_to_position src/move_to_position.cpp)
add_executable(move_to_position_testing src/move_to_position_testing.cpp)

ament_target_dependencies(move_to_position
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  std_msgs
)

ament_target_dependencies(move_to_position_testing
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
)

# Install
install(TARGETS move_to_position
  move_to_position_testing
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()