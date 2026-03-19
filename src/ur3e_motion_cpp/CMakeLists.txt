cmake_minimum_required(VERSION 3.8)
project(ur3e_motion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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

# Build the executable
add_executable(move_to_position src/move_to_position.cpp)

ament_target_dependencies(move_to_position
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
)

# Install
install(TARGETS move_to_position
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()