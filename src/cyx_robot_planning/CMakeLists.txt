cmake_minimum_required(VERSION 3.8)
project(cyx_robot_planning)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cyx_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(cyx_motion_server src/motion_server.cpp)
ament_target_dependencies(cyx_motion_server
  cyx_robot_interfaces geometry_msgs moveit_ros_planning_interface moveit_msgs moveit_core
  rclcpp rclcpp_action sensor_msgs std_msgs tf2_geometry_msgs)

add_executable(cyx_keyboard_jog src/keyboard_jog.cpp)
ament_target_dependencies(cyx_keyboard_jog
  cyx_robot_interfaces geometry_msgs rclcpp std_msgs)

add_executable(cyx_self_collision_analyzer src/self_collision_analyzer.cpp)
ament_target_dependencies(cyx_self_collision_analyzer
  moveit_core moveit_ros_planning rclcpp)

install(TARGETS cyx_motion_server cyx_keyboard_jog cyx_self_collision_analyzer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
