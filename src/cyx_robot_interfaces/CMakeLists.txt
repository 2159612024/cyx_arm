cmake_minimum_required(VERSION 3.8)
project(cyx_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionOptions.msg"
  "msg/MotionStatus.msg"
  "msg/CartesianJog.msg"
  "msg/JointJog.msg"
  "action/MoveToNamed.action"
  "action/MoveToJoints.action"
  "action/MoveToPose.action"
  "action/FollowCartesianPath.action"
  "action/ExecutePlan.action"
  "srv/Plan.srv"
  "srv/StopMotion.srv"
  "srv/GetMotionState.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
