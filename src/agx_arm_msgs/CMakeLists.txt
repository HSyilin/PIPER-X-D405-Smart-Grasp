cmake_minimum_required(VERSION 3.8)
project(agx_arm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgxArmStatus.msg"
  "msg/GripperStatus.msg"
  "msg/HandCmd.msg"
  "msg/HandPositionTimeCmd.msg"
  "msg/HandStatus.msg"
  "msg/MoveMITMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
