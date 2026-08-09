cmake_minimum_required(VERSION 3.8)
project(smart_grasp_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(agx_arm_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(smart_grasp_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tl_expected REQUIRED)
find_package(trajectory_msgs REQUIRED)

if(TARGET tl_expected::tl_expected AND NOT TARGET tl::expected)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()

add_executable(pick_server src/pick_server.cpp)
target_compile_features(pick_server PUBLIC cxx_std_17)
target_include_directories(pick_server PRIVATE include)
ament_target_dependencies(pick_server
  agx_arm_msgs control_msgs geometry_msgs moveit_core moveit_msgs
  moveit_ros_planning_interface rclcpp rclcpp_action sensor_msgs shape_msgs
  smart_grasp_interfaces std_srvs tf2 tf2_geometry_msgs tl_expected trajectory_msgs
)

install(TARGETS pick_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pick_safety test/test_pick_safety.cpp)
  target_compile_features(test_pick_safety PUBLIC cxx_std_17)
  target_include_directories(test_pick_safety PRIVATE include)
  ament_target_dependencies(test_pick_safety geometry_msgs)
endif()

ament_package()
