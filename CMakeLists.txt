cmake_minimum_required(VERSION 3.8)
project(ls_thesis)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament and dependency packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(cartesian_control_msgs REQUIRED)
find_package(motor_angles_msg REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(include)

## Helper libraries (pure C++ / Eigen / tf2, no ROS API)
add_library(pose src/Pose.cpp)
ament_target_dependencies(pose tf2)

add_library(manipulator_module src/ManipulatorModule.cpp)
target_link_libraries(manipulator_module Eigen3::Eigen)

## ur5e_control node
add_executable(ur5e_control src/ur5e_control.cpp)
target_link_libraries(ur5e_control pose)
ament_target_dependencies(ur5e_control
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  controller_manager_msgs
  omni_msgs
  cartesian_control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## micro_module_control node
add_executable(micro_module_control src/micro_module_control.cpp)
target_link_libraries(micro_module_control manipulator_module pose Eigen3::Eigen)
ament_target_dependencies(micro_module_control
  rclcpp
  std_msgs
  geometry_msgs
  omni_msgs
  motor_angles_msg
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## android_pose_subscriber node
add_executable(android_pose_subscriber src/android_pose_subscriber.cpp)
ament_target_dependencies(android_pose_subscriber
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  omni_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

#############
## Install ##
#############

## C++ executables
install(TARGETS
  ur5e_control
  micro_module_control
  android_pose_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

## Python node
install(PROGRAMS
  src/android_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

## Resources (launch files, model, visualisation, meshes)
install(DIRECTORY
  launch
  urdf
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
