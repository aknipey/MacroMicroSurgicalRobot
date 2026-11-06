cmake_minimum_required(VERSION 3.8)
project(macro_micro_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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
find_package(ur_client_library REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

## Helper libraries (pure C++ / Eigen / tf2, no ROS API)
add_library(pose src/pose.cpp)
ament_target_dependencies(pose tf2)

add_library(manipulator_module src/manipulator_module.cpp)
target_link_libraries(manipulator_module Eigen3::Eigen)

## ur16e_control node
add_executable(ur16e_control src/ur16e_control.cpp)
target_link_libraries(ur16e_control pose)
ament_target_dependencies(ur16e_control
  rclcpp
  rclcpp_action
  geometry_msgs
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
  tf2_geometry_msgs
)

## android_pose_subscriber node
add_executable(android_pose_subscriber src/android_pose_subscriber.cpp)
ament_target_dependencies(android_pose_subscriber
  rclcpp
  geometry_msgs
  visualization_msgs
  omni_msgs
  tf2
  tf2_geometry_msgs
)

## ur_calibration_extract: reads the arm's factory calibration into a kinematics_params_file.
## Replaces the driver's ur_calibration, which in our 3.8.0 checkout never connects and writes wrong
## link rotations (docs/UR_CALIBRATION_FIX.md). vendor/ur_calibration = byte-identical upstream sources.
add_executable(ur_calibration_extract
  src/ur_calibration_extract.cpp
  vendor/ur_calibration/src/calibration.cpp
)
target_include_directories(ur_calibration_extract PRIVATE vendor/ur_calibration/include)
target_link_libraries(ur_calibration_extract ur_client_library::urcl yaml-cpp Eigen3::Eigen)
ament_target_dependencies(ur_calibration_extract rclcpp)

## Touch driver, split in two: OpenHaptics renders no force in a process that has ROS 2's
## libraries (lttng-ust / libnuma) loaded, so touch_server (device, OpenHaptics only -
## never link ROS into it) and touch_bridge (ROS only) talk over UDP on localhost
find_path(OPENHAPTICS_INCLUDE_DIR NAMES HD/hd.h)
if(OPENHAPTICS_INCLUDE_DIR)
  add_executable(touch_server src/touch_server.cpp)
  target_include_directories(touch_server PRIVATE ${OPENHAPTICS_INCLUDE_DIR})
  target_link_libraries(touch_server HD HDU rt pthread)
  install(TARGETS touch_server DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "OpenHaptics not found - touch_server is not built")
endif()

add_executable(touch_bridge src/touch_bridge.cpp)
ament_target_dependencies(touch_bridge rclcpp geometry_msgs sensor_msgs omni_msgs)

install(TARGETS
  ur16e_control
  micro_module_control
  android_pose_subscriber
  touch_bridge
  ur_calibration_extract
  DESTINATION lib/${PROJECT_NAME}
)

## Python nodes
install(PROGRAMS
  src/android_pose_publisher.py
  src/ur16e_keyboard_teleop.py
  src/ur16e_touch_teleop.py
  src/trajectory_recorder.py
  src/trajectory_player.py
  src/stylus_replay.py
  src/ur_emulator.py
  src/joint_velocity_limiter.py
  src/tool_library.py
  DESTINATION lib/${PROJECT_NAME}
)

## Python helpers shared by the UR16e nodes (import macro_micro_robot.ur16e_common)
ament_python_install_package(${PROJECT_NAME})

## Resources (launch files, model, visualisation, meshes)
install(DIRECTORY
  launch
  urdf
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  config/lab_ur5e_1_calibration.yaml
  config/external_control_smooth_speedj.urscript
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
