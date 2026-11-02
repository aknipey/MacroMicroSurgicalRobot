"""Read the arm's factory kinematic calibration from the controller into a kinematics_params_file.

Use this instead of `ros2 launch ur_calibration calibration_correction.launch.py`: the driver's
ur_calibration (3.8.0 here) never connects, and once connected writes wrong link rotations.
See docs/UR_CALIBRATION_FIX.md.

    ros2 launch macro_micro_robot ur_calibration_extract.launch.py
    ros2 launch macro_micro_robot ur_calibration_extract.launch.py robot_ip:=<ip> target_filename:=<file>

Robot powered on (brakes may stay on, no remote control or External Control needed), port 30001 open.
Re-check a saved file offline: dh_file:=<that file> (no robot needed).
"""

import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Into the source tree, not install/, so it can be committed and passed to launch_system.sh
DEFAULT_TARGET = os.path.expanduser(
    "~/ros2_ws/src/MacroMicroSurgicalRobot/config/calibrations/"
    f"ur16e_kinematics_{datetime.date.today().isoformat()}.yaml")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.100"),
        DeclareLaunchArgument("target_filename", default_value=DEFAULT_TARGET),
        DeclareLaunchArgument("ur_type", default_value="ur16e",
                              description="Only picks the nominal model the calibration is compared against"),
        DeclareLaunchArgument("dh_file", default_value="",
                              description="Offline: raw D-H parameters from a file instead of the robot"),
        Node(package="macro_micro_robot", executable="ur_calibration_extract", output="screen",
             parameters=[{
                 "robot_ip": LaunchConfiguration("robot_ip"),
                 "output_filename": LaunchConfiguration("target_filename"),
                 "dh_file": LaunchConfiguration("dh_file"),
                 "nominal_kinematics_file": PathJoinSubstitution([
                     FindPackageShare("ur_description"), "config", LaunchConfiguration("ur_type"),
                     "default_kinematics.yaml"]),
             }]),
    ])
