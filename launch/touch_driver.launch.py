"""Touch driver: touch_server (OpenHaptics device loop, no ROS) + touch_bridge (ROS topics).

Drop-in for `ros2 launch omni_common omni_state.launch.py`: the same /phantom/state,
/phantom/button, /phantom/pose and /phantom/joint_states, and /phantom/force_feedback
in. omni_state's forces never reach the motors, because OpenHaptics renders no force
in a process that has ROS 2's libraries loaded (see src/touch_server.cpp).

    ros2 launch macro_micro_robot touch_driver.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Not a ROS node (it ignores the --ros-args launch adds); owns the Touch
        Node(package="macro_micro_robot", executable="touch_server", name="touch_server", output="screen"),
        Node(package="macro_micro_robot", executable="touch_bridge", name="omni_haptic_node", output="screen",
             parameters=[{"omni_name": "phantom", "reference_frame": "/map"}]),
    ])
