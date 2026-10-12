"""UR5e teleoperation stack: UR driver + MoveIt Servo + trajectory recorder (+ Touch).

    # Real robot, keyboard (run the keyboard node in a second terminal)
    ros2 launch ls_thesis ur5e_teleop.launch.py robot_ip:=192.168.0.100
    ros2 run ls_thesis ur5e_keyboard_teleop.py

    # Real robot, Touch stylus
    ros2 launch ls_thesis ur5e_teleop.launch.py robot_ip:=192.168.0.100 input:=touch

    # No robot: simulated (mock) hardware, same commands otherwise
    ros2 launch ls_thesis ur5e_teleop.launch.py use_mock_hardware:=true

    # Other UR models (e.g. a UR16e): pass ur_type, ideally with its own calibration
    ros2 launch ls_thesis ur5e_teleop.launch.py ur_type:=ur16e robot_ip:=<ip> kinematics_params_file:=<file>

Terminal output is kept short: nodes from other packages (UR driver, controller
spawners, MoveIt, Touch driver) only print warnings and errors, and launch's own
"process started" / "finished cleanly" lines are hidden. Add verbose:=true to
see everything (full logs are always in ~/.ros/log either way).
"""

import logging
import sys

import launch.logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ls_thesis.teleop_settings import TOUCH_STARTUP_DELAY

# Launch's own per-process chatter, hidden unless verbose:=true
LAUNCH_CHATTER = ("process started with pid", "process has finished cleanly")
# move_group prints its plugin list straight to stdout (not ROS logging, so the log
# level does not catch it); its ROS warnings and errors go to stderr and still show.
HIDDEN_STDOUT = ("move_group-",)
# Per-logger levels on top of `--log-level warn`, for known harmless errors
EXTRA_LOG_LEVELS = {
    # "No 3D sensor plugin(s) defined for octomap updates" - there is no 3D sensor
    "move_group": ["move_group.moveit.moveit.ros.occupancy_map_monitor:=fatal"],
}
# ros2 launch loads included launch files once before anything runs (to list their
# arguments), so check the command line here as well as in _set_quiet
_quiet = {"on": not any(arg.replace(" ", "").lower() == "verbose:=true" for arg in sys.argv)}


class _HideRootWarnings(logging.Filter):
    """moveit_configs_utils warns "Cannot infer URDF/SRDF" on the root logger while
    ur_moveit.launch.py loads; the description comes from the topic, so it is harmless.

    A filter on the root logger only sees records logged to the root logger itself, not
    ones propagated from launch's loggers (which carry all node output), unlike setLevel.
    """

    def filter(self, record):
        return not (_quiet["on"] and record.levelno < logging.ERROR)


logging.getLogger().addFilter(_HideRootWarnings())


class _HideLaunchChatter(logging.Filter):
    def filter(self, record):
        if not _quiet["on"]:
            return True
        if record.name.startswith(HIDDEN_STDOUT) and record.name.endswith("-stdout"):
            return False
        return not record.getMessage().startswith(LAUNCH_CHATTER)


def _set_quiet(context):
    _quiet["on"] = context.launch_configurations.get("verbose", "false").lower() != "true"
    launch.logging.launch_config.get_screen_handler().addFilter(_HideLaunchChatter())
    return []


# The UR / MoveIt launch files hard-code output="screen", so instead of editing them,
# every Node not from this package gets `--log-level warn` added to its ROS arguments
# just before it starts (Node.cmd already ends in --ros-args at that point).
_node_execute = Node.execute


def _execute_quietly(self, context):
    package = perform_substitutions(context, normalize_to_list_of_substitutions(self.node_package))
    if _quiet["on"] and package != "ls_thesis" and not getattr(self, "_ls_quieted", False):
        executable = perform_substitutions(context, normalize_to_list_of_substitutions(self.node_executable))
        for level in ["warn"] + EXTRA_LOG_LEVELS.get(executable, []):
            self.cmd.extend([[TextSubstitution(text="--log-level")], [TextSubstitution(text=level)]])
        self._ls_quieted = True
    return _node_execute(self, context)


Node.execute = _execute_quietly


def generate_launch_description():
    pkg_share = FindPackageShare("ls_thesis")
    use_touch = IfCondition(EqualsSubstitution(LaunchConfiguration("input"), "touch"))

    ur_type = LaunchConfiguration("ur_type")

    # The lab UR5e calibration only applies to that arm; any other type falls back
    # to its factory kinematics unless a calibration file is passed explicitly.
    default_kinematics = PythonExpression([
        "'", PathJoinSubstitution([pkg_share, "config", "lab_ur5e_1_calibration.yaml"]), "' if '",
        ur_type, "' == 'ur5e' else '",
        PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]), "'",
    ])

    args = [
        DeclareLaunchArgument("ur_type", default_value="ur5e",
                              choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
                              description="UR model being driven, e.g. ur16e"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.100"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Simulate the UR5e instead of connecting to it"),
        DeclareLaunchArgument("kinematics_params_file",
                              default_value=default_kinematics),
        DeclareLaunchArgument("input", default_value="keyboard", choices=["keyboard", "touch", "none"],
                              description="'touch' also starts the Touch driver and stylus teleop node. "
                                          "Keyboard teleop must be run in its own terminal."),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("trajectory_dir", default_value="~/ros2_ws/trajectories"),
        DeclareLaunchArgument("verbose", default_value="false",
                              description="Show INFO output from the UR driver, MoveIt and launch itself"),
    ]

    ur_driver = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            # MoveIt Servo streams joint positions to this controller
            "initial_joint_controller": "forward_position_controller",
            "launch_rviz": "false",
        }.items(),
    )

    moveit_servo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "launch_servo": "true",
            "launch_rviz": "false",
        }.items(),
    )

    recorder = Node(
        package="ls_thesis",
        executable="trajectory_recorder.py",
        name="trajectory_recorder",
        output="screen",
        parameters=[{"output_dir": LaunchConfiguration("trajectory_dir")}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "ur5e_teleop.rviz"])],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        output="log",
    )

    touch_driver = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("omni_common"), "launch", "omni_state.launch.py"]),
        condition=use_touch,
    )

    # Give the driver + servo time to come up before the teleop node configures them
    touch_teleop = TimerAction(
        period=TOUCH_STARTUP_DELAY,
        actions=[Node(
            package="ls_thesis",
            executable="ur5e_touch_teleop.py",
            name="ur5e_touch_teleop",
            output="screen",
            condition=use_touch,
        )],
    )

    return LaunchDescription(args + [OpaqueFunction(function=_set_quiet), ur_driver, moveit_servo, recorder, rviz, touch_driver, touch_teleop])
