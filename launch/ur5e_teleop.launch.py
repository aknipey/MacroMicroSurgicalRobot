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
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        period=8.0,
        actions=[Node(
            package="ls_thesis",
            executable="ur5e_touch_teleop.py",
            name="ur5e_touch_teleop",
            output="screen",
            condition=use_touch,
        )],
    )

    return LaunchDescription(args + [ur_driver, moveit_servo, recorder, rviz, touch_driver, touch_teleop])
