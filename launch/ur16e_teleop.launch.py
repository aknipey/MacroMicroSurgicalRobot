"""UR16e teleoperation stack: UR driver + MoveIt Servo + trajectory recorder (+ Touch).

    # Real robot, keyboard (run the keyboard node in a second terminal)
    ros2 launch macro_micro_robot ur16e_teleop.launch.py robot_ip:=192.168.0.100
    ros2 run macro_micro_robot ur16e_keyboard_teleop.py

    # Real robot, Touch stylus
    ros2 launch macro_micro_robot ur16e_teleop.launch.py robot_ip:=192.168.0.100 input:=touch

    # Replay a logged run's stylus instead of the Touch (same hand motion every time)
    ros2 launch macro_micro_robot ur16e_teleop.launch.py input:=replay replay_run:=V2/Up_10CM
    # ...or input:=replay alone, then in a second terminal (repeat as often as you like):
    ros2 run macro_micro_robot stylus_replay.py V2/Up_10CM

    # No robot: simulated (mock) hardware, same commands otherwise
    ros2 launch macro_micro_robot ur16e_teleop.launch.py use_mock_hardware:=true
    # ...with the real arm's delays added (ur_emulator.py), for latency testing
    ros2 launch macro_micro_robot ur16e_teleop.launch.py use_mock_hardware:=true emulate_ur:=true

    # Defaults to the lab UR16e with the nominal ur_description model, not this arm's calibration
    # (docs/UR_CALIBRATION_FIX.md). Other models / a calibration file:
    ros2 launch macro_micro_robot ur16e_teleop.launch.py ur_type:=ur5e robot_ip:=<ip> kinematics_params_file:=<file>

Terminal output is kept short: nodes from other packages (UR driver, controller
spawners, MoveIt, Touch driver) only print warnings and errors, and launch's own
"process started" / "finished cleanly" lines are hidden. Add verbose:=true to
see everything (full logs are always in ~/.ros/log either way).
"""

import logging
import os
import sys
from pathlib import Path

import yaml

import launch.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from macro_micro_robot.teleop_settings import (
    SERVO_LOOKAHEAD, SERVO_OUTPUT, SERVO_SMOOTH_SPEEDJ, SERVO_VELOCITY_RATE, TOUCH_STARTUP_DELAY,
    VELOCITY_LIMITER,
)
from macro_micro_robot.ur16e_common import (
    EMULATOR_CONTROLLER, EMULATOR_POSITION_TOPIC, EMULATOR_TRAJECTORY_TOPIC, EMULATOR_VELOCITY_TOPIC,
    LIMITER_INPUT_TOPIC, LIMITER_NODE, POSITION_CONTROLLER, SERVO_COMMAND_TOPIC, SERVO_POSITION_TOPIC,
    SERVO_VELOCITY_TOPIC, TELEOP_CONTROLLER, VELOCITY_CONTROLLER,
)

if SERVO_OUTPUT not in ("trajectory", "velocity", "position"):
    raise ValueError(f"SERVO_OUTPUT must be 'trajectory', 'velocity' or 'position', not {SERVO_OUTPUT!r}")
# The UR controller each output mode streams to (emulate_ur:=true aside)
OUTPUT_CONTROLLER = {"trajectory": TELEOP_CONTROLLER, "velocity": VELOCITY_CONTROLLER,
                     "position": POSITION_CONTROLLER}[SERVO_OUTPUT]

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
    if _quiet["on"] and package != "macro_micro_robot" and not getattr(self, "_quieted", False):
        executable = perform_substitutions(context, normalize_to_list_of_substitutions(self.node_executable))
        for level in ["warn"] + EXTRA_LOG_LEVELS.get(executable, []):
            self.cmd.extend([[TextSubstitution(text="--log-level")], [TextSubstitution(text=level)]])
        self._quieted = True
    return _node_execute(self, context)


Node.execute = _execute_quietly


def package_node(script, name, **kwargs):
    """Node for one of this package's scripts: the installed one, or straight from the
    source tree before the next colcon build."""
    if os.path.exists(os.path.join(get_package_prefix("macro_micro_robot"), "lib", "macro_micro_robot", script)):
        return Node(package="macro_micro_robot", executable=script, name=name, output="screen", **kwargs)
    source = Path(__file__).resolve().parent.parent / "src" / script
    parameters = kwargs.pop("parameters", [])
    arguments = ["--ros-args", "-r", f"__node:={name}"]
    for group in parameters:
        for key, value in group.items():
            arguments += ["-p", f"{key}:={value}"]
    return ExecuteProcess(cmd=["python3", str(source)] + arguments, name=name, output="screen", **kwargs)


def generate_launch_description():
    pkg_share = FindPackageShare("macro_micro_robot")
    use_touch = IfCondition(EqualsSubstitution(LaunchConfiguration("input"), "touch"))
    # The teleop node runs for the real Touch and for a replayed one; the driver only for the real one
    use_stylus = IfCondition(PythonExpression(["'", LaunchConfiguration("input"), "' in ('touch', 'replay')"]))
    auto_replay = IfCondition(PythonExpression([
        "'", LaunchConfiguration("input"), "' == 'replay' and '", LaunchConfiguration("replay_run"), "' != ''"]))

    ur_type = LaunchConfiguration("ur_type")

    # The old lab UR5e calibration only applies to that arm; any other type falls back
    # to its nominal ur_description model unless a calibration file is passed explicitly.
    default_kinematics = PythonExpression([
        "'", PathJoinSubstitution([pkg_share, "config", "lab_ur5e_1_calibration.yaml"]), "' if '",
        ur_type, "' == 'ur5e' else '",
        PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]), "'",
    ])

    args = [
        DeclareLaunchArgument("ur_type", default_value="ur16e",
                              choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
                              description="UR model being driven, e.g. ur16e"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.100"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Simulate the UR16e instead of connecting to it"),
        DeclareLaunchArgument("kinematics_params_file",
                              default_value=default_kinematics),
        DeclareLaunchArgument("input", default_value="keyboard", choices=["keyboard", "touch", "replay", "none"],
                              description="'touch' also starts the Touch driver and stylus teleop node. "
                                          "'replay' starts the teleop node without the driver, for "
                                          "stylus_replay.py. Keyboard teleop must be run in its own terminal."),
        DeclareLaunchArgument("replay_run", default_value="",
                              description="With input:=replay: run folder to replay once the stack is up "
                                          "(e.g. V2/Up_10CM, relative to logs/). Moves the robot to the "
                                          "run's start pose without asking."),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("trajectory_dir", default_value="~/ros2_ws/trajectories"),
        DeclareLaunchArgument("emulate_ur", default_value="false",
                              description="With use_mock_hardware:=true: add the real UR's delays "
                                          "(ur_emulator.py between Servo and the mock joints)"),
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
            # The controller MoveIt Servo streams to
            "initial_joint_controller": PythonExpression([
                "'", EMULATOR_CONTROLLER, "' if '", LaunchConfiguration("emulate_ur"), "' == 'true' else '",
                OUTPUT_CONTROLLER, "'"]),
            "launch_rviz": "false",
            # Same as the driver's ur_rsp.launch.py but with our robot script (see
            # SERVO_SMOOTH_SPEEDJ); the stock one otherwise
            "description_launchfile": PathJoinSubstitution([
                FindPackageShare("macro_micro_robot" if SERVO_SMOOTH_SPEEDJ else "ur_robot_driver"), "launch",
                "ur_rsp_smooth_speedj.launch.py" if SERVO_SMOOTH_SPEEDJ else "ur_rsp.launch.py"]),
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "launch_servo": "false",  # started below with our output settings
            "launch_rviz": "false",
        }.items(),
    )

    # MoveIt Servo, as ur_moveit.launch.py would start it but sending JointTrajectory
    # to the trajectory controller instead of Float64MultiArray positions (see
    # ur16e_common for why). Its output settings are read-only, hence started here.
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )
    servo_yaml_path = Path(get_package_share_directory("ur_moveit_config")) / "config" / "ur_servo.yaml"
    servo_yaml = yaml.safe_load(servo_yaml_path.read_text())
    # Position mode always needs the limiter: it integrates Servo's velocities
    use_limiter = SERVO_OUTPUT == "position" or (SERVO_OUTPUT == "velocity" and VELOCITY_LIMITER)
    if SERVO_OUTPUT in ("velocity", "position"):
        servo_yaml.update({
            "command_out_type": "std_msgs/Float64MultiArray",
            # Through joint_velocity_limiter.py when it is on
            "command_out_topic": LIMITER_INPUT_TOPIC if use_limiter else SERVO_VELOCITY_TOPIC,
            # One new joint speed per UR control cycle (see SERVO_VELOCITY_RATE)
            "publish_period": 1.0 / SERVO_VELOCITY_RATE,
            "publish_joint_positions": False,
            "publish_joint_velocities": True,
            "publish_joint_accelerations": False,
        })
    else:
        servo_yaml.update({
            "command_out_type": "trajectory_msgs/JointTrajectory",
            "command_out_topic": SERVO_COMMAND_TOPIC,
            "publish_joint_positions": True,
            # The newest point's velocity is never zero, which the controller rejects
            "publish_joint_velocities": False,
            "publish_joint_accelerations": False,
            "max_expected_latency": SERVO_LOOKAHEAD,
        })
    wait_robot_description = Node(package="ur_robot_driver", executable="wait_for_robot_description",
                                  output="screen")

    def start_servo(context):
        servo_params = dict(servo_yaml)
        emulate = context.launch_configurations.get("emulate_ur") == "true"
        if SERVO_OUTPUT == "position":
            limiter_out = EMULATOR_POSITION_TOPIC if emulate else SERVO_POSITION_TOPIC
        else:
            limiter_out = EMULATOR_VELOCITY_TOPIC if emulate else SERVO_VELOCITY_TOPIC
        velocity_out = EMULATOR_VELOCITY_TOPIC if emulate else SERVO_VELOCITY_TOPIC
        if emulate and not use_limiter:
            servo_params["command_out_topic"] = (
                velocity_out if SERVO_OUTPUT == "velocity" else EMULATOR_TRAJECTORY_TOPIC)
        servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            parameters=[moveit_config.to_dict(), {"moveit_servo": servo_params}],
            output="screen",
        )
        actions = [RegisterEventHandler(OnProcessExit(target_action=wait_robot_description, on_exit=[servo_node]))]
        if use_limiter:
            actions.append(package_node("joint_velocity_limiter.py", LIMITER_NODE,
                                        parameters=[{"output_topic": limiter_out, "output": SERVO_OUTPUT}]))
        return actions

    servo = OpaqueFunction(function=start_servo)

    ur_emulator = package_node("ur_emulator.py", "ur_emulator",
                               condition=IfCondition(LaunchConfiguration("emulate_ur")))

    recorder = Node(
        package="macro_micro_robot",
        executable="trajectory_recorder.py",
        name="trajectory_recorder",
        output="screen",
        parameters=[{"output_dir": LaunchConfiguration("trajectory_dir"),
                     "kinematics_params_file": LaunchConfiguration("kinematics_params_file")}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "ur16e_teleop.rviz"])],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        output="log",
    )

    # macro_micro_robot's own Touch driver, not omni_common's omni_state: forces from omni_state
    # never reach the motors (OpenHaptics renders none in a ROS process)
    touch_driver = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_share, "launch", "touch_driver.launch.py"]),
        condition=use_touch,
    )

    # Give the driver + servo time to come up before the teleop node configures them
    touch_teleop = TimerAction(
        period=TOUCH_STARTUP_DELAY,
        actions=[Node(
            package="macro_micro_robot",
            executable="ur16e_touch_teleop.py",
            name="ur16e_touch_teleop",
            output="screen",
            condition=use_stylus,
        )],
    )

    # After the teleop node has configured Servo (it needs a few seconds after it starts)
    stylus_replay = TimerAction(
        period=TOUCH_STARTUP_DELAY + 5.0,
        actions=[Node(
            package="macro_micro_robot",
            executable="stylus_replay.py",
            name="stylus_replay",
            output="screen",
            arguments=[LaunchConfiguration("replay_run"), "--yes"],
            condition=auto_replay,
        )],
    )

    return LaunchDescription(args + [
        OpaqueFunction(function=_set_quiet), ur_driver, moveit, wait_robot_description, servo, ur_emulator, recorder,
        rviz, touch_driver, touch_teleop, stylus_replay,
    ])
