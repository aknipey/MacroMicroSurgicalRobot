"""Shared helpers for the UR5e teleoperation / trajectory nodes.

UR5e motion in ROS 2 goes through MoveIt Servo:

    keyboard / Touch node --TwistStamped--> /servo_node/delta_twist_cmds
    servo_node --JointTrajectory--> /scaled_joint_trajectory_controller/joint_trajectory
    (or, with SERVO_OUTPUT "velocity", Float64MultiArray joint velocities to
    /forward_velocity_controller/commands)

Servo must not output Float64MultiArray positions: in position mode it re-bases every
step on the measured joint state, so on the real arm (which takes ~40 ms to reach a
command) the robot only ever moves ~10% of the commanded speed.
Recorded trajectories are replayed through the same controller's action interface.
"""

from controller_manager_msgs.srv import SwitchController
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from moveit_msgs.srv import ServoCommandType
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger
import rclpy

SERVO_TWIST_TOPIC = "/servo_node/delta_twist_cmds"
SERVO_COMMAND_TYPE_SERVICE = "/servo_node/switch_command_type"
SERVO_PAUSE_SERVICE = "/servo_node/pause_servo"
SERVO_SET_PARAMETERS_SERVICE = "/servo_node/set_parameters"
SERVO_GET_PARAMETERS_SERVICE = "/servo_node/get_parameters"

SWITCH_CONTROLLER_SERVICE = "/controller_manager/switch_controller"
TELEOP_CONTROLLER = "scaled_joint_trajectory_controller"
SERVO_COMMAND_TOPIC = f"/{TELEOP_CONTROLLER}/joint_trajectory"
VELOCITY_CONTROLLER = "forward_velocity_controller"
SERVO_VELOCITY_TOPIC = f"/{VELOCITY_CONTROLLER}/commands"
# ur_emulator.py (mock hardware with the real arm's delays) takes Servo's output here
# and drives the mock joints through forward_position_controller
EMULATOR_TRAJECTORY_TOPIC = "/ur_emulator/joint_trajectory"
EMULATOR_VELOCITY_TOPIC = "/ur_emulator/commands"
EMULATOR_CONTROLLER = "forward_position_controller"
# forward_velocity_controller keeps its last command until a new one arrives, so the
# arm would keep moving if Servo died mid-motion: VelocityWatchdog zeroes it after this (s)
VELOCITY_WATCHDOG_TIMEOUT = 0.05
TRAJECTORY_CONTROLLER = "scaled_joint_trajectory_controller"
TRAJECTORY_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"

# Every other joint-level controller the UR driver may have running. Only one of
# these can claim the joint command interfaces at a time.
JOINT_CONTROLLERS = [
    "scaled_joint_trajectory_controller",
    "joint_trajectory_controller",
    "forward_velocity_controller",
    "forward_position_controller",
    "passthrough_trajectory_controller",
    "freedrive_mode_controller",
]

RECORDER_START_SERVICE = "/trajectory_recorder/start"
RECORDER_STOP_SERVICE = "/trajectory_recorder/stop"


def call_service(node, srv_type, name, request, timeout=5.0):
    """Blocking service call. Only use outside of callbacks (e.g. during startup)."""
    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            node.get_logger().error(f"Service {name} not available after {timeout:.0f}s")
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            node.get_logger().error(f"Service {name} timed out")
            return None
        return future.result()
    finally:
        node.destroy_client(client)


def activate_controller(node, controller, timeout=10.0):
    """Activate `controller`, deactivating any other joint controller that would conflict."""
    request = SwitchController.Request()
    request.activate_controllers = [controller]
    request.deactivate_controllers = [c for c in JOINT_CONTROLLERS if c != controller]
    request.strictness = SwitchController.Request.BEST_EFFORT
    request.activate_asap = True
    response = call_service(node, SwitchController, SWITCH_CONTROLLER_SERVICE, request, timeout)
    if response is None or not response.ok:
        node.get_logger().error(f"Failed to activate controller {controller}")
        return False
    node.get_logger().debug(f"Active joint controller: {controller}")
    return True


def servo_command_topic(node, timeout=10.0):
    """MoveIt Servo's command_out_topic, or "" if Servo can't be asked."""
    request = GetParameters.Request(names=["moveit_servo.command_out_topic"])
    response = call_service(node, GetParameters, SERVO_GET_PARAMETERS_SERVICE, request, timeout)
    return response.values[0].string_value if response is not None and response.values else ""


def servo_controller(node, timeout=10.0):
    """The joint controller MoveIt Servo's output goes to (from its command_out_topic),
    i.e. the one to activate for teleop. TELEOP_CONTROLLER if Servo can't be asked."""
    topic = servo_command_topic(node, timeout)
    if topic.startswith("/ur_emulator/"):
        return EMULATOR_CONTROLLER
    controller = topic.strip("/").split("/")[0]
    if controller in JOINT_CONTROLLERS:
        return controller
    node.get_logger().warn(f"Could not tell Servo's controller from '{topic}' - using {TELEOP_CONTROLLER}")
    return TELEOP_CONTROLLER


def set_servo_speed_limits(node, linear, angular, timeout=10.0):
    """Set MoveIt Servo's own Cartesian speed caps (m/s, rad/s).

    In speed_units mode Servo clamps every twist to scale.linear / scale.rotational
    (ur_servo.yaml: 0.6 m/s, 0.3 rad/s), on top of whatever the teleop node sends.
    Servo re-reads its parameters every cycle, so this takes effect immediately.
    """
    def double_param(name, value):
        return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                         double_value=float(value)))

    request = SetParameters.Request()
    request.parameters = [double_param("moveit_servo.scale.linear", linear),
                          double_param("moveit_servo.scale.rotational", angular)]
    response = call_service(node, SetParameters, SERVO_SET_PARAMETERS_SERVICE, request, timeout)
    if response is None or not all(r.successful for r in response.results):
        node.get_logger().error("Could not set MoveIt Servo speed limits - it stays at ur_servo.yaml values")
        return False
    node.get_logger().debug(f"MoveIt Servo speed limits: {linear:.2f} m/s, {angular:.2f} rad/s")
    return True


def set_servo_paused(node, paused, timeout=10.0):
    """Pause / unpause MoveIt Servo. Paused, it sends nothing to the teleop controller."""
    request = SetBool.Request()
    request.data = paused
    return call_service(node, SetBool, SERVO_PAUSE_SERVICE, request, timeout) is not None


def start_servo_twist_mode(node, timeout=30.0):
    """Unpause MoveIt Servo and put it in twist (Cartesian velocity) mode."""
    if not set_servo_paused(node, False, timeout):
        return False

    command_type = ServoCommandType.Request()
    command_type.command_type = ServoCommandType.Request.TWIST
    response = call_service(node, ServoCommandType, SERVO_COMMAND_TYPE_SERVICE, command_type, timeout)
    if response is None or not response.success:
        node.get_logger().error("MoveIt Servo refused to switch to TWIST commands")
        return False
    return True


class VelocityWatchdog:
    """Stops the arm if MoveIt Servo goes quiet while commanding joint velocities.

    With SERVO_OUTPUT "velocity", forward_velocity_controller (and the UR's speedj)
    keeps the last velocity until a new command arrives. Servo itself sends zeros when
    its input stops, but if Servo crashes nothing would. This watches Servo's output
    and publishes zeros once if it stops for VELOCITY_WATCHDOG_TIMEOUT mid-motion.
    Does nothing unless Servo outputs velocities."""

    def __init__(self, node):
        self.node = node
        self.topic = servo_command_topic(node)
        self.enabled = self.topic in (SERVO_VELOCITY_TOPIC, EMULATOR_VELOCITY_TOPIC)
        self.last_command = None    # node time (s) of Servo's last nonzero command
        if not self.enabled:
            return
        self.publisher = node.create_publisher(Float64MultiArray, self.topic, 10)
        node.create_subscription(Float64MultiArray, self.topic, self.on_command, 10)
        node.create_timer(0.01, self.check)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_command(self, msg):
        moving = any(abs(v) > 1e-9 for v in msg.data)
        self.last_command = self.now() if moving else None

    def check(self):
        if self.last_command is not None and self.now() - self.last_command > VELOCITY_WATCHDOG_TIMEOUT:
            self.last_command = None
            self.publisher.publish(Float64MultiArray(data=[0.0] * 6))
            self.node.get_logger().error(
                f"MoveIt Servo stopped sending joint velocities mid-motion - zeroed {self.topic}")


class RecorderClient:
    """Non-blocking start/stop calls to trajectory_recorder, safe to use from callbacks."""

    def __init__(self, node):
        self._node = node
        self._start = node.create_client(Trigger, RECORDER_START_SERVICE)
        self._stop = node.create_client(Trigger, RECORDER_STOP_SERVICE)
        self.recording = False

    def toggle(self):
        if self.recording:
            self.stop()
        else:
            self.start()

    def start(self):
        self._call(self._start, starting=True)

    def stop(self):
        self._call(self._stop, starting=False)

    def _call(self, client, starting):
        if not client.service_is_ready():
            self._node.get_logger().warn(
                "trajectory_recorder is not running - start it to record trajectories")
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._done(f, starting))

    def _done(self, future, starting):
        result = future.result()
        if result is None:
            return
        if result.success:
            self.recording = starting
            self._node.get_logger().info(result.message)
        else:
            self._node.get_logger().warn(result.message)
