"""Shared helpers for the UR5e teleoperation / trajectory nodes.

UR5e motion in ROS 2 goes through MoveIt Servo:

    keyboard / Touch node --TwistStamped--> /servo_node/delta_twist_cmds
    servo_node --Float64MultiArray--> /forward_position_controller/commands

Recorded trajectories are replayed through the scaled_joint_trajectory_controller,
so the active joint controller has to be swapped between teleop and playback.
"""

from controller_manager_msgs.srv import SwitchController
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool, Trigger
import rclpy

SERVO_TWIST_TOPIC = "/servo_node/delta_twist_cmds"
SERVO_COMMAND_TYPE_SERVICE = "/servo_node/switch_command_type"
SERVO_PAUSE_SERVICE = "/servo_node/pause_servo"

SWITCH_CONTROLLER_SERVICE = "/controller_manager/switch_controller"
TELEOP_CONTROLLER = "forward_position_controller"
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
    node.get_logger().info(f"Active joint controller: {controller}")
    return True


def start_servo_twist_mode(node, timeout=30.0):
    """Unpause MoveIt Servo and put it in twist (Cartesian velocity) mode."""
    pause = SetBool.Request()
    pause.data = False
    if call_service(node, SetBool, SERVO_PAUSE_SERVICE, pause, timeout) is None:
        return False

    command_type = ServoCommandType.Request()
    command_type.command_type = ServoCommandType.Request.TWIST
    response = call_service(node, ServoCommandType, SERVO_COMMAND_TYPE_SERVICE, command_type, timeout)
    if response is None or not response.success:
        node.get_logger().error("MoveIt Servo refused to switch to TWIST commands")
        return False
    return True


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
