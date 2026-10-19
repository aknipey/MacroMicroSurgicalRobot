"""Shared helpers for the UR5e teleoperation / trajectory nodes.

UR5e motion in ROS 2 goes through MoveIt Servo:

    keyboard / Touch node --TwistStamped--> /servo_node/delta_twist_cmds
    servo_node --JointTrajectory--> /scaled_joint_trajectory_controller/joint_trajectory
    (or, with SERVO_OUTPUT "velocity", Float64MultiArray joint velocities to
    /joint_velocity_limiter/commands -> /forward_velocity_controller/commands)

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
from scipy.signal import butter
from scipy.spatial.transform import Rotation
import numpy as np
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
# joint_velocity_limiter.py (velocity mode) sits between Servo and the velocity
# controller (or ur_emulator): Servo publishes here, the limiter's output_topic
# parameter says where the limited velocities go
LIMITER_NODE = "joint_velocity_limiter"
LIMITER_INPUT_TOPIC = f"/{LIMITER_NODE}/commands"
LIMITER_GET_PARAMETERS_SERVICE = f"/{LIMITER_NODE}/get_parameters"
UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
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


def string_parameter(node, service, name, timeout=10.0):
    """A string parameter of another node (through its get_parameters service), or ""."""
    request = GetParameters.Request(names=[name])
    response = call_service(node, GetParameters, service, request, timeout)
    return response.values[0].string_value if response is not None and response.values else ""


def servo_command_topic(node, timeout=10.0):
    """MoveIt Servo's command_out_topic, or "" if Servo can't be asked."""
    return string_parameter(node, SERVO_GET_PARAMETERS_SERVICE, "moveit_servo.command_out_topic", timeout)


def servo_output_topic(node, timeout=10.0):
    """The topic Servo's commands finally reach a controller (or ur_emulator) on: its
    command_out_topic, or joint_velocity_limiter's output_topic when Servo feeds the
    limiter."""
    topic = servo_command_topic(node, timeout)
    if topic == LIMITER_INPUT_TOPIC:
        topic = string_parameter(node, LIMITER_GET_PARAMETERS_SERVICE, "output_topic", timeout)
    return topic


def servo_controller(node, timeout=10.0):
    """The joint controller MoveIt Servo's output goes to (from its command_out_topic,
    through the velocity limiter if there is one), i.e. the one to activate for teleop.
    TELEOP_CONTROLLER if Servo can't be asked."""
    topic = servo_output_topic(node, timeout)
    if topic.startswith("/ur_emulator/"):
        return EMULATOR_CONTROLLER
    controller = topic.strip("/").split("/")[0]
    if controller in JOINT_CONTROLLERS:
        return controller
    node.get_logger().warn(f"Could not tell Servo's controller from '{topic}' - using {TELEOP_CONTROLLER}")
    return TELEOP_CONTROLLER


def double_param(name, value):
    return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                     double_value=float(value)))


def set_servo_singularity_thresholds(node, slowdown, stop, timeout=10.0):
    """Set the condition numbers at which MoveIt Servo slows / halts for a singularity
    (see SERVO_SINGULARITY_* in teleop_settings). Takes effect immediately."""
    request = SetParameters.Request()
    request.parameters = [double_param("moveit_servo.lower_singularity_threshold", slowdown),
                          double_param("moveit_servo.hard_stop_singularity_threshold", stop)]
    response = call_service(node, SetParameters, SERVO_SET_PARAMETERS_SERVICE, request, timeout)
    if response is None or not all(r.successful for r in response.results):
        node.get_logger().error("Could not set MoveIt Servo singularity thresholds")
        return False
    node.get_logger().debug(f"MoveIt Servo singularity thresholds: {slowdown:g} / {stop:g}")
    return True


def set_servo_speed_limits(node, linear, angular, timeout=10.0):
    """Set MoveIt Servo's own Cartesian speed caps (m/s, rad/s).

    In speed_units mode Servo clamps every twist to scale.linear / scale.rotational
    (ur_servo.yaml: 0.6 m/s, 0.3 rad/s), on top of whatever the teleop node sends.
    Servo re-reads its parameters every cycle, so this takes effect immediately.
    """
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
    its input stops, but if Servo (or joint_velocity_limiter) crashes nothing would.
    This watches the controller's command topic and publishes zeros once if it stops
    for VELOCITY_WATCHDOG_TIMEOUT mid-motion. Does nothing unless Servo outputs
    velocities."""

    def __init__(self, node):
        self.node = node
        self.topic = servo_output_topic(node)
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


class LowPass:
    """2nd-order Butterworth low-pass on a vector, sampled at a fixed rate."""

    def __init__(self, cutoff, rate, size):
        self.b, self.a = butter(2, cutoff, fs=rate)
        self.size = size
        self.reset()

    def reset(self, value=None):
        """Start at rest (zeros), or settled at value."""
        start = np.zeros(self.size) if value is None else np.asarray(value, dtype=float)
        self.inputs = np.array([start, start])      # previous two inputs, newest first
        self.outputs = np.array([start, start])     # previous two outputs, newest first

    def __call__(self, value):
        b, a = self.b, self.a
        out = (b[0] * value + b[1] * self.inputs[0] + b[2] * self.inputs[1]
               - a[1] * self.outputs[0] - a[2] * self.outputs[1])
        self.inputs = np.array([value, self.inputs[0]])
        self.outputs = np.array([out, self.outputs[0]])
        return out


def jacobian_derivatives(jacobian):
    """dJ/dq_k for each joint k of a serial chain of revolute joints, from its geometric
    Jacobian alone (columns [z_j x (p_ee - p_j); z_j], base to tip). Turning joint k
    rotates everything after it about z_k: for k <= j the column turns with it
    (dJ_j = [z_k x Jv_j; z_k x z_j]); for k > j only p_ee moves (dJv_j = z_j x Jv_k)."""
    linear, axes = jacobian[:3].T, jacobian[3:].T       # rows = joints
    n = len(axes)
    later = np.triu(np.ones((n, n), dtype=bool))[:, :, None]       # [k, j]: k <= j
    turned_linear = np.cross(axes[:, None], linear[None, :])        # [k, j] = z_k x Jv_j
    moved_linear = np.cross(axes[None, :], linear[:, None])         # [k, j] = z_j x Jv_k
    turned_axes = np.cross(axes[:, None], axes[None, :]) * later    # [k, j] = z_k x z_j
    derivatives = np.concatenate([np.where(later, turned_linear, moved_linear), turned_axes], axis=2)
    return derivatives.transpose(0, 2, 1)       # [k, row, j]


class SingularityGuard:
    """Keeps a Cartesian twist command away from arm singularities, and backs out of them.

    MoveIt Servo's own handling scales the *whole* twist to zero as soon as any part of
    it heads into a singularity, and past its hard stop halts every command, even one
    leading out, so the arm gets stuck. Before it gets there,
    J^-1 lets a small tool speed become a huge wrist speed (the wrist overspeed faults).

    This works on the twist before Servo, in joint space (q' = J^-1 v), with the
    condition number c = s_max / s_min of the same 6x6 Jacobian Servo uses:
      * c < slow: the twist passes unchanged.
      * above slow: damped least squares (damping grows from 0 to `damping` as s_min
        falls), so joint speeds stay bounded, and the part of q' that heads into the singularity is
        scaled down, from none at `slow` to all of it at `stop`. Motion along or away
        from the singularity is kept, so the tool slides along the boundary.
      * above stop: also an escape joint velocity straight away from it (0 at `stop`,
        `escape_gain` rad/s per unit of c/stop - 1, capped at `escape_speed`), so the
        arm settles at c = stop and backs out if it was ever pushed (or started) past it.
    The result goes back to a twist (v = J q'), which Servo turns into exactly that q'.

    `jacobian_fn(positions)` gives the 6x6 geometric Jacobian (linear rows first) for
    joint positions in base-to-tip order.
    """

    # One-sided joint offset (rad) for finding the way out where the gradient vanishes
    PROBE_STEP = 0.02

    def __init__(self, jacobian_fn, slow, stop, damping, escape_gain, escape_speed):
        if not 1.0 < slow < stop:
            raise ValueError(f"Singularity thresholds need 1 < slow < stop, not {slow}, {stop}")
        self.jacobian_fn = jacobian_fn
        self.slow, self.stop = slow, stop
        self.damping = damping
        self.escape_gain, self.escape_speed = escape_gain, escape_speed
        self.condition = 1.0
        self.limiting = 0.0     # 0 = untouched, 1 = all motion into the singularity removed
        self.escaping = False

    @staticmethod
    def measure(singular_values):
        """m = (sum s_i^-2)^-1/2: ~ the smallest singular value, 0 when singular."""
        return np.sum(np.maximum(singular_values, 1e-12) ** -2) ** -0.5

    def __call__(self, twist, positions):
        """Filtered copy of twist (6: linear, angular; same frame and point as the Jacobian)."""
        positions = np.asarray(positions, dtype=float)
        jacobian = self.jacobian_fn(positions)
        u, s, vt = np.linalg.svd(jacobian)
        self.condition = s[0] / s[-1] if s[-1] > 0.0 else np.inf
        self.limiting, self.escaping = 0.0, False
        if self.condition < self.slow:
            return twist

        # Damped least squares: damping from 0 at c = slow to `damping` at s_min = 0
        threshold = s[0] / self.slow
        damping_sq = self.damping ** 2 * (1.0 - (s[-1] / threshold) ** 2)
        joint_velocity = vt.T @ ((s / (s ** 2 + damping_sq)) * (u.T @ twist))

        # Joint-space direction away from the singularity: the gradient of
        # m = (sum s_i^-2)^-1/2 (ds_i = u_i^T dJ v_i). m ~ s_min, but stays smooth where
        # the two smallest s_i cross (two singularities at once, e.g. elbow + shoulder);
        # the gradient of s_min itself flips between them every cycle there.
        derivatives = jacobian_derivatives(jacobian)
        ds = np.diagonal(u.T @ derivatives @ vt.T, axis1=1, axis2=2)     # [k, i] = ds_i / dq_k
        safe = np.maximum(s, 1e-12)
        gradient = self.measure(s) ** 3 * (ds @ safe ** -3)
        norm_sq = gradient @ gradient
        if not norm_sq > 1e-12 and self.condition > self.stop:
            # Exactly on a singularity (e.g. mock hardware's all-zero start) m is like |x|
            # and the analytic gradient can cancel to 0: step each joint one way instead.
            # Scored by log(s_1 ... s_6), which rises when any one of two singularities
            # at once is relieved (m stays 0 until both are).
            def score(values):
                return np.sum(np.log(np.maximum(values, 1e-12)))
            base = score(s)
            gradient = np.array([
                score(np.linalg.svd(self.jacobian_fn(positions + self.PROBE_STEP * step), compute_uv=False))
                - base for step in np.eye(len(positions))]) / self.PROBE_STEP
            norm_sq = gradient @ gradient
        if norm_sq > 1e-12:
            self.limiting = min(1.0, (self.condition - self.slow) / (self.stop - self.slow))
            towards = gradient @ joint_velocity
            if towards < 0.0:
                joint_velocity -= self.limiting * towards / norm_sq * gradient
            if self.condition > self.stop:
                speed = min(self.escape_speed, self.escape_gain * (self.condition / self.stop - 1.0))
                joint_velocity += speed * gradient / np.sqrt(norm_sq)
                self.escaping = speed > 0.0
        return jacobian @ joint_velocity


def ur_singularity_name(positions):
    """Which UR singularity a joint configuration (UR_JOINTS order) is nearest."""
    wrist, elbow = abs(np.sin(positions[4])), abs(np.sin(positions[2]))
    if min(wrist, elbow) > 0.35:
        return "shoulder (wrist centre near the base axis)"
    return "wrist (wrist 2 near 0 / 180 deg)" if wrist < elbow else "elbow (arm stretched out)"


class RcmTool:
    """A straight tool shaft fixed to tool0, pivoting about a remote centre of motion.

    In the tool0 frame the shaft starts at `shaft_start`, runs along `shaft_direction`
    and ends at the tip `shaft_length` further on. With an RCM point c (base frame), the
    insertion depth is how far the tip is past c along the shaft, and the shaft must
    keep passing through c: of the tool's 6 DOF only the pivot (2), depth and roll
    about the shaft are left.
    """

    def __init__(self, shaft_start, shaft_direction, shaft_length):
        self.start = np.asarray(shaft_start, dtype=float)
        direction = np.asarray(shaft_direction, dtype=float)
        if self.start.shape != (3,) or direction.shape != (3,) or not np.linalg.norm(direction) > 1e-9:
            raise ValueError(f"RCM shaft needs a 3-vector start and a non-zero direction, "
                             f"not {shaft_start}, {shaft_direction}")
        if not shaft_length > 0.0:
            raise ValueError(f"RCM shaft length must be positive, not {shaft_length}")
        self.direction = direction / np.linalg.norm(direction)
        self.length = float(shaft_length)
        self.tip_offset = self.start + self.length * self.direction     # tool0 -> tip, tool0 frame
        # Tool0 orientation -> tip frame, whose z is the shaft axis
        self.tip_rotation = self.align(np.array([0.0, 0.0, 1.0]), self.direction)

    @staticmethod
    def align(a, b):
        """Smallest rotation turning unit vector a onto unit vector b."""
        axis = np.cross(a, b)
        sin, cos = np.linalg.norm(axis), float(a @ b)
        if sin < 1e-12:
            if cos > 0.0:
                return Rotation.identity()
            # Opposite: half a turn about any axis perpendicular to a
            perpendicular = np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
            return Rotation.from_rotvec(np.pi * perpendicular / np.linalg.norm(perpendicular))
        return Rotation.from_rotvec(axis / sin * np.arctan2(sin, cos))

    def shaft(self, position, rotation):
        """(start, unit axis, tip) in the base frame for a tool0 pose (position, Rotation)."""
        start = position + rotation.apply(self.start)
        axis = rotation.apply(self.direction)
        return start, axis, start + self.length * axis

    def depth(self, position, rotation, rcm):
        """How far the tip is past the RCM point along the shaft (m; < 0 = not reached)."""
        _, axis, tip = self.shaft(position, rotation)
        return float((tip - rcm) @ axis)

    def line_error(self, position, rotation, rcm):
        """Distance (m) from the shaft line to the RCM point - 0 when the constraint holds."""
        start, axis, _ = self.shaft(position, rotation)
        offset = rcm - start
        return float(np.linalg.norm(offset - (offset @ axis) * axis))

    def tool0_position(self, tip, rotation):
        """Tool0 position that puts the tip at `tip` with tool0 orientation `rotation`."""
        return tip - rotation.apply(self.tip_offset)

    @staticmethod
    def target_shaft(tip_target, rcm, axis_origin, min_pivot_depth, max_depth):
        """(unit axis, depth) for a wanted tip position, the shaft through the RCM point.

        The axis points from the RCM point towards the tip target, and the depth is
        their distance, except near the RCM point: there the sideways part (across
        axis_origin, the axis at engage) is faded out by (axial / min_pivot_depth)^2,
        so the pivot angle starts from 0 at depth 0 and grows smoothly into the plain
        direction-to-target at min_pivot_depth. A target behind the RCM point gives
        depth 0 along axis_origin; the depth is capped at max_depth.
        """
        offset = tip_target - rcm
        axial = float(offset @ axis_origin)
        if axial <= 0.0:
            return axis_origin, 0.0
        sideways = offset - axial * axis_origin
        if axial < min_pivot_depth:
            sideways = sideways * (axial / min_pivot_depth) ** 2
        offset = axial * axis_origin + sideways
        depth = np.linalg.norm(offset)
        return offset / depth, min(depth, max_depth)

    def pivot(self, position, rotation, rcm):
        """(point of the shaft line nearest the RCM point, unit shaft axis), base frame."""
        start, axis, _ = self.shaft(position, rotation)
        return start + ((rcm - start) @ axis) * axis, axis

    def twist(self, position, rotation, rcm, insertion_speed, angular, rcm_gain, max_correction, lead=0.0):
        """Tool0 linear velocity (base frame) that turns the tool at `angular` about the RCM
        point and slides it along the shaft at insertion_speed (> 0 = deeper).

        Drift of the shaft off the RCM point is pulled back across the shaft at rcm_gain
        (capped at max_correction m/s). The twist reaches the arm `lead` s later, by when
        the tool has turned and slid on; it is built for the pose predicted then, as
        built for the current one the lever arm and shaft axis are stale and the shaft
        drifts outwards on fast pivots (~|angular|^2 |lever| lead, plus insertion x
        |angular| x lead).
        """
        pivot, axis = self.pivot(position, rotation, rcm)
        drift = rcm_gain * (rcm - pivot)            # across the shaft only
        norm = np.linalg.norm(drift)
        if norm > max_correction:
            drift *= max_correction / norm
        turn = Rotation.from_rotvec(lead * angular)
        position = pivot + turn.apply(position - pivot) + lead * insertion_speed * axis
        pivot, axis = self.pivot(position, turn * rotation, rcm)
        return insertion_speed * axis + drift + np.cross(angular, position - pivot)


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
