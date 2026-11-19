#!/usr/bin/env python3
"""Touch stylus teleoperation of the UR16e through MoveIt Servo.

WHITE is the clutch: while held, tool0 follows the stylus motion since the press.
Double-click WHITE cycles the motion scale (`scale_steps`). A GREY click starts or stops
recording; holding GREY for `mode_switch_hold_time` switches between this arm (macro) and
micro_module_control (micro), published latched on `mode_topic`.

WHITE + GREY latches the tool tip as a remote centre of motion (RCM): the stylus then drives
the tip, the arm pivots the shaft about the point, and pen twist rolls the tool. The same
chord, with the tool retracted, leaves RCM mode. The RCM tool comes from the tool library
(`tool_library`, switch with the `tool` parameter).

Also: a singularity guard on the arm Jacobian's condition number (`singularity_guard`),
haptic cues on the stylus (`haptics`), and the target and RCM state on ~/target_pose,
~/motion_scale, ~/condition_number, ~/rcm_* for the logger.

Mapping "toward_base" (default): up is up, towards the Touch body moves the tool towards the
robot base. "fixed" uses `axis_map`. Defaults are in macro_micro_robot/teleop_settings.py.
"""

import signal
import socket
import struct
import time
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from omni_msgs.msg import OmniButtonEvent, OmniFeedback, OmniState
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from macro_micro_robot.ur16e_common import (
    SERVO_TWIST_TOPIC,
    LowPass,
    RcmTool,
    RecorderClient,
    SingularityGuard,
    VelocityWatchdog,
    activate_controller,
    servo_controller,
    set_servo_singularity_thresholds,
    set_servo_speed_limits,
    start_servo_twist_mode,
    ur_singularity_name,
)
from macro_micro_robot import teleop_settings as cfg
from macro_micro_robot import tools as tool_library
from macro_micro_robot.haptics import PulsePlayer, limit_force, singularity_force

# omni_state publishes orientation = R_device * Rz(-90) (hduMatrix is row-vector /
# transposed, so its "offset * rotation" is a right multiply), so the offset cancels
# in R * R0^-1 and the stylus delta comes out in device axes (x right, y up, z
# towards the operator). Its position axes are those rotated by +90 deg about X.
TOUCH_POSITION_FRAME = Rotation.from_euler("x", 90, degrees=True)

PROGRAM_RUNNING_TOPIC = "/io_and_status_controller/robot_program_running"

# The pen runs along z of the published orientation (pointing from the tip to the
# back of the pen); tool0 z points out of the flange towards the tool tip, so the
# same visual twist is a rotation of opposite sign.
PEN_AXIS_TO_TOOL_Z_SIGN = -1.0


class UrdfChain:
    """Forward kinematics base_frame -> ee_frame from a URDF string, so the tool pose can
    be computed from each /joint_states message (500 Hz) instead of waiting for
    robot_state_publisher's TF (20 Hz, so up to 50 ms old)."""

    def __init__(self, urdf, base_frame, ee_frame):
        joints = {}     # child link -> (joint name, type, parent, origin (xyz, Rotation), axis)
        for joint in ET.fromstring(urdf).findall("joint"):     # not the ros2_control ones
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            axis = joint.find("axis")
            joints[joint.find("child").get("link")] = (
                joint.get("name"), joint.get("type"), joint.find("parent").get("link"),
                (np.array(xyz), Rotation.from_euler("xyz", rpy)),
                np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None else np.array([0, 0, 1.0]))
        self.chain = []
        link = ee_frame
        while link != base_frame:
            if link not in joints:
                raise ValueError(f"No joint chain from {base_frame} to {ee_frame} in the URDF")
            self.chain.insert(0, joints[link])
            link = joints[link][2]
        self.joint_names = [j[0] for j in self.chain if j[1] in ("revolute", "continuous")]
        self.matrices = [(xyz, rpy.as_matrix()) for _, _, _, (xyz, rpy), _ in self.chain]

    def jacobian(self, positions):
        """Geometric Jacobian (6 x joints: linear velocity of the ee_frame origin, then
        angular velocity, both in base_frame) for {joint name: angle} - the one MoveIt
        Servo uses. Plain numpy (~0.1 ms), as it runs every update."""
        position, rotation, columns = np.zeros(3), np.eye(3), []
        for (name, kind, _, _, axis), (xyz, rpy) in zip(self.chain, self.matrices):
            position = position + rotation @ xyz
            rotation = rotation @ rpy
            if kind in ("revolute", "continuous"):
                columns.append((position, rotation @ axis))
                # Rodrigues: rotation about the (unit) joint axis
                k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
                angle = positions[name]
                rotation = rotation @ (np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k))
        return np.array([np.concatenate([np.cross(z, position - p), z]) for p, z in columns]).T

    def pose(self, positions):
        """(position, Rotation) of ee_frame in base_frame for {joint name: angle}."""
        position, rotation = np.zeros(3), Rotation.identity()
        for name, kind, _, (xyz, rpy), axis in self.chain:
            position = position + rotation.apply(xyz)
            rotation = rotation * rpy
            if kind in ("revolute", "continuous"):
                rotation = rotation * Rotation.from_rotvec(axis * positions[name])
        return position, rotation


def axis_map_to_matrix(axis_map):
    """['y', '-x', 'z'] -> 3x3 matrix M such that robot_vector = M @ touch_vector."""
    if len(axis_map) != 3:
        raise ValueError("axis_map needs exactly 3 entries")
    matrix = np.zeros((3, 3))
    for row, entry in enumerate(axis_map):
        entry = entry.strip().lower()
        sign = -1.0 if entry.startswith("-") else 1.0
        matrix[row, "xyz".index(entry.lstrip("+-"))] = sign
    if not np.isclose(np.linalg.det(matrix), 1.0):
        raise ValueError(f"axis_map {axis_map} is a reflection, not a rotation - flip one sign")
    return matrix


def toward_base_matrix(tool_position):
    """Columns = robot-frame directions for stylus right, towards the Touch body, up.

    Towards the Touch body maps to horizontally from the tool towards the robot
    base; None if the tool is (nearly) above the base and that is undefined.
    """
    forward = -np.array([tool_position[0], tool_position[1], 0.0])
    if np.linalg.norm(forward) < 0.05:
        return None
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    return np.column_stack([np.cross(forward, up), forward, up])


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def split_twist(rotation):
    """Split a body-frame rotation into (swing, twist angle about body z): rotation = swing * Rz(twist)."""
    x, y, z, w = rotation.as_quat()
    twist = 2.0 * np.arctan2(z, w)
    return rotation * Rotation.from_rotvec([0.0, 0.0, -twist]), wrap_angle(twist)


def clamp_norm(vector, limit):
    norm = np.linalg.norm(vector)
    return vector * (limit / norm) if norm > limit else vector


class ConstantVelocityKalman:
    """Kalman filter for a stylus position (3-vector) or orientation (scipy Rotation)
    moving at a roughly constant velocity.

    omni_state publishes at 1 kHz, but the pose only changes every ~4 ms (up to 12 ms);
    the samples in between repeat the last one. Differencing that staircase at the
    teleop rate makes the feedforward velocity jump with how many real updates fell
    in each window. This filter is fed only the real updates and predicts in
    between, so its estimate moves smoothly and does not lag the stylus.
    For an orientation the error is the rotation vector between measurement and
    estimate, applied back multiplicatively (an error-state / extended Kalman filter).
    All three axes share one 2x2 covariance: same noise, same update times.
    """

    # A sample repeated for this long (s) means the stylus has stopped, so it is used as
    # a measurement too; while it moves, real updates arrive every ~4 ms.
    HOLD_TIME = 0.008

    def __init__(self, accel_noise, measurement_noise, rotation=False):
        self.accel_noise = accel_noise              # process noise, (unit/s^2)^2 * s
        self.measurement_variance = measurement_noise ** 2
        self.rotation = rotation
        self.clear()

    def clear(self):
        """Forget the estimate; the next sample starts the filter again at rest."""
        self.value = None
        self.raw = None

    def feed(self, value, raw, stamp):
        """Update on a new sample (raw, an array, changed) or a repeat after HOLD_TIME,
        then return the estimate at stamp."""
        if self.value is None or not np.array_equal(raw, self.raw) or stamp - self.stamp >= self.HOLD_TIME:
            self.update(value, stamp)
            self.raw = raw
        return self.predict(stamp)

    def reset(self, value, stamp):
        self.value, self.velocity, self.stamp = value, np.zeros(3), stamp
        self.covariance = np.diag([self.measurement_variance, 1.0])

    def update(self, value, stamp):
        if self.value is None:
            self.reset(value, stamp)
            return
        dt = max(stamp - self.stamp, 0.0)
        self.value, self.stamp = self.predict(stamp), stamp
        transition = np.array([[1.0, dt], [0.0, 1.0]])
        process = self.accel_noise * np.array([[dt ** 3 / 3.0, dt ** 2 / 2.0], [dt ** 2 / 2.0, dt]])
        self.covariance = transition @ self.covariance @ transition.T + process
        gain = self.covariance[:, 0] / (self.covariance[0, 0] + self.measurement_variance)
        error = (value * self.value.inv()).as_rotvec() if self.rotation else value - self.value
        self.value = self.apply(self.value, gain[0] * error)
        self.velocity = self.velocity + gain[1] * error
        self.covariance = self.covariance - np.outer(gain, self.covariance[0])

    def predict(self, stamp):
        """Estimate at stamp (s), extrapolated from the last update at constant velocity."""
        return self.apply(self.value, self.velocity * (stamp - self.stamp))

    def apply(self, value, step):
        return Rotation.from_rotvec(step) * value if self.rotation else value + step


class TouchTeleop(Node):
    def __init__(self):
        super().__init__("ur16e_touch_teleop")

        self.declare_parameter("state_topic", "/phantom/state")
        self.declare_parameter("button_topic", "/phantom/button")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("mapping", cfg.TOUCH_MAPPING)  # or "fixed" (uses axis_map)
        self.declare_parameter("axis_map", cfg.TOUCH_AXIS_MAP)
        self.declare_parameter("touch_units_to_m", cfg.TOUCH_UNITS_TO_M)
        self.declare_parameter("translation_scale", cfg.TOUCH_TRANSLATION_SCALE)
        self.declare_parameter("scale_steps", cfg.TOUCH_SCALE_STEPS)
        self.declare_parameter("double_click_time", cfg.TOUCH_DOUBLE_CLICK_TIME)
        self.declare_parameter("mode_switch_hold_time", cfg.TOUCH_MODE_SWITCH_HOLD_TIME)
        self.declare_parameter("mode_topic", "/teleop_mode")
        self.declare_parameter("scale_rotation_exponent", cfg.TOUCH_SCALE_ROTATION_EXPONENT)
        self.declare_parameter("enable_rotation", cfg.TOUCH_ENABLE_ROTATION)
        self.declare_parameter("rotation_scale", cfg.TOUCH_ROTATION_SCALE)
        self.declare_parameter("enable_twist", cfg.TOUCH_ENABLE_TWIST)
        self.declare_parameter("twist_scale", cfg.TOUCH_TWIST_SCALE)
        self.declare_parameter("position_gain", cfg.TOUCH_POSITION_GAIN)
        self.declare_parameter("rotation_gain", cfg.TOUCH_ROTATION_GAIN)
        self.declare_parameter("max_correction_speed", cfg.TOUCH_MAX_CORRECTION_SPEED)
        self.declare_parameter("max_correction_angular_speed", cfg.TOUCH_MAX_CORRECTION_ANGULAR_SPEED)
        self.declare_parameter("feedforward", cfg.TOUCH_FEEDFORWARD)
        self.declare_parameter("feedforward_filter", cfg.TOUCH_FEEDFORWARD_FILTER)
        self.declare_parameter("kalman", cfg.TOUCH_KALMAN)
        self.declare_parameter("kalman_accel_noise", cfg.TOUCH_KALMAN_ACCEL_NOISE)
        self.declare_parameter("kalman_position_noise", cfg.TOUCH_KALMAN_POSITION_NOISE)
        self.declare_parameter("kalman_angular_accel_noise", cfg.TOUCH_KALMAN_ANGULAR_ACCEL_NOISE)
        self.declare_parameter("kalman_rotation_noise", cfg.TOUCH_KALMAN_ROTATION_NOISE)
        self.declare_parameter("command_filter", cfg.TOUCH_COMMAND_FILTER)
        self.declare_parameter("loop_delay", cfg.TOUCH_LOOP_DELAY)
        self.declare_parameter("max_linear_speed", cfg.TOUCH_MAX_LINEAR_SPEED)
        self.declare_parameter("max_angular_speed", cfg.TOUCH_MAX_ANGULAR_SPEED)
        self.declare_parameter("deadband", cfg.TOUCH_DEADBAND)
        self.declare_parameter("touch_timeout", cfg.TOUCH_TIMEOUT)
        self.declare_parameter("publish_rate", cfg.TOUCH_PUBLISH_RATE)
        self.declare_parameter("pose_source", cfg.TOUCH_POSE_SOURCE)
        self.declare_parameter("singularity_guard", cfg.TOUCH_SINGULARITY_GUARD)
        self.declare_parameter("singularity_slow", cfg.TOUCH_SINGULARITY_SLOW)
        self.declare_parameter("singularity_stop", cfg.TOUCH_SINGULARITY_STOP)
        self.declare_parameter("singularity_damping", cfg.TOUCH_SINGULARITY_DAMPING)
        self.declare_parameter("singularity_escape_gain", cfg.TOUCH_SINGULARITY_ESCAPE_GAIN)
        self.declare_parameter("singularity_escape_speed", cfg.TOUCH_SINGULARITY_ESCAPE_SPEED)
        self.declare_parameter("rcm_shaft_start", cfg.TOUCH_RCM_SHAFT_START)
        self.declare_parameter("rcm_shaft_direction", cfg.TOUCH_RCM_SHAFT_DIRECTION)
        self.declare_parameter("rcm_shaft_length", cfg.TOUCH_RCM_SHAFT_LENGTH)
        self.declare_parameter("rcm_depth_margin", cfg.TOUCH_RCM_DEPTH_MARGIN)
        self.declare_parameter("tool_library", cfg.TOOL_LIBRARY)
        self.declare_parameter("tool", 0)       # library number; 0 = the library's default
        self.declare_parameter("tool_switch_max_joint_speed", cfg.TOOL_SWITCH_MAX_JOINT_SPEED)
        self.declare_parameter("tool_switch_still_time", cfg.TOOL_SWITCH_STILL_TIME)
        self.declare_parameter("rcm_pivot_fade_depth", cfg.TOUCH_RCM_PIVOT_FADE_DEPTH)
        self.declare_parameter("rcm_pivot_fade_strength", cfg.TOUCH_RCM_PIVOT_FADE_STRENGTH)
        self.declare_parameter("rcm_tool0_gain", cfg.TOUCH_RCM_TOOL0_GAIN)
        self.declare_parameter("rcm_tool0_gain_depth", cfg.TOUCH_RCM_TOOL0_GAIN_DEPTH)
        self.declare_parameter("rcm_retract_depth", cfg.TOUCH_RCM_RETRACT_DEPTH)
        self.declare_parameter("rcm_exit_depth", cfg.TOUCH_RCM_EXIT_DEPTH)
        self.declare_parameter("rcm_gain", cfg.TOUCH_RCM_GAIN)
        self.declare_parameter("rcm_max_error", cfg.TOUCH_RCM_MAX_ERROR)
        self.declare_parameter("force_topic", "/phantom/force_feedback")
        # touch_server (macro_micro_robot touch_driver.launch.py) renders the RCM walls itself
        self.declare_parameter("haptic_scene_port", 47102)
        self.declare_parameter("haptics", cfg.TOUCH_HAPTICS)
        self.declare_parameter("haptic_max_force", cfg.TOUCH_HAPTIC_MAX_FORCE)
        self.declare_parameter("haptic_slew_rate", cfg.TOUCH_HAPTIC_SLEW_RATE)
        self.declare_parameter("haptic_wall_stiffness", cfg.TOUCH_HAPTIC_WALL_STIFFNESS)
        self.declare_parameter("haptic_crossing_force", cfg.TOUCH_HAPTIC_CROSSING_FORCE)
        self.declare_parameter("haptic_crossing_hysteresis", cfg.TOUCH_HAPTIC_CROSSING_HYSTERESIS)
        self.declare_parameter("haptic_crossing_holdoff", cfg.TOUCH_HAPTIC_CROSSING_HOLDOFF)
        self.declare_parameter("haptic_limit_force", cfg.TOUCH_HAPTIC_LIMIT_FORCE)
        self.declare_parameter("haptic_pulse_force", cfg.TOUCH_HAPTIC_PULSE_FORCE)
        self.declare_parameter("haptic_pulse_frequency", cfg.TOUCH_HAPTIC_PULSE_FREQUENCY)
        self.declare_parameter("haptic_singularity_force", cfg.TOUCH_HAPTIC_SINGULARITY_FORCE)
        self.declare_parameter("haptic_singularity_damping", cfg.TOUCH_HAPTIC_SINGULARITY_DAMPING)
        self.declare_parameter("haptic_singularity_filter", cfg.TOUCH_HAPTIC_SINGULARITY_FILTER)
        self.declare_parameter("setup_servo", True)

        p = lambda name: self.get_parameter(name).value
        self.base_frame = p("base_frame")
        self.ee_frame = p("ee_frame")
        self.mapping = p("mapping")
        if self.mapping not in ("toward_base", "fixed"):
            raise ValueError(f"mapping must be 'toward_base' or 'fixed', not {self.mapping!r}")
        self.fixed_axis_matrix = axis_map_to_matrix(p("axis_map"))
        self.units = p("touch_units_to_m")
        self.base_translation_scale = p("translation_scale")
        self.scale_steps = [float(v) for v in p("scale_steps")]
        if not self.scale_steps or min(self.scale_steps) <= 0.0:
            raise ValueError(f"scale_steps must be positive, not {self.scale_steps}")
        self.double_click_time = p("double_click_time")
        self.mode_switch_hold_time = p("mode_switch_hold_time")
        self.scale_rotation_exponent = p("scale_rotation_exponent")
        self.enable_rotation = p("enable_rotation")
        self.base_rotation_scale = p("rotation_scale")
        self.enable_twist = p("enable_twist")
        self.base_twist_scale = p("twist_scale")
        self.position_gain = p("position_gain")
        self.rotation_gain = p("rotation_gain")
        # Caps on the gain terms alone (0 = none), so catching up after the robot fell
        # behind (a speed cap or the joint limiter held it back) is gradual, not a surge
        self.max_correction = p("max_correction_speed") or np.inf
        self.max_angular_correction = p("max_correction_angular_speed") or np.inf
        self.feedforward = p("feedforward")
        self.feedforward_filter = p("feedforward_filter")
        # Kalman filters on the raw stylus pose (metres / Rotation), or None when off
        self.position_kalman = self.orientation_kalman = None
        if p("kalman"):
            self.position_kalman = ConstantVelocityKalman(
                p("kalman_accel_noise"), p("kalman_position_noise"))
            self.orientation_kalman = ConstantVelocityKalman(
                p("kalman_angular_accel_noise"), p("kalman_rotation_noise"), rotation=True)
        self.loop_delay = p("loop_delay")
        # Low-pass on the outgoing twist (linear + angular), or None when off
        command_filter, publish_rate = p("command_filter"), p("publish_rate")
        if command_filter >= publish_rate / 2.0:
            raise ValueError(f"command_filter ({command_filter} Hz) must be below half "
                             f"publish_rate ({publish_rate} Hz)")
        self.command_filter = LowPass(command_filter, publish_rate, 6) if command_filter > 0.0 else None
        self.max_linear = p("max_linear_speed")
        self.max_angular = p("max_angular_speed")
        self.base_deadband = p("deadband")
        self.touch_timeout = p("touch_timeout")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_source = p("pose_source")
        if self.pose_source not in ("tf", "joint_states"):
            raise ValueError(f"pose_source must be 'tf' or 'joint_states', not {self.pose_source!r}")
        self.urdf_chain = None
        self.joint_positions = None     # {name: angle} from the newest /joint_states
        if self.pose_source == "joint_states":
            latched_description = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, "/robot_description", self.on_robot_description, latched_description)
        # Joint speeds too, for the tool switch's standstill check
        self.joint_states_time = None   # node clock (s) of the newest /joint_states
        self.last_motion_time = None    # newest /joint_states with a joint over the switch speed
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        # Needs the joint angles, so only with pose_source "joint_states"
        self.singularity_guard = None
        if p("singularity_guard"):
            if self.pose_source == "joint_states":
                self.singularity_guard = SingularityGuard(
                    self.jacobian_at, p("singularity_slow"), p("singularity_stop"), p("singularity_damping"),
                    p("singularity_escape_gain"), p("singularity_escape_speed"))
            else:
                self.get_logger().warn("singularity_guard needs pose_source 'joint_states' - "
                                       "only MoveIt Servo's own singularity check is active")
        self.singularity_state = 0      # 0 clear, 1 limiting, 2 backing out (for the log messages)

        # The RCM tool (rcm_tool, rcm_max_depth) is set by select_tool once the publishers exist
        self.rcm_depth_margin = p("rcm_depth_margin")
        self.tool_library = p("tool_library")
        self.tool_switch_speed = p("tool_switch_max_joint_speed")
        self.tool_switch_still_time = p("tool_switch_still_time")
        self.tool_number = None
        self.rcm_pivot_fade_depth = p("rcm_pivot_fade_depth")
        self.rcm_pivot_fade_strength = p("rcm_pivot_fade_strength")
        if not self.rcm_pivot_fade_strength >= 1.0:
            raise ValueError(f"rcm_pivot_fade_strength must be >= 1, not {self.rcm_pivot_fade_strength}")
        self.rcm_tool0_gain = p("rcm_tool0_gain")
        self.rcm_tool0_gain_depth = p("rcm_tool0_gain_depth")
        self.rcm_min_depth = -p("rcm_retract_depth")
        if self.rcm_min_depth > 0.0:
            raise ValueError(f"rcm_retract_depth must be >= 0, not {-self.rcm_min_depth}")
        self.rcm_exit_depth = p("rcm_exit_depth")
        self.rcm_gain = p("rcm_gain")
        self.rcm_max_error = p("rcm_max_error")

        # Haptic cues; all forces in the stylus position axes (N)
        self.force_pub = None
        if p("haptics"):
            if p("haptic_pulse_frequency") >= publish_rate / 2.0:
                raise ValueError(f"haptic_pulse_frequency ({p('haptic_pulse_frequency')} Hz) must be "
                                 f"below half publish_rate ({publish_rate} Hz)")
            self.force_pub = self.create_publisher(OmniFeedback, p("force_topic"), 10)
        self.haptic_max_force = p("haptic_max_force")
        self.publish_period = 1.0 / publish_rate
        self.haptic_max_step = p("haptic_slew_rate") * self.publish_period
        self.wall_stiffness = p("haptic_wall_stiffness")
        self.scene_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.scene_address = ("127.0.0.1", p("haptic_scene_port"))
        self.scene_active = False
        self.crossing_force = p("haptic_crossing_force")
        self.crossing_hysteresis = p("haptic_crossing_hysteresis")
        self.crossing_side = None                   # +1 / -1: tip target past / short of the RCM point
        self.crossing_holdoff = p("haptic_crossing_holdoff")
        self.last_crossing_click = -1e9
        self.limit_force = p("haptic_limit_force")
        self.limit_side = None                      # +1 past deepest / -1 past retract / 0 between
        self.last_limit_click = -1e9
        self.pulses = PulsePlayer(p("haptic_pulse_force"), p("haptic_pulse_frequency"))
        self.singularity_push = p("haptic_singularity_force")
        self.singularity_push_damping = p("haptic_singularity_damping")
        self.singularity_push_filter = p("haptic_singularity_filter")
        self.haptic_force = np.zeros(3)             # last force sent
        self.singularity_force = np.zeros(3)        # low-passed singularity push
        self.singularity_cue = None                 # (limiting, twist_gradient) this cycle
        self.touch_velocity = np.zeros(3)           # m/s, stylus frame (driver's estimate)

        self.twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, 10)
        self.target_pub = self.create_publisher(PoseStamped, "~/target_pose", 100)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scale_pub = self.create_publisher(Float64, "~/motion_scale", latched)
        self.condition_pub = self.create_publisher(Float64, "~/condition_number", 10)
        self.rcm_active_pub = self.create_publisher(Bool, "~/rcm_active", latched)
        self.mode_pub = self.create_publisher(String, p("mode_topic"), latched)
        self.rcm_point_pub = self.create_publisher(PointStamped, "~/rcm_point", latched)
        self.rcm_error_pub = self.create_publisher(Float64, "~/rcm_error", 10)
        self.insertion_depth_pub = self.create_publisher(Float64, "~/insertion_depth", 10)
        self.tool_tip_pub = self.create_publisher(PoseStamped, "~/tool_tip", 10)
        self.active_tool_pub = self.create_publisher(String, "~/active_tool", latched)
        self.select_tool(p("tool"))
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.recorder = RecorderClient(self)

        self.touch_position = None      # metres, stylus frame
        self.touch_orientation = None   # scipy Rotation, as published
        self.touch_stamp = None
        self.white_pressed = False
        self.grey_pressed = False
        # Double-click detection on WHITE (node clock, s)
        self.white_press_time = None    # current press; None once it was used by a double click
        self.white_click_time = None    # start of the last short press-and-release
        # GREY: a click records, a long hold without WHITE switches module
        self.grey_press_time = None     # current press; None once WHITE spoiled it or it switched
        self.mode = "macro"             # which module the stylus drives: "macro" or "micro"

        self.scale_index = 0
        self.set_scale(0)

        # Captured when the white button goes down
        self.clutched = False
        self.touch_origin = None
        self.robot_origin = None
        self.axis_matrix = None         # robot_vector = axis_matrix @ touch_vector
        self.axis_rotation = None
        self.pen_twist = 0.0            # rad since engage, unwrapped so it can pass +-180 deg
        self.target = None              # (position, Rotation or None): tool0 pose to pull towards
        # Feedforward: target as seen by the previous update, and its filtered velocity
        self.previous_target = None
        self.previous_update = None
        self.target_history = deque()   # (time, target) for the loop-delay comparison
        self.target_linear_velocity = np.zeros(3)
        self.target_angular_velocity = np.zeros(3)
        self.target_depth_velocity = 0.0

        # RCM mode: the latched point (base frame) or None when off; tip and shaft axis
        # captured when WHITE goes down
        self.rcm_point = None
        self.rcm_fault = False          # shaft beyond rcm_max_error: holding still
        self.tip_origin = None
        self.shaft_axis_origin = None
        self.rcm_active_pub.publish(Bool(data=False))
        self.mode_pub.publish(String(data=self.mode))
        self.send_zero_force()      # clear a force left on by a node that died

        self.create_subscription(OmniState, p("state_topic"), self.on_state, 1)
        self.create_subscription(OmniButtonEvent, p("button_topic"), self.on_button, 10)
        # Stands in for the driver's INFO messages, which the launch file hides
        self.program_running = None
        self.create_subscription(Bool, PROGRAM_RUNNING_TOPIC, self.on_program_running, latched)

        if p("setup_servo"):
            activate_controller(self, servo_controller(self))
            self.velocity_watchdog = VelocityWatchdog(self)
            if not start_servo_twist_mode(self):
                self.get_logger().error(
                    "Could not configure MoveIt Servo - is ur16e_teleop.launch.py running?")
            set_servo_speed_limits(self, self.max_linear, self.max_angular)
            # With the guard on, Servo's own check would only halt the arm where the guard
            # is trying to back it out; without it, Servo's check is all there is
            if self.singularity_guard is not None:
                set_servo_singularity_thresholds(
                    self, cfg.SERVO_SINGULARITY_SLOWDOWN_GUARDED, cfg.SERVO_SINGULARITY_STOP_GUARDED)
            else:
                set_servo_singularity_thresholds(self, cfg.SERVO_SINGULARITY_SLOWDOWN, cfg.SERVO_SINGULARITY_STOP)

        self.create_timer(1.0 / p("publish_rate"), self.update)
        self.get_logger().info(
            "Touch teleop ready: hold WHITE to move, double-click WHITE to change scale, "
            "click GREY to start/stop recording, hold WHITE + click GREY for RCM mode, "
            f"hold GREY {self.mode_switch_hold_time:.1f} s to switch macro/micro "
            f"(tilt {'on' if self.enable_rotation else 'off'}, twist {'on' if self.enable_twist else 'off'}, "
            f"max {self.max_linear:.2f} m/s)")

    def on_program_running(self, msg):
        if msg.data == self.program_running:
            return
        if msg.data:
            self.get_logger().info("Robot program running - ready")
        elif self.program_running is None:
            self.get_logger().warn("Robot program not running - press play on the teach pendant")
        else:
            self.get_logger().warn("Robot program stopped - the robot will not move until it is restarted")
        self.program_running = msg.data

    def on_state(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        position = np.array([pos.x, pos.y, pos.z]) * self.units
        quat = np.array([ori.x, ori.y, ori.z, ori.w])
        orientation = Rotation.from_quat(quat) if np.linalg.norm(quat) > 1e-6 else None
        if self.position_kalman is None:
            self.touch_position, self.touch_orientation = position, orientation
        else:
            if not self.touch_is_fresh():   # first sample, or after a gap: start again at rest
                self.position_kalman.clear()
                self.orientation_kalman.clear()
            now = self.get_clock().now()
            stamp = (Time.from_msg(msg.header.stamp).nanoseconds or now.nanoseconds) * 1e-9
            self.touch_position = self.position_kalman.feed(position, position, stamp)
            self.touch_orientation = (None if orientation is None
                                      else self.orientation_kalman.feed(orientation, quat, stamp))
        self.touch_stamp = self.get_clock().now()
        # For the haptic damping. The driver's estimate is 20 Hz low-passed, and damping on
        # it lags enough at ~25 Hz to push along the motion and shake the pen, so the
        # Kalman filter's (no lag) is used when it is on
        if self.position_kalman is not None and self.position_kalman.value is not None:
            self.touch_velocity = self.position_kalman.velocity.copy()
        else:
            self.touch_velocity = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z]) * self.units
        if self.clutched:
            self.target = self.compute_target()
            self.publish_target(msg.header.stamp)

    def on_button(self, msg):
        white, grey = bool(msg.white_button), bool(msg.grey_button)
        now = self.get_clock().now().nanoseconds * 1e-9
        if grey and not self.grey_pressed:
            # With WHITE held GREY is the RCM chord, so a chord never records or switches
            self.grey_press_time = None if white else now
            if white and self.mode == "macro":
                self.toggle_rcm()
        if white:
            self.grey_press_time = None     # touching WHITE spoils a GREY click or hold
        elif not grey and self.grey_pressed:
            # A release that beats the update timer past the hold time still switches
            if not self.check_mode_switch(now) and self.grey_press_time is not None:
                self.recorder.toggle()
            self.grey_press_time = None
        if self.mode == "micro":
            pass    # WHITE drives the micro module, not this arm
        elif white and not self.white_pressed:
            self.white_press_time = now
            if self.white_click_time is not None and now - self.white_click_time <= self.double_click_time:
                self.set_scale(self.scale_index + 1)
                # This press is used up: a triple click is one scale change, not two
                self.white_press_time = self.white_click_time = None
            self.engage()   # after set_scale, so this stroke already uses the new scale
        elif not white and self.white_pressed:
            short = self.white_press_time is not None and now - self.white_press_time <= self.double_click_time
            self.white_click_time = self.white_press_time if short else None
            self.release()
        self.white_pressed, self.grey_pressed = white, grey

    def check_mode_switch(self, now):
        """Switch module once GREY has been held long enough on its own; True if it did."""
        if self.grey_press_time is None or now - self.grey_press_time < self.mode_switch_hold_time:
            return False
        self.grey_press_time = None     # used up: its release does not record
        self.release()
        self.white_click_time = None
        self.mode = "micro" if self.mode == "macro" else "macro"
        self.mode_pub.publish(String(data=self.mode))
        self.get_logger().info(
            "Switched to MICRO - WHITE now drives the micro-manipulator" if self.mode == "micro"
            else "Switched to MACRO - WHITE now drives the arm")
        return True

    def toggle_rcm(self):
        """Latch the tool tip as the RCM point, or leave RCM mode if the tool is retracted."""
        robot = self.lookup_robot_pose()
        if robot is None:
            self.get_logger().warn("Cannot switch RCM mode: no robot pose yet")
            return
        if self.rcm_point is None:
            self.rcm_point = self.rcm_tool.shaft(*robot)[2]
            self.rcm_fault = False
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.point.x, msg.point.y, msg.point.z = (float(v) for v in self.rcm_point)
            self.rcm_point_pub.publish(msg)
            self.get_logger().info(
                "RCM mode ON at tip ({:.4f}, {:.4f}, {:.4f}) m - the stylus now drives the tip, "
                "max depth {:.0f} mm".format(*self.rcm_point, 1e3 * self.rcm_max_depth))
            self.pulses.play("rcm_on", self.now())
        else:
            depth = self.rcm_tool.depth(*robot, self.rcm_point)
            if depth > self.rcm_exit_depth and not self.rcm_fault:
                self.get_logger().warn(f"Retract the tool before leaving RCM mode (depth {1e3 * depth:.1f} mm, "
                                       f"needs <= {1e3 * self.rcm_exit_depth:.1f} mm)")
                self.pulses.play("refused", self.now())
                return
            self.rcm_point = None
            self.get_logger().info("RCM mode OFF - free motion")
            self.pulses.play("rcm_off", self.now())
        self.rcm_active_pub.publish(Bool(data=self.rcm_point is not None))
        if self.clutched:
            self.engage()       # restart the stroke from here in the new mode

    def set_scale(self, index):
        """Select scale_steps[index] (wrapping); only call while not clutched or right before engage."""
        self.scale_index = index % len(self.scale_steps)
        step = self.scale_steps[self.scale_index]
        self.translation_scale = self.base_translation_scale * step
        self.deadband = self.base_deadband * step
        # Tilt and twist scale with translation (exponent 1 by default; < 1 scales them less)
        rotation_step = step ** self.scale_rotation_exponent
        self.rotation_scale = self.base_rotation_scale * rotation_step
        self.twist_scale = self.base_twist_scale * rotation_step
        self.scale_pub.publish(Float64(data=self.translation_scale))
        self.get_logger().info(
            f"Motion scale 1:{1.0 / step:g} (rotation 1:{1.0 / rotation_step:.3g})")

    def on_robot_description(self, msg):
        try:
            self.urdf_chain = UrdfChain(msg.data, self.base_frame, self.ee_frame)
        except (ET.ParseError, ValueError) as exc:
            self.get_logger().error(f"Cannot use /robot_description for the tool pose ({exc}) - using TF")

    def on_joint_states(self, msg):
        self.joint_positions = dict(zip(msg.name, msg.position))
        now = self.now()
        self.joint_states_time = now
        # No velocities in the message: treat it as moving, so a switch is never let through blind
        if not msg.velocity or max(abs(v) for v in msg.velocity) > self.tool_switch_speed:
            self.last_motion_time = now

    def select_tool(self, number):
        """Load tool `number` (0 = default) from the library as the RCM tool. Raises
        ValueError if it is not there or not usable; the current tool is kept then. With
        no library at all, falls back to the rcm_shaft_* parameters as tool 0."""
        default, tools = tool_library.load_library(self.tool_library)
        if not tools:
            if self.tool_number is not None or number:
                raise ValueError(f"no tools in {self.tool_library}")
            shaft = RcmTool(*(self.get_parameter(name).value for name in
                              ("rcm_shaft_start", "rcm_shaft_direction", "rcm_shaft_length")))
            entry = {"name": "rcm_shaft_* parameters (no tool library)",
                     "tip_tool0_mm": list(1e3 * shaft.tip_offset),
                     "direction_tool0": list(shaft.direction),
                     "shaft_length_mm": 1e3 * shaft.length}
            self.get_logger().warn(f"No tools in {self.tool_library} - using the rcm_shaft_* parameters")
            number = 0
        else:
            number = number or default or min(tools)
            if number not in tools:
                raise ValueError(f"no tool {number} in the library (have {', '.join(map(str, sorted(tools)))})")
            entry = tools[number]
        tool = tool_library.rcm_tool(entry)
        max_depth = tool.length - self.rcm_depth_margin
        if not max_depth > 0.0:
            raise ValueError(f"tool {number} shaft ({1e3 * tool.length:.1f} mm) is not longer than "
                             f"rcm_depth_margin ({1e3 * self.rcm_depth_margin:.1f} mm)")
        self.rcm_tool, self.rcm_max_depth, self.tool_number = tool, max_depth, number
        self.active_tool_pub.publish(String(data=tool_library.active_tool_message(number, entry)))
        self.get_logger().info(f"Tool {tool_library.describe(number, entry)}")
        return number

    def tool_switch_blocker(self):
        """Why a tool switch is not allowed right now, or None."""
        if self.rcm_point is not None:
            return "leave RCM mode first"
        if self.clutched:
            return "release WHITE first"
        now = self.now()
        if self.joint_states_time is None or now - self.joint_states_time > 0.5:
            return "no fresh /joint_states - cannot tell whether the robot is stationary"
        if self.last_motion_time is not None and now - self.last_motion_time < self.tool_switch_still_time:
            return "the robot is moving - wait until it is stationary"
        return None

    def on_set_parameters(self, params):
        for param in params:
            if param.name == "tool":
                reason = self.tool_switch_blocker()
                if reason is None:
                    try:
                        self.select_tool(param.value)
                    except (ValueError, OSError) as exc:
                        reason = str(exc)
                if reason is not None:
                    self.get_logger().warn(f"Tool switch to {param.value} refused: {reason}")
                    return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

    def jacobian_at(self, positions):
        """Arm Jacobian for joint positions in urdf_chain.joint_names order."""
        return self.urdf_chain.jacobian(dict(zip(self.urdf_chain.joint_names, positions)))

    def lookup_robot_pose(self):
        if self.urdf_chain is not None and self.joint_positions is not None:
            try:
                return self.urdf_chain.pose(self.joint_positions)
            except KeyError:
                pass        # joint missing from this message - fall back to TF
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        except TransformException as exc:
            self.get_logger().warn(f"No {self.base_frame}->{self.ee_frame} transform: {exc}",
                                   throttle_duration_sec=1.0)
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), Rotation.from_quat([r.x, r.y, r.z, r.w])

    def touch_is_fresh(self):
        if self.touch_stamp is None:
            return False
        age = (self.get_clock().now() - self.touch_stamp).nanoseconds * 1e-9
        return age <= self.touch_timeout

    def engage(self):
        robot = self.lookup_robot_pose()
        if robot is None or not self.touch_is_fresh():
            self.get_logger().warn("Cannot engage: no fresh Touch state or robot pose yet")
            return
        self.axis_matrix = toward_base_matrix(robot[0]) if self.mapping == "toward_base" else None
        if self.axis_matrix is None:
            if self.mapping == "toward_base":
                self.get_logger().warn("Tool is above the robot base - using axis_map for this stroke")
            self.axis_matrix = self.fixed_axis_matrix
        self.axis_rotation = Rotation.from_matrix(self.axis_matrix)
        self.touch_origin = (self.touch_position.copy(), self.touch_orientation)
        self.robot_origin = robot
        if self.rcm_point is not None:
            # The tip as if the shaft ran exactly through the RCM point, so a small drift
            # is corrected rather than becoming part of the target
            _, axis, _ = self.rcm_tool.shaft(*robot)
            depth = np.clip(self.rcm_tool.depth(*robot, self.rcm_point), self.rcm_min_depth, self.rcm_max_depth)
            self.shaft_axis_origin = axis
            self.tip_origin = self.rcm_point + depth * axis
        self.pen_twist = 0.0
        self.singularity_state = 0      # report it again if this stroke starts near one
        self.target = self.compute_target()
        self.previous_target = None
        self.target_history.clear()
        self.target_linear_velocity = np.zeros(3)
        self.target_angular_velocity = np.zeros(3)
        self.target_depth_velocity = 0.0
        if self.command_filter is not None:
            self.command_filter.reset()     # start from rest, as the robot is
        self.clutched = True
        self.get_logger().debug("Engaged - robot follows the stylus")

    def release(self):
        if self.clutched:
            self.clutched = False
            self.publish_twist(np.zeros(3), np.zeros(3))
            self.get_logger().debug("Released")

    def compute_target(self):
        """Tool0 target (position, orientation or None) from the stylus motion since engage."""
        touch_origin_position, touch_origin_orientation = self.touch_origin
        robot_origin_position, robot_origin_orientation = self.robot_origin

        # Position: robot origin + scaled, axis-mapped stylus displacement
        touch_delta = self.axis_matrix @ (self.touch_position - touch_origin_position)
        target_position = robot_origin_position + self.translation_scale * touch_delta

        have_orientation = self.touch_orientation is not None and touch_origin_orientation is not None
        if have_orientation:
            # Stylus rotation since engage in its own body frame, split into tilt of the pen
            # axis (swing) and twist about it
            swing, twist = split_twist(touch_origin_orientation.inv() * self.touch_orientation)
            self.pen_twist += wrap_angle(twist - wrap_angle(self.pen_twist))
        if self.rcm_point is not None:
            return self.rcm_target(touch_delta)
        if not (self.enable_rotation or self.enable_twist) or not have_orientation:
            return target_position, None

        target_orientation = robot_origin_orientation
        if self.enable_rotation:
            # Tilt in device axes, expressed in the stylus position frame, then the robot
            # base frame, and applied about the tool on top of its engage orientation
            delta = touch_origin_orientation * swing * touch_origin_orientation.inv()
            delta = TOUCH_POSITION_FRAME * delta * TOUCH_POSITION_FRAME.inv()
            delta = self.axis_rotation * delta * self.axis_rotation.inv()
            delta = Rotation.from_rotvec(self.rotation_scale * delta.as_rotvec())
            target_orientation = delta * target_orientation
        if self.enable_twist:
            # Twist about tool0 z (the last joint's axis)
            tool_twist = PEN_AXIS_TO_TOOL_Z_SIGN * self.twist_scale * self.pen_twist
            target_orientation = target_orientation * Rotation.from_rotvec([0.0, 0.0, tool_twist])
        return target_position, target_orientation

    def rcm_target(self, touch_delta):
        """Tool0 target in RCM mode: the shaft through the RCM point, aimed at the stylus tip
        target, rolled about itself by the pen twist (pen tilt is not used)."""
        tip_target = self.tip_origin + self.translation_scale * touch_delta
        axis, depth = self.rcm_tool.target_shaft(tip_target, self.rcm_point, self.shaft_axis_origin,
                                                 self.rcm_max_depth, self.rcm_slope, self.rcm_min_depth)
        orientation = RcmTool.align(self.shaft_axis_origin, axis) * self.robot_origin[1]
        if self.enable_twist:
            roll = PEN_AXIS_TO_TOOL_Z_SIGN * self.twist_scale * self.pen_twist
            orientation = orientation * Rotation.from_rotvec(roll * self.rcm_tool.direction)
        return self.rcm_tool.tool0_position(self.rcm_point + depth * axis, orientation), orientation

    def update(self):
        self.singularity_cue = None     # set again by guard_singularity if it runs this cycle
        self.check_mode_switch(self.now())
        self.update_motion()
        if self.force_pub is not None:
            self.update_haptics()

    def update_motion(self):
        if not self.clutched and self.rcm_point is None:
            return
        robot = self.lookup_robot_pose()
        if robot is not None and self.rcm_point is not None:
            self.publish_rcm_state(robot)
        if not self.clutched:
            return
        if not self.touch_is_fresh():
            self.get_logger().warn("Touch state is stale - holding still", throttle_duration_sec=1.0)
            return
        if robot is None:
            return
        if self.rcm_point is not None and self.rcm_fault:
            self.publish_twist(np.zeros(3), np.zeros(3))
            return
        robot_position, robot_orientation = robot
        self.update_target_velocity()
        # The robot pose reflects commands from loop_delay ago, so correct towards
        # where the target was then; the feedforward term carries the motion since
        target_position, target_orientation = self.delayed_target()

        if self.rcm_point is not None:
            # [insertion speed, 0, 0] + angular velocity here; the filter below works on these
            linear, angular = self.rcm_pivot_twist(robot, (target_position, target_orientation))
        else:
            error = target_position - robot_position
            linear = self.feedforward * self.target_linear_velocity
            if np.linalg.norm(error) >= self.deadband:
                linear = linear + clamp_norm(self.position_gain * error, self.max_correction)
            linear = clamp_norm(linear, self.max_linear)

            angular = np.zeros(3)
            if target_orientation is not None:
                rotation_error = (target_orientation * robot_orientation.inv()).as_rotvec()
                correction = clamp_norm(self.rotation_gain * rotation_error, self.max_angular_correction)
                angular = clamp_norm(self.feedforward * self.target_angular_velocity + correction, self.max_angular)

        if self.command_filter is not None:
            # Release still sends an unfiltered zero, so the robot stops at once.
            # Clamped again as the Butterworth can overshoot a step by ~4%.
            filtered = self.command_filter(np.concatenate([linear, angular]))
            if self.rcm_point is None:
                linear = clamp_norm(filtered[:3], self.max_linear)
                angular = clamp_norm(filtered[3:], self.max_angular)
            else:
                linear, angular = filtered[:3], filtered[3:]
        if self.rcm_point is not None:
            linear, angular = self.rcm_tool0_twist(robot, linear[0], angular)

        # Last, so it sees the twist Servo will get and the filter cannot smear its limits
        if self.singularity_guard is not None:
            linear, angular = self.guard_singularity(linear, angular)

        self.publish_twist(linear, angular)

    def rcm_pivot_twist(self, robot, target):
        """(insertion speed, angular velocity) as a 6-vector for the command filter: turn
        towards the target orientation about the RCM point, slide towards the target depth."""
        robot_position, robot_orientation = robot
        target_position, target_orientation = target
        rotation_error = (target_orientation * robot_orientation.inv()).as_rotvec()
        angular = (self.feedforward * self.target_angular_velocity
                   + clamp_norm(self.rotation_gain * rotation_error, self.max_angular_correction))
        depth_error = (self.rcm_tool.depth(target_position, target_orientation, self.rcm_point)
                       - self.rcm_tool.depth(robot_position, robot_orientation, self.rcm_point))
        insertion = self.feedforward * self.target_depth_velocity
        if abs(depth_error) >= self.deadband:
            insertion += float(np.clip(self.position_gain * depth_error, -self.max_correction, self.max_correction))
        return np.array([insertion, 0.0, 0.0]), angular

    def rcm_tool0_twist(self, robot, insertion, angular):
        """The tool0 twist for an insertion speed + angular velocity, built from the newest
        pose after the command filter (so the filter's delay does not bend the lever arm),
        led by loop_delay for the arm's own delay and speed-capped as a whole."""
        linear = self.rcm_tool.twist(*robot, self.rcm_point, insertion, angular,
                                     self.rcm_gain, self.max_correction, self.loop_delay)
        return self.cap_together(linear, angular)

    def cap_together(self, linear, angular):
        """Scale linear and angular by one factor to within the speed caps. Capping them
        separately would change the twist's shape, which moves the shaft off the RCM point."""
        factor = min(1.0, self.max_linear / max(np.linalg.norm(linear), 1e-12),
                     self.max_angular / max(np.linalg.norm(angular), 1e-12))
        return linear * factor, angular * factor

    def publish_rcm_state(self, robot):
        """Publish depth, shaft-to-RCM error and the tool tip; hold still if the shaft is off."""
        error = self.rcm_tool.line_error(*robot, self.rcm_point)
        self.rcm_error_pub.publish(Float64(data=error))
        self.insertion_depth_pub.publish(Float64(data=self.rcm_tool.depth(*robot, self.rcm_point)))
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = \
            (float(v) for v in self.rcm_tool.shaft(*robot)[2])
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = \
            (float(v) for v in (robot[1] * self.rcm_tool.tip_rotation).as_quat())
        self.tool_tip_pub.publish(msg)
        if error > self.rcm_max_error and not self.rcm_fault:
            self.rcm_fault = True
            self.get_logger().error(
                f"Shaft is {1e3 * error:.1f} mm off the RCM point (limit {1e3 * self.rcm_max_error:.1f} mm) - "
                "holding still. Hold WHITE + click GREY to leave RCM mode")

    def guard_singularity(self, linear, angular):
        """The twist with motion into a singularity removed (and a push back out of one)."""
        if self.urdf_chain is None or self.joint_positions is None:
            return linear, angular
        try:
            positions = [self.joint_positions[name] for name in self.urdf_chain.joint_names]
        except KeyError:
            return linear, angular
        guard = self.singularity_guard
        command = np.concatenate([linear, angular])
        twist = guard(command, positions)
        self.condition_pub.publish(Float64(data=float(guard.condition)))
        if guard.twist_gradient is not None:
            self.singularity_cue = (guard.limiting, guard.twist_gradient)
        if self.rcm_point is not None and twist is not command:
            # The guard reshapes the twist (and adds a back-off) in joint space, which would
            # move the shaft off the RCM point. Keep only how much of the command it lets
            # through: motion into the singularity is slowed, away from it is not, and with
            # no command the arm holds still at the limit instead of backing out.
            size = command @ command
            factor = float(np.clip(command @ twist / size, 0.0, 1.0)) if size > 0.0 else 0.0
            twist = factor * command

        # 2 = backing out, 1 = limiting, 0 = clear; with 10% hysteresis, as the guard
        # holds the arm right at `stop`, where it would flip every cycle
        condition, previous = guard.condition, self.singularity_state
        if guard.escaping or (previous == 2 and condition > 0.9 * guard.stop):
            state = 2
        elif guard.limiting > 0.0 or (previous >= 1 and condition > 0.9 * guard.slow):
            state = 1
        else:
            state = 0
        if state > previous:
            what = ur_singularity_name(positions) if len(positions) == 6 else "a singularity"
            if state == 1:
                self.get_logger().warn(f"Near the {what} singularity (condition {condition:.0f}) - "
                                       "motion towards it is slowed, along / away from it is free")
            elif self.rcm_point is not None:
                self.get_logger().warn(f"At the {what} singularity limit (condition {condition:.0f}) - "
                                       "RCM mode does not back out: move the stylus away")
            else:
                self.get_logger().warn(f"At the {what} singularity limit (condition {condition:.0f}) - "
                                       "backing out; keep holding WHITE, or move the stylus away")
        elif state == 0 and previous > 0:
            self.get_logger().info("Clear of the singularity")
        self.singularity_state = state
        # Servo's own speed caps still apply, so keep within them
        return clamp_norm(twist[:3], self.max_linear), clamp_norm(twist[3:], self.max_angular)

    def update_haptics(self):
        """Add up this cycle's haptic cues and send them (capped and slew-limited)."""
        force = np.zeros(3)
        engaged = self.clutched and self.touch_is_fresh()
        self.send_scene(engaged and self.rcm_point is not None)
        self.check_crossing(engaged and self.rcm_point is not None)
        self.check_limits(engaged and self.rcm_point is not None)
        if engaged:
            push = self.singularity_haptic_force() if self.singularity_cue is not None else np.zeros(3)
            alpha = self.publish_period / (self.publish_period + self.singularity_push_filter)
            self.singularity_force += alpha * (push - self.singularity_force)
        else:
            self.singularity_force = np.zeros(3)
        force += self.singularity_force
        force += self.pulses(self.now())
        self.send_force(force)

    def rcm_slope(self, axial):
        return self.rcm_tool.pivot_slope(axial, self.rcm_tool0_gain, self.rcm_tool0_gain_depth,
                                         self.rcm_pivot_fade_depth, self.rcm_pivot_fade_strength)

    # touch_server's TouchScenePacket (include/macro_micro_robot/touch_packets.hpp)
    SCENE_FORMAT = "<II3d3d7d"
    SCENE_MAGIC = 0x54534331

    def send_scene(self, active):
        """RCM mode: walls at the deepest insertion and at the retract limit, along the
        stylus direction that drives the depth, for touch_server to render from the pen's
        position every servo tick (computed here, over ROS, they arrived tens of ms late
        and shook the pen). touch_server can also render a detent there, but an attracting
        spring shakes a hand-held pen, so a click (check_crossing) is used instead. Placed on the stylus target's depth,
        unclipped, so pushing past a limit (where the target stops) presses into the wall.
        The depth limit is on the axial part of the target (the shaft's tilt is left
        out: the wall comes a little after the target stops when pivoted)."""
        if not active:
            if self.scene_active:
                self.send_scene_packet(enabled=False)
            return
        mm = 1.0 / self.units                                       # stylus metres -> mm
        scale = self.translation_scale
        axis = self.axis_matrix.T @ self.shaft_axis_origin          # stylus direction going deeper
        axial0 = float((self.tip_origin - self.rcm_point) @ self.shaft_axis_origin)
        # The target's depth is axial0 + scale * u for u stylus metres along axis
        self.send_scene_packet(
            axis=axis, origin=self.touch_origin[0] * mm,
            u_zero=-axial0 / scale * mm,
            u_min=(self.rcm_min_depth - axial0) / scale * mm,
            u_max=(self.rcm_max_depth - axial0) / scale * mm,
            stiffness=self.wall_stiffness / mm, detent_force=0.0, detent_width=1.0,
            max_force=self.haptic_max_force)

    def check_crossing(self, active):
        """Click (one tap against the direction of travel) when the tip target crosses the
        RCM point, with hysteresis so hovering at the point does not click repeatedly."""
        if not active:
            self.crossing_side = None
            return
        axis = self.axis_matrix.T @ self.shaft_axis_origin          # stylus direction going deeper
        axial0 = float((self.tip_origin - self.rcm_point) @ self.shaft_axis_origin)
        # Target depth in stylus metres (the target's depth is axial0 + scale * this travel)
        depth = axial0 / self.translation_scale + float((self.touch_position - self.touch_origin[0]) @ axis)
        side = 1 if depth > self.crossing_hysteresis else -1 if depth < -self.crossing_hysteresis else None
        if side is None:
            return
        now = self.now()
        if (self.crossing_side is not None and side != self.crossing_side
                and now - self.last_crossing_click >= self.crossing_holdoff):
            self.pulses.play("crossing", now, direction=-side * axis, amplitude=self.crossing_force)
            self.last_crossing_click = now
        self.crossing_side = side

    def check_limits(self, active):
        """Click (one tap back out of the limit) when the tip target reaches the deepest
        insertion or the retract limit. Replaces the walls, which buzzed at their edge on
        the real Touch. Same hysteresis and hold-off as the crossing click."""
        if not active or self.limit_force <= 0.0:
            self.limit_side = None
            return
        axis = self.axis_matrix.T @ self.shaft_axis_origin          # stylus direction going deeper
        axial0 = float((self.tip_origin - self.rcm_point) @ self.shaft_axis_origin)
        # Target depth past each limit, in stylus metres (as check_crossing)
        depth = axial0 / self.translation_scale + float((self.touch_position - self.touch_origin[0]) @ axis)
        deep = depth - self.rcm_max_depth / self.translation_scale
        shallow = self.rcm_min_depth / self.translation_scale - depth
        h = self.crossing_hysteresis
        if deep > 0.0 or shallow > 0.0:
            side = 1 if deep > 0.0 else -1
        elif deep < -h and shallow < -h:
            side = 0
        else:
            return                                                  # within the hysteresis band
        now = self.now()
        if (self.limit_side == 0 and side != 0
                and now - self.last_limit_click >= self.crossing_holdoff):
            self.pulses.play("limit", now, direction=-side * axis, amplitude=self.limit_force)
            self.last_limit_click = now
        self.limit_side = side

    def send_scene_packet(self, enabled=True, axis=(1.0, 0.0, 0.0), origin=(0.0, 0.0, 0.0), u_zero=0.0,
                          u_min=0.0, u_max=0.0, stiffness=0.0, detent_force=0.0, detent_width=0.0, max_force=0.0):
        packet = struct.pack(self.SCENE_FORMAT, self.SCENE_MAGIC, int(enabled), *axis, *origin,
                             u_zero, u_min, u_max, stiffness, detent_force, detent_width, max_force)
        try:
            self.scene_socket.sendto(packet, self.scene_address)
        except OSError:
            pass
        self.scene_active = enabled

    def singularity_haptic_force(self):
        """Push the stylus the way that takes the arm out of the singularity."""
        limiting, twist_gradient = self.singularity_cue
        relieve = self.stylus_twist_jacobian().T @ twist_gradient
        return singularity_force(relieve, limiting, self.touch_velocity,
                                 self.singularity_push, self.singularity_push_damping)

    # Stylus step (m) for the numerical RCM-mode stylus Jacobian
    STYLUS_STEP = 1e-4

    def stylus_twist_jacobian(self):
        """6 x 3: tool0 target twist (linear, angular; base frame) per metre of stylus
        motion along each stylus axis. In free mode the pen's position only moves the
        tool; in RCM mode it pivots it too (numerically, through rcm_target)."""
        if self.rcm_point is None:
            return np.vstack([self.translation_scale * self.axis_matrix, np.zeros((3, 3))])
        touch_delta = self.axis_matrix @ (self.touch_position - self.touch_origin[0])
        position, orientation = self.rcm_target(touch_delta)
        columns = []
        for column in self.axis_matrix.T:   # robot-frame image of each stylus axis
            moved_position, moved_orientation = self.rcm_target(touch_delta + self.STYLUS_STEP * column)
            columns.append(np.concatenate([moved_position - position,
                                           (moved_orientation * orientation.inv()).as_rotvec()]))
        return np.column_stack(columns) / self.STYLUS_STEP

    def send_force(self, force):
        force = limit_force(force, self.haptic_force, self.haptic_max_force, self.haptic_max_step)
        if force.any() or self.haptic_force.any():  # nothing to send while it stays 0
            self.haptic_force = force
            self.publish_force(force)

    def send_zero_force(self):
        if self.force_pub is not None:
            self.send_scene_packet(enabled=False)
            self.haptic_force = np.zeros(3)
            self.publish_force(self.haptic_force)

    def publish_force(self, force):
        msg = OmniFeedback()
        msg.force.x, msg.force.y, msg.force.z = (float(v) for v in force)
        self.force_pub.publish(msg)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def delayed_target(self):
        """The stylus target as it was loop_delay ago (the oldest one kept, early in a stroke)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        self.target_history.append((now, self.target))
        while len(self.target_history) > 1 and self.target_history[1][0] <= now - self.loop_delay:
            self.target_history.popleft()
        return self.target_history[0][1]

    def update_target_velocity(self):
        """Low-pass filtered velocity of the stylus target, for the feedforward term."""
        now = self.get_clock().now()
        previous, self.previous_target = self.previous_target, self.target
        dt = None if self.previous_update is None else (now - self.previous_update).nanoseconds * 1e-9
        self.previous_update = now
        if previous is None or not dt or dt > 0.1:
            return
        linear = (self.target[0] - previous[0]) / dt
        angular = np.zeros(3)
        if self.target[1] is not None and previous[1] is not None:
            angular = (self.target[1] * previous[1].inv()).as_rotvec() / dt
        alpha = dt / (dt + self.feedforward_filter)
        self.target_linear_velocity += alpha * (linear - self.target_linear_velocity)
        self.target_angular_velocity += alpha * (angular - self.target_angular_velocity)
        if self.rcm_point is not None:
            depth = (self.rcm_tool.depth(*self.target, self.rcm_point)
                     - self.rcm_tool.depth(*previous, self.rcm_point)) / dt
            self.target_depth_velocity += alpha * (depth - self.target_depth_velocity)

    def publish_target(self, stamp):
        position, orientation = self.target
        if orientation is None:
            orientation = self.robot_origin[1]
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
        q = orientation.as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = \
            (float(v) for v in q)
        self.target_pub.publish(msg)

    def publish_twist(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(v) for v in linear)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = (float(v) for v in angular)
        self.twist_pub.publish(msg)


def main():
    # Ctrl-C / SIGTERM handled here, not by rclpy (which would shut the context down
    # first), so the zero force can still be sent: the driver keeps the last one
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = TouchTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero_force()
        time.sleep(0.05)        # let it go out before the publisher is destroyed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
