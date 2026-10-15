#!/usr/bin/env python3
"""Touch (Geomagic / 3D Systems) stylus teleoperation of the UR5e through MoveIt Servo.

Hold the WHITE button to drive the robot (clutch / deadman). The stylus pose
at the moment the button goes down is the origin: while held, the robot tool
follows the stylus displacement and tilt since then (scaled by
`translation_scale` / `rotation_scale`); releasing it freezes the robot so you
can re-centre the stylus and press again.

Double-click WHITE to cycle the motion scale (1:1 -> 1:5 -> 1:20 by default,
`scale_steps`). Nothing waits to see whether a press becomes a double click:
both presses engage straight away, and the new scale applies from the second
press (which re-captures the origin, so the robot does not jump). Tilt and
twist are scaled by step ** `scale_rotation_exponent` (sqrt by default). The current
scale is published latched on ~/motion_scale for the logs.

Press the GREY button to start / stop recording a trajectory and the
Touch / UR path logs (needs trajectory_recorder running).

While engaged, the stylus pose mapped into the robot frame (the tool0 pose the
robot is being pulled towards) is published on ~/target_pose, once per stylus
sample and stamped with the stylus sample's own stamp, for the logger.

Frames: the omni_state driver publishes the stylus position in millimetres
with x = right, y = away from the operator (towards the Touch body), z = up.
With `mapping` = "toward_base" (default) the Touch is treated as a small copy
of the robot seen from the tool: up is up, pushing the stylus towards the
Touch body moves the tool horizontally towards the robot base, and right is
right from that viewpoint. The direction is frozen when WHITE is pressed.
With `mapping` = "fixed", `axis_map` says which stylus axis drives each robot
axis of `base_frame`. Stylus tilts are mapped through the same frame, so the
wrist tilts the way the pen does. Twisting the pen about its own axis
(`enable_twist`) turns the tool about tool0 z, i.e. the last joint.

Singularities (`singularity_guard`): near one (wrist 2 near 0 / 180 deg, elbow
straight, wrist centre over the base) the part of the motion heading into it is
slowed and then removed, so the tool slides along the edge instead of stalling,
and if the arm is past the limit (or starts there) holding WHITE backs it out.
The arm's condition number is published on ~/condition_number.
"""

import xml.etree.ElementTree as ET
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from omni_msgs.msg import OmniButtonEvent, OmniState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from ls_thesis.ur5e_common import (
    SERVO_TWIST_TOPIC,
    LowPass,
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
from ls_thesis import teleop_settings as cfg

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
        super().__init__("ur5e_touch_teleop")

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

        self.twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, 10)
        self.target_pub = self.create_publisher(PoseStamped, "~/target_pose", 100)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scale_pub = self.create_publisher(Float64, "~/motion_scale", latched)
        self.condition_pub = self.create_publisher(Float64, "~/condition_number", 10)
        self.recorder = RecorderClient(self)

        self.touch_position = None      # metres, stylus frame
        self.touch_orientation = None   # scipy Rotation, as published
        self.touch_stamp = None
        self.white_pressed = False
        self.grey_pressed = False
        # Double-click detection on WHITE (node clock, s)
        self.white_press_time = None    # current press; None once it was used by a double click
        self.white_click_time = None    # start of the last short press-and-release

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
                    "Could not configure MoveIt Servo - is ur5e_teleop.launch.py running?")
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
            "GREY to start/stop recording "
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
        if self.clutched:
            self.target = self.compute_target()
            self.publish_target(msg.header.stamp)

    def on_button(self, msg):
        white, grey = bool(msg.white_button), bool(msg.grey_button)
        now = self.get_clock().now().nanoseconds * 1e-9
        if white and not self.white_pressed:
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
        if grey and not self.grey_pressed:
            self.recorder.toggle()
        self.white_pressed, self.grey_pressed = white, grey

    def set_scale(self, index):
        """Select scale_steps[index] (wrapping); only call while not clutched or right before engage."""
        self.scale_index = index % len(self.scale_steps)
        step = self.scale_steps[self.scale_index]
        self.translation_scale = self.base_translation_scale * step
        self.deadband = self.base_deadband * step
        # Tilt and twist are scaled less than translation (sqrt of it by default)
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
        self.pen_twist = 0.0
        self.singularity_state = 0      # report it again if this stroke starts near one
        self.target = self.compute_target()
        self.previous_target = None
        self.target_history.clear()
        self.target_linear_velocity = np.zeros(3)
        self.target_angular_velocity = np.zeros(3)
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
        if not (self.enable_rotation or self.enable_twist) or not have_orientation:
            return target_position, None

        # Stylus rotation since engage in its own body frame, split into tilt of the pen
        # axis (swing) and twist about it
        swing, twist = split_twist(touch_origin_orientation.inv() * self.touch_orientation)
        self.pen_twist += wrap_angle(twist - wrap_angle(self.pen_twist))

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

    def update(self):
        if not self.clutched:
            return
        if not self.touch_is_fresh():
            self.get_logger().warn("Touch state is stale - holding still", throttle_duration_sec=1.0)
            return
        robot = self.lookup_robot_pose()
        if robot is None:
            return
        robot_position, robot_orientation = robot
        self.update_target_velocity()
        # The robot pose reflects commands from loop_delay ago, so correct towards
        # where the target was then; the feedforward term carries the motion since
        target_position, target_orientation = self.delayed_target()

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
            linear = clamp_norm(filtered[:3], self.max_linear)
            angular = clamp_norm(filtered[3:], self.max_angular)

        # Last, so it sees the twist Servo will get and the filter cannot smear its limits
        if self.singularity_guard is not None:
            linear, angular = self.guard_singularity(linear, angular)

        self.publish_twist(linear, angular)

    def guard_singularity(self, linear, angular):
        """The twist with motion into a singularity removed (and a push back out of one)."""
        if self.urdf_chain is None or self.joint_positions is None:
            return linear, angular
        try:
            positions = [self.joint_positions[name] for name in self.urdf_chain.joint_names]
        except KeyError:
            return linear, angular
        guard = self.singularity_guard
        twist = guard(np.concatenate([linear, angular]), positions)
        self.condition_pub.publish(Float64(data=float(guard.condition)))

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
            else:
                self.get_logger().warn(f"At the {what} singularity limit (condition {condition:.0f}) - "
                                       "backing out; keep holding WHITE, or move the stylus away")
        elif state == 0 and previous > 0:
            self.get_logger().info("Clear of the singularity")
        self.singularity_state = state
        # Servo's own speed caps still apply, so keep within them
        return clamp_norm(twist[:3], self.max_linear), clamp_norm(twist[3:], self.max_angular)

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
    rclpy.init()
    node = TouchTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
