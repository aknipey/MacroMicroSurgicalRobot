#!/usr/bin/env python3
"""Touch (Geomagic / 3D Systems) stylus teleoperation of the UR5e through MoveIt Servo.

Hold the WHITE button to drive the robot (clutch / deadman). The stylus pose
at the moment the button goes down is the origin: while held, the robot tool
follows the stylus displacement and tilt since then (scaled by
`translation_scale` / `rotation_scale`); releasing it freezes the robot so you
can re-centre the stylus and press again.

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
"""

from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from omni_msgs.msg import OmniButtonEvent, OmniState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from ls_thesis.ur5e_common import (
    SERVO_TWIST_TOPIC,
    TELEOP_CONTROLLER,
    RecorderClient,
    activate_controller,
    set_servo_speed_limits,
    start_servo_twist_mode,
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
        self.declare_parameter("enable_rotation", cfg.TOUCH_ENABLE_ROTATION)
        self.declare_parameter("rotation_scale", cfg.TOUCH_ROTATION_SCALE)
        self.declare_parameter("enable_twist", cfg.TOUCH_ENABLE_TWIST)
        self.declare_parameter("twist_scale", cfg.TOUCH_TWIST_SCALE)
        self.declare_parameter("position_gain", cfg.TOUCH_POSITION_GAIN)
        self.declare_parameter("rotation_gain", cfg.TOUCH_ROTATION_GAIN)
        self.declare_parameter("feedforward", cfg.TOUCH_FEEDFORWARD)
        self.declare_parameter("feedforward_filter", cfg.TOUCH_FEEDFORWARD_FILTER)
        self.declare_parameter("loop_delay", cfg.TOUCH_LOOP_DELAY)
        self.declare_parameter("max_linear_speed", cfg.TOUCH_MAX_LINEAR_SPEED)
        self.declare_parameter("max_angular_speed", cfg.TOUCH_MAX_ANGULAR_SPEED)
        self.declare_parameter("deadband", cfg.TOUCH_DEADBAND)
        self.declare_parameter("touch_timeout", cfg.TOUCH_TIMEOUT)
        self.declare_parameter("publish_rate", cfg.TOUCH_PUBLISH_RATE)
        self.declare_parameter("setup_servo", True)

        p = lambda name: self.get_parameter(name).value
        self.base_frame = p("base_frame")
        self.ee_frame = p("ee_frame")
        self.mapping = p("mapping")
        if self.mapping not in ("toward_base", "fixed"):
            raise ValueError(f"mapping must be 'toward_base' or 'fixed', not {self.mapping!r}")
        self.fixed_axis_matrix = axis_map_to_matrix(p("axis_map"))
        self.units = p("touch_units_to_m")
        self.translation_scale = p("translation_scale")
        self.enable_rotation = p("enable_rotation")
        self.rotation_scale = p("rotation_scale")
        self.enable_twist = p("enable_twist")
        self.twist_scale = p("twist_scale")
        self.position_gain = p("position_gain")
        self.rotation_gain = p("rotation_gain")
        self.feedforward = p("feedforward")
        self.feedforward_filter = p("feedforward_filter")
        self.loop_delay = p("loop_delay")
        self.max_linear = p("max_linear_speed")
        self.max_angular = p("max_angular_speed")
        self.deadband = p("deadband")
        self.touch_timeout = p("touch_timeout")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, 10)
        self.target_pub = self.create_publisher(PoseStamped, "~/target_pose", 100)
        self.recorder = RecorderClient(self)

        self.touch_position = None      # metres, stylus frame
        self.touch_orientation = None   # scipy Rotation, as published
        self.touch_stamp = None
        self.white_pressed = False
        self.grey_pressed = False

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
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, PROGRAM_RUNNING_TOPIC, self.on_program_running, latched)

        if p("setup_servo"):
            activate_controller(self, TELEOP_CONTROLLER)
            if not start_servo_twist_mode(self):
                self.get_logger().error(
                    "Could not configure MoveIt Servo - is ur5e_teleop.launch.py running?")
            set_servo_speed_limits(self, self.max_linear, self.max_angular)

        self.create_timer(1.0 / p("publish_rate"), self.update)
        self.get_logger().info(
            "Touch teleop ready: hold WHITE to move, GREY to start/stop recording "
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
        self.touch_position = np.array([pos.x, pos.y, pos.z]) * self.units
        quat = [ori.x, ori.y, ori.z, ori.w]
        self.touch_orientation = Rotation.from_quat(quat) if np.linalg.norm(quat) > 1e-6 else None
        self.touch_stamp = self.get_clock().now()
        if self.clutched:
            self.target = self.compute_target()
            self.publish_target(msg.header.stamp)

    def on_button(self, msg):
        white, grey = bool(msg.white_button), bool(msg.grey_button)
        if white and not self.white_pressed:
            self.engage()
        elif not white and self.white_pressed:
            self.release()
        if grey and not self.grey_pressed:
            self.recorder.toggle()
        self.white_pressed, self.grey_pressed = white, grey

    def lookup_robot_pose(self):
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
        self.target = self.compute_target()
        self.previous_target = None
        self.target_history.clear()
        self.target_linear_velocity = np.zeros(3)
        self.target_angular_velocity = np.zeros(3)
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
            linear = linear + self.position_gain * error
        linear = clamp_norm(linear, self.max_linear)

        angular = np.zeros(3)
        if target_orientation is not None:
            rotation_error = (target_orientation * robot_orientation.inv()).as_rotvec()
            angular = clamp_norm(self.feedforward * self.target_angular_velocity
                                 + self.rotation_gain * rotation_error, self.max_angular)

        self.publish_twist(linear, angular)

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
