#!/usr/bin/env python3
"""Touch (Geomagic / 3D Systems) stylus teleoperation of the UR5e through MoveIt Servo.

Hold the WHITE button to drive the robot (clutch / deadman). While held, the
robot tool tries to follow the stylus displacement since the button was
pressed (scaled by `translation_scale`); releasing it freezes the robot so you
can re-centre the stylus and press again.

Press the GREY button to start / stop recording a trajectory (needs
trajectory_recorder running).

Frames: the omni_state driver publishes the stylus position in millimetres
with x = right, y = away from the operator, z = up. `axis_map` says which
stylus axis drives each robot axis of `base_frame`; the default assumes the
operator stands in front of the robot facing along base_link +X.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from omni_msgs.msg import OmniButtonEvent, OmniState
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener

from ls_thesis.ur5e_common import (
    SERVO_TWIST_TOPIC,
    TELEOP_CONTROLLER,
    RecorderClient,
    activate_controller,
    start_servo_twist_mode,
)

# Orientation published by omni_state = R_OFFSET * R_device (see omni_state.cpp),
# while its position axes are the device axes rotated by +90 deg about X.
TOUCH_ROTATION_OFFSET = Rotation.from_euler("z", 90, degrees=True)
TOUCH_POSITION_FRAME = Rotation.from_euler("x", 90, degrees=True)
TOUCH_ORIENTATION_TO_POSITION_FRAME = TOUCH_POSITION_FRAME * TOUCH_ROTATION_OFFSET.inv()


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
        self.declare_parameter("axis_map", ["y", "-x", "z"])
        self.declare_parameter("touch_units_to_m", 1e-3)     # omni_state default units are mm
        self.declare_parameter("translation_scale", 1.0)     # robot metres per stylus metre
        self.declare_parameter("enable_rotation", False)
        self.declare_parameter("rotation_scale", 1.0)
        self.declare_parameter("position_gain", 4.0)         # 1/s
        self.declare_parameter("rotation_gain", 3.0)         # 1/s
        self.declare_parameter("max_linear_speed", 0.15)     # m/s
        self.declare_parameter("max_angular_speed", 0.6)     # rad/s
        self.declare_parameter("deadband", 3e-4)             # m
        self.declare_parameter("touch_timeout", 0.1)         # s
        self.declare_parameter("publish_rate", 100.0)        # Hz
        self.declare_parameter("setup_servo", True)

        p = lambda name: self.get_parameter(name).value
        self.base_frame = p("base_frame")
        self.ee_frame = p("ee_frame")
        self.axis_matrix = axis_map_to_matrix(p("axis_map"))
        self.axis_rotation = Rotation.from_matrix(self.axis_matrix)
        self.units = p("touch_units_to_m")
        self.translation_scale = p("translation_scale")
        self.enable_rotation = p("enable_rotation")
        self.rotation_scale = p("rotation_scale")
        self.position_gain = p("position_gain")
        self.rotation_gain = p("rotation_gain")
        self.max_linear = p("max_linear_speed")
        self.max_angular = p("max_angular_speed")
        self.deadband = p("deadband")
        self.touch_timeout = p("touch_timeout")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, 10)
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

        self.create_subscription(OmniState, p("state_topic"), self.on_state, 1)
        self.create_subscription(OmniButtonEvent, p("button_topic"), self.on_button, 10)

        if p("setup_servo"):
            activate_controller(self, TELEOP_CONTROLLER)
            if not start_servo_twist_mode(self):
                self.get_logger().error(
                    "Could not configure MoveIt Servo - is ur5e_teleop.launch.py running?")

        self.create_timer(1.0 / p("publish_rate"), self.update)
        self.get_logger().info(
            "Touch teleop ready: hold WHITE to move the UR5e, GREY to start/stop recording. "
            f"Rotation {'enabled' if self.enable_rotation else 'disabled'}.")

    def on_state(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.touch_position = np.array([pos.x, pos.y, pos.z]) * self.units
        quat = [ori.x, ori.y, ori.z, ori.w]
        self.touch_orientation = Rotation.from_quat(quat) if np.linalg.norm(quat) > 1e-6 else None
        self.touch_stamp = self.get_clock().now()

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
        self.touch_origin = (self.touch_position.copy(), self.touch_orientation)
        self.robot_origin = robot
        self.clutched = True
        self.get_logger().info("Engaged - robot follows the stylus")

    def release(self):
        if self.clutched:
            self.clutched = False
            self.publish_twist(np.zeros(3), np.zeros(3))
            self.get_logger().info("Released")

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
        touch_origin_position, touch_origin_orientation = self.touch_origin
        robot_origin_position, robot_origin_orientation = self.robot_origin

        # Position: robot target = robot origin + scaled, axis-mapped stylus displacement
        touch_delta = self.axis_matrix @ (self.touch_position - touch_origin_position)
        target = robot_origin_position + self.translation_scale * touch_delta
        error = target - robot_position
        linear = np.zeros(3) if np.linalg.norm(error) < self.deadband else \
            clamp_norm(self.position_gain * error, self.max_linear)

        angular = np.zeros(3)
        if self.enable_rotation and self.touch_orientation is not None and touch_origin_orientation is not None:
            # Stylus rotation since engage, expressed in the stylus position frame, then the robot base frame
            delta = self.touch_orientation * touch_origin_orientation.inv()
            delta = TOUCH_ORIENTATION_TO_POSITION_FRAME * delta * TOUCH_ORIENTATION_TO_POSITION_FRAME.inv()
            delta = self.axis_rotation * delta * self.axis_rotation.inv()
            delta = Rotation.from_rotvec(self.rotation_scale * delta.as_rotvec())
            target_orientation = delta * robot_origin_orientation
            rotation_error = (target_orientation * robot_orientation.inv()).as_rotvec()
            angular = clamp_norm(self.rotation_gain * rotation_error, self.max_angular)

        self.publish_twist(linear, angular)

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
