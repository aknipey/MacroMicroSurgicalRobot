#!/usr/bin/env python3
"""Smooths and limits MoveIt Servo's joint velocities before they reach the UR
(SERVO_OUTPUT "velocity" only):

    servo_node --Float64MultiArray qd--> /joint_velocity_limiter/commands
    joint_velocity_limiter --qd--> /forward_velocity_controller/commands  (or ur_emulator)

In velocity mode the UR runs `speedj(qd, 40 rad/s^2, 2 ms)` on whatever arrives, and
nothing between Servo and the arm smooths or limits it. On the real UR16e that gave
wrist torque chatter at the command update rate, and joint faults (C306A0) with the
chatter at its worst and a wrist at 28-43 rad/s^2 (see claude-notes/VERSIONS.md, V5).

Each command from Servo is, in this order:
1. scaled down, all joints together, so no joint exceeds `speed_fraction` of its
   URDF velocity limit (the tool keeps its direction, it just goes slower),
2. limited in acceleration: the change from the previous limited command is scaled
   down, all joints together, so no joint changes speed faster than its
   `max_acceleration`,
3. smoothed: two first-order stages with time constant `smoothing` each (critically
   damped). This rounds the corners the caps make when they engage and the steps
   left when a command arrives late. Its impulse response is never negative, so the
   output can never exceed the speed or acceleration caps (a Butterworth overshoots).

Within the limits only the smoothing acts (~6 ms at 3 ms). The output is published as
each command arrives. If Servo goes quiet for a period (it sends a few zeros when it
halts, then nothing) its last command is repeated through the limits and smoothing on a
timer, as Servo would have, and replaced by zero after VELOCITY_WATCHDOG_TIMEOUT, so
the arm never keeps a half-finished speed.

With `output` "position" (SERVO_OUTPUT "position") the limited velocities are
integrated into joint positions for forward_position_controller, so the UR runs servoj
instead of speedj (speedj buzzed in some poses, servoj never did):

    joint_velocity_limiter --q + qd * lead--> /forward_position_controller/commands

The position is this node's own running sum, not rebuilt from /joint_states each step
(that is what makes Servo's own position output crawl on the real arm). It starts from
the measured joints, again whenever the arm was moved by something else while this
node was idle, and is never allowed more than `max_position_error` ahead of them. Each
command is sent `lead` seconds ahead at the current speed, to cancel servoj's own lag.

Started by ur16e_teleop.launch.py when SERVO_OUTPUT is "position", or "velocity" with
VELOCITY_LIMITER on.
"""

import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from macro_micro_robot import teleop_settings as cfg
from macro_micro_robot.ur16e_common import (
    LIMITER_INPUT_TOPIC,
    LIMITER_NODE,
    SERVO_VELOCITY_TOPIC,
    UR_JOINTS,
    VELOCITY_WATCHDOG_TIMEOUT,
)


def urdf_velocity_limits(urdf, joints):
    """{joint: max velocity (rad/s)} from the URDF's <limit velocity=...> tags."""
    limits = {}
    for joint in ET.fromstring(urdf).iter("joint"):
        limit = joint.find("limit")
        if joint.get("name") in joints and limit is not None and limit.get("velocity") is not None:
            limits[joint.get("name")] = float(limit.get("velocity"))
    return limits


class JointVelocityLimiter(Node):
    def __init__(self):
        super().__init__(LIMITER_NODE)
        self.declare_parameter("output_topic", SERVO_VELOCITY_TOPIC)
        self.declare_parameter("rate", cfg.SERVO_VELOCITY_RATE)      # Hz, Servo's cycle
        self.declare_parameter("speed_fraction", cfg.VELOCITY_LIMITER_SPEED_FRACTION)
        self.declare_parameter("max_acceleration", cfg.VELOCITY_LIMITER_MAX_ACCELERATION)
        self.declare_parameter("smoothing", cfg.VELOCITY_LIMITER_SMOOTHING)    # s, per stage
        self.declare_parameter("output", "velocity")        # or "position"
        self.declare_parameter("lead", cfg.SERVO_POSITION_LEAD)                 # s, position output
        self.declare_parameter("max_position_error", cfg.SERVO_POSITION_MAX_ERROR)     # rad, position output
        param = lambda name: self.get_parameter(name).value
        self.position_output = param("output") == "position"
        if param("output") not in ("velocity", "position"):
            raise ValueError(f"output must be 'velocity' or 'position', not {param('output')!r}")
        self.lead = param("lead")
        self.max_position_error = param("max_position_error")
        self.period = 1.0 / param("rate")
        self.speed_fraction = param("speed_fraction")
        self.max_acceleration = np.array(param("max_acceleration"), dtype=float)
        if self.max_acceleration.shape != (len(UR_JOINTS),) or np.any(self.max_acceleration <= 0.0):
            raise ValueError(f"max_acceleration needs {len(UR_JOINTS)} positive values, not {param('max_acceleration')}")
        smoothing = param("smoothing")
        if smoothing < 0.0:
            raise ValueError(f"smoothing must be >= 0, not {smoothing}")
        self.alpha = self.period / (smoothing + self.period)   # per stage; 1 = no smoothing

        self.max_speed = None               # rad/s per joint, once /robot_description arrives
        self.command = np.zeros(len(UR_JOINTS))     # newest command from Servo, as sent
        self.limited = np.zeros(len(UR_JOINTS))     # last command after the caps, before smoothing
        self.stage = np.zeros(len(UR_JOINTS))       # first smoothing stage
        self.output = np.zeros(len(UR_JOINTS))      # last velocities sent
        self.last_input = None              # node time (s) of Servo's newest command
        self.last_output = None             # node time (s) of the newest output
        self.limit_count = 0                # commands held back by a cap since the last report
        self.measured = None                # newest /joint_states positions (position output)
        self.position = None                # integrated joint positions (position output)
        self.integrated_at = None           # node time (s) self.position was last advanced to
        self.held_count = 0                 # steps held back by max_position_error since the last report

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_robot_description, latched)
        self.publisher = self.create_publisher(Float64MultiArray, param("output_topic"), 10)
        self.create_subscription(Float64MultiArray, LIMITER_INPUT_TOPIC, self.on_command, 10)
        if self.position_output:
            self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_timer(self.period / 2.0, self.check)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"Velocity limiter -> {param('output_topic')}: speed <= {self.speed_fraction:g} x URDF limits, "
            f"acceleration <= {self.max_acceleration.tolist()} rad/s^2, smoothing 2 x {smoothing * 1000:g} ms"
            + (f", as positions {self.lead * 1000:g} ms ahead" if self.position_output else ""))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_robot_description(self, msg):
        try:
            limits = urdf_velocity_limits(msg.data, UR_JOINTS)
        except ET.ParseError as exc:
            self.get_logger().error(f"Cannot read joint limits from /robot_description: {exc}")
            return
        missing = [j for j in UR_JOINTS if j not in limits]
        if missing:
            self.get_logger().error(f"/robot_description has no velocity limit for {missing} - holding the arm still")
            return
        self.max_speed = self.speed_fraction * np.array([limits[j] for j in UR_JOINTS])
        self.get_logger().info(f"Joint speed caps: {np.round(self.max_speed, 2).tolist()} rad/s")

    def on_joint_states(self, msg):
        if all(n in msg.name for n in UR_JOINTS):
            self.measured = np.array([msg.position[msg.name.index(n)] for n in UR_JOINTS])

    def on_command(self, msg):
        if len(msg.data) != len(UR_JOINTS):
            self.get_logger().error(f"Expected {len(UR_JOINTS)} joint velocities, got {len(msg.data)}",
                                    throttle_duration_sec=1.0)
            return
        command = np.array(msg.data, dtype=float)
        if not np.all(np.isfinite(command)):
            self.get_logger().error("Non-finite joint velocity from Servo - treating it as zero",
                                    throttle_duration_sec=1.0)
            command = np.zeros(len(UR_JOINTS))
        self.command = command
        self.last_input = self.now()
        self.step(command)

    def check(self):
        """While Servo is quiet, repeats its last command (zero once it is older than
        VELOCITY_WATCHDOG_TIMEOUT) once per period, until the output has settled at zero."""
        now = self.now()
        if self.last_output is None or now - self.last_output < self.period:
            return
        command = self.command if now - self.last_input <= VELOCITY_WATCHDOG_TIMEOUT else np.zeros(len(UR_JOINTS))
        if np.any(self.output != 0.0) or np.any(self.limited != 0.0) or np.any(command != 0.0):
            self.step(command)

    def step(self, command):
        """Limits and smooths one command and publishes it."""
        if self.max_speed is None:
            limited = np.zeros(len(UR_JOINTS))      # no limits known yet: keep still
        else:
            limited = command
            # 1. speed: scale all joints together
            speed_scale = np.min(np.where(np.abs(limited) > 1e-12,
                                          self.max_speed / np.maximum(np.abs(limited), 1e-12), np.inf))
            if speed_scale < 1.0:
                limited = limited * speed_scale
            # 2. acceleration: scale the change together, so the joints stay in step
            change = limited - self.limited
            allowed = self.max_acceleration * self.period
            change_scale = np.min(np.where(np.abs(change) > 1e-12,
                                           allowed / np.maximum(np.abs(change), 1e-12), np.inf))
            if change_scale < 1.0:
                limited = self.limited + change * change_scale
            if speed_scale < 1.0 or change_scale < 1.0:
                self.limit_count += 1
        self.limited = limited
        # 3. smoothing: two first-order stages
        self.stage = self.stage + self.alpha * (limited - self.stage)
        velocities = self.output + self.alpha * (self.stage - self.output)
        velocities[np.abs(velocities) < 1e-4] = 0.0     # the smoothing tail, 0.006 deg/s
        was_moving = np.any(self.output != 0.0)
        self.output = velocities
        self.last_output = self.now()
        if self.position_output:
            self.publish_position(velocities, was_moving)
        else:
            self.publisher.publish(Float64MultiArray(data=[float(v) for v in velocities]))

    def publish_position(self, velocities, was_moving):
        """Integrates one step of velocities and publishes the position, `lead` ahead."""
        if self.measured is None:
            return      # nothing to start from: forward_position_controller keeps the arm still
        # Idle until now: start again from the measured joints if they are no longer where
        # the last command left them (another controller, e.g. a replay's approach, moved
        # the arm). Settled servoj tracking is far inside 0.01 rad.
        if self.position is None or (not was_moving and np.max(np.abs(self.position - self.measured)) > 0.01):
            self.position = self.measured.copy()
        # Over the time since the last step, not one period: commands arrive in bursts
        # (two within 0.1 ms, then a gap), and a full step each would move the set point
        # unevenly - a kick per robot cycle again. Capped so a long gap is no jump.
        now = self.last_output
        if not was_moving or self.integrated_at is None:
            dt = self.period
        else:
            dt = min(max(now - self.integrated_at, 0.0), 4 * self.period)
        self.integrated_at = now
        self.position = self.position + velocities * dt
        ahead = self.position - self.measured
        if np.any(np.abs(ahead) > self.max_position_error):
            self.position = self.measured + np.clip(ahead, -self.max_position_error, self.max_position_error)
            self.held_count += 1
        command = self.position + velocities * self.lead
        self.publisher.publish(Float64MultiArray(data=[float(q) for q in command]))

    def report(self):
        if self.limit_count:
            self.get_logger().info(f"Limited {self.limit_count} commands in the last 5 s "
                                   f"({self.limit_count * self.period:.2f} s of motion)")
            self.limit_count = 0
        if self.held_count:
            self.get_logger().warn(f"Arm more than {self.max_position_error:g} rad behind the command for "
                                   f"{self.held_count * self.period:.2f} s in the last 5 s - command held back")
            self.held_count = 0


def main():
    rclpy.init()
    node = JointVelocityLimiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
