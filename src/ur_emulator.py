#!/usr/bin/env python3
"""Stands in for the real UR's response on mock hardware, so latency changes can be
tested without the arm.

Mock hardware follows every command instantly, which hides the delays that matter on
the real robot. This node takes MoveIt Servo's output in place of the UR controller,
runs it through a model of the real arm, and drives the mock joints through
forward_position_controller:

    servo_node --JointTrajectory-->      /ur_emulator/joint_trajectory  (SERVO_OUTPUT "trajectory")
    servo_node --Float64MultiArray qd--> /ur_emulator/commands          (SERVO_OUTPUT "velocity")
    ur_emulator --positions--> /forward_position_controller/commands -> mock joints

Trajectory mode does what scaled_joint_trajectory_controller does (samples the newest
trajectory at the current time, linear between points) and then what the UR does with
that set point: `dead_time` of pure delay and a first-order lag `tau`. Fitted to the
real UR16e logs of 2026-10-12 (logs/V3/*): 16 ms + 15 ms, 0.1-0.4 mrad RMS.

Velocity mode models forward_velocity_controller + the driver's `speedj(qd, 40, 2 ms)`:
the same dead time, then the joint speed ramps to the command at `accel_limit` rad/s^2
(plus an optional first-order `velocity_tau`). This is an estimate - no real
velocity-mode data has been logged yet.

Started by ur5e_teleop.launch.py with use_mock_hardware:=true emulate_ur:=true.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from ls_thesis.ur5e_common import EMULATOR_TRAJECTORY_TOPIC, EMULATOR_VELOCITY_TOPIC

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
OUTPUT_TOPIC = "/forward_position_controller/commands"


class UrEmulator(Node):
    def __init__(self):
        super().__init__("ur_emulator")
        self.declare_parameter("rate", 500.0)               # Hz, the UR's control rate
        self.declare_parameter("dead_time", 0.016)          # s
        self.declare_parameter("tau", 0.015)                # s, position mode
        self.declare_parameter("accel_limit", 40.0)         # rad/s^2, velocity mode
        self.declare_parameter("velocity_tau", 0.0)         # s, velocity mode
        p = lambda name: self.get_parameter(name).value
        self.dead_time = p("dead_time")
        self.tau = p("tau")
        self.accel_limit = p("accel_limit")
        self.velocity_tau = p("velocity_tau")

        self.q = None                   # emulated joint positions
        self.qd = np.zeros(6)           # emulated joint velocities (velocity mode)
        self.trajectory = None          # (times s, positions [n x 6]) of the newest trajectory
        self.velocity_command = None    # qd, kept until the next one (no timeout, like the real controller)
        self.setpoints = deque()        # (time s, mode, value) waiting out dead_time
        self.active = None              # the set point the emulated UR is following
        self.previous = None
        self.sent = deque()             # (time s, positions) recently sent to the mock joints
        self.overridden = False         # another controller is moving the mock joints

        self.output = self.create_publisher(Float64MultiArray, OUTPUT_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(JointTrajectory, EMULATOR_TRAJECTORY_TOPIC, self.on_trajectory, 10)
        self.create_subscription(Float64MultiArray, EMULATOR_VELOCITY_TOPIC, self.on_velocity, 10)
        self.create_timer(1.0 / p("rate"), self.step)
        self.get_logger().info(
            f"Emulating the UR: dead time {self.dead_time * 1000:.0f} ms, position lag {self.tau * 1000:.0f} ms, "
            f"velocity mode {self.accel_limit:g} rad/s^2, velocity lag {self.velocity_tau * 1000:.0f} ms")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joint_states(self, msg):
        if not all(n in msg.name for n in UR_JOINTS):
            return
        measured = np.array([msg.position[msg.name.index(n)] for n in UR_JOINTS])
        # Mock joints that are not where this node put them were moved by another
        # controller (e.g. the trajectory controller for a replay's approach): start
        # again from there, dropping the old command
        if self.q is not None and any(np.max(np.abs(measured - q)) < 1e-6 for _, q in self.sent):
            self.overridden = False
        else:
            if self.q is not None and not self.overridden:
                self.get_logger().info("Joints moved by another controller - following them until it stops")
            self.overridden = self.q is not None
            self.q, self.qd = measured, np.zeros(6)
            self.trajectory = self.velocity_command = self.active = None
            self.setpoints.clear()
            self.sent.clear()

    def on_trajectory(self, msg):
        if not msg.points or any(n not in msg.joint_names for n in UR_JOINTS):
            return
        order = [msg.joint_names.index(n) for n in UR_JOINTS]
        # Like the trajectory controller: a zero stamp means "start now"
        start = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9 or self.now()
        times = np.array([start + p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in msg.points])
        positions = np.array([[p.positions[i] for i in order] for p in msg.points])
        self.trajectory = (times, positions)
        self.velocity_command = None

    def on_velocity(self, msg):
        if len(msg.data) == 6:
            self.velocity_command = np.array(msg.data)
            self.trajectory = None

    def set_point(self, now):
        """(mode, value) the controller asks for right now, before the UR's delay."""
        if self.trajectory is not None:
            times, positions = self.trajectory
            return "position", np.array([np.interp(now, times, positions[:, j]) for j in range(6)])
        if self.velocity_command is not None:
            return "velocity", self.velocity_command
        return None

    def step(self):
        if self.q is None:
            return
        now = self.now()
        dt = 0.0 if self.previous is None else min(now - self.previous, 0.05)
        self.previous = now
        command = self.set_point(now)
        if command is not None:
            self.setpoints.append((now,) + command)
        # The newest set point that has waited out the dead time
        current = None
        while self.setpoints and self.setpoints[0][0] <= now - self.dead_time:
            current = self.setpoints.popleft()
        if current is not None:
            self.active = current
        if self.active is not None and dt > 0.0:
            _, mode, value = self.active
            if mode == "position":
                self.qd = np.zeros(6)
                self.q = self.q + dt / (self.tau + dt) * (value - self.q)
            else:
                target = value
                if self.velocity_tau > 0.0:
                    target = self.qd + dt / (self.velocity_tau + dt) * (value - self.qd)
                change = np.clip(target - self.qd, -self.accel_limit * dt, self.accel_limit * dt)
                self.qd = self.qd + change
                self.q = self.q + self.qd * dt
        self.output.publish(Float64MultiArray(data=[float(v) for v in self.q]))
        self.sent.append((now, self.q.copy()))
        while len(self.sent) > 1 and self.sent[0][0] < now - 0.2:
            self.sent.popleft()


def main():
    rclpy.init()
    node = UrEmulator()
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
