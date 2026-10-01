#!/usr/bin/env python3
"""Replays a trajectory saved by trajectory_recorder.py on the UR5e.

    ros2 run ls_thesis trajectory_player.py ~/ros2_ws/trajectories/trajectory_XXXX.yaml
    ros2 run ls_thesis trajectory_player.py latest --speed 0.5
    ros2 run ls_thesis trajectory_player.py home     # move to a ready pose (elbow up, tool down)

1. Switches to the scaled_joint_trajectory_controller (teleop is paused).
2. Moves to the first recorded joint configuration over --approach-time seconds.
3. Plays the recording back, time-scaled by --speed.
4. Switches back to forward_position_controller so teleop works again.

The recorded tool path is published on /trajectory_player/path for RViz.
Asks for confirmation before moving unless --yes is given.
"""

import argparse
import glob
import os
import sys

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from trajectory_msgs.msg import JointTrajectoryPoint

from ls_thesis.ur5e_common import (
    TELEOP_CONTROLLER,
    TRAJECTORY_ACTION,
    TRAJECTORY_CONTROLLER,
    activate_controller,
)

DEFAULT_DIR = os.path.expanduser("~/ros2_ws/trajectories")

# Elbow-up, tool pointing down - well away from singularities, good for starting teleop
HOME_POSITIONS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
HOME_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# Recorded samples closer together than this are dropped; the controller
# interpolates between the rest.
MIN_POINT_SPACING = 0.04  # s


def to_duration(seconds):
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


def load_trajectory(name):
    if name == "home":
        # A single point; the approach move is the whole motion
        sample = {"time": 0.0, "positions": HOME_POSITIONS, "ee_pose": [0, 0, 0, 0, 0, 0, 1]}
        return "home pose", {"joint_names": HOME_JOINTS, "duration": 0.0, "samples": [sample]}
    path = resolve_file(name)
    with open(path) as f:
        data = yaml.safe_load(f)
    if not data or len(data.get("samples", [])) < 2:
        sys.exit(f"{path} has no trajectory samples")
    return path, data


def resolve_file(name):
    if name == "latest":
        files = sorted(glob.glob(os.path.join(DEFAULT_DIR, "trajectory_*.yaml")))
        if not files:
            sys.exit(f"No recorded trajectories in {DEFAULT_DIR}")
        return files[-1]
    if not os.path.exists(name) and os.path.exists(os.path.join(DEFAULT_DIR, name)):
        return os.path.join(DEFAULT_DIR, name)
    return name


class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__("trajectory_player")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "~/path", latched)
        self.action_client = ActionClient(self, FollowJointTrajectory, TRAJECTORY_ACTION)

    def publish_path(self, data):
        path = Path()
        path.header.frame_id = data.get("base_frame", "base_link")
        path.header.stamp = self.get_clock().now().to_msg()
        for sample in data["samples"]:
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            x, y, z, qx, qy, qz, qw = sample["ee_pose"]
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
            path.poses.append(pose)
        self.path_pub.publish(path)

    def build_goal(self, data, speed, approach_time):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = data["joint_names"]

        last_time = None
        for sample in data["samples"]:
            t = approach_time + sample["time"] / speed
            if last_time is not None and t - last_time < MIN_POINT_SPACING and sample is not data["samples"][-1]:
                continue
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in sample["positions"]]
            point.time_from_start = to_duration(t)
            goal.trajectory.points.append(point)
            last_time = t

        # Start and finish at rest
        zeros = [0.0] * len(goal.trajectory.joint_names)
        goal.trajectory.points[0].velocities = zeros
        goal.trajectory.points[-1].velocities = zeros
        return goal

    def execute(self, goal):
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Action server {TRAJECTORY_ACTION} not available")
            return False

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Trajectory goal was rejected")
            return False

        self.get_logger().info("Executing trajectory... (Ctrl+C to cancel)")
        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            self.get_logger().warn("Cancelling trajectory")
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return False

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"Trajectory failed (code {result.error_code}): {result.error_string}")
            return False
        self.get_logger().info("Trajectory complete")
        return True


def main():
    parser = argparse.ArgumentParser(description="Replay a recorded UR5e trajectory")
    parser.add_argument("file", help="trajectory YAML from trajectory_recorder, 'latest', or 'home'")
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed factor (default 1.0)")
    parser.add_argument("--approach-time", type=float, default=5.0,
                        help="seconds to move to the start pose (default 5)")
    parser.add_argument("--yes", action="store_true", help="don't ask for confirmation")
    parser.add_argument("--no-restore-teleop", action="store_true",
                        help="leave the trajectory controller active afterwards")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    if args.speed <= 0 or args.speed > 2.0:
        sys.exit("--speed must be in (0, 2]")

    path, data = load_trajectory(args.file)

    # Keep the context alive on Ctrl+C so the goal can be cancelled and teleop restored
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TrajectoryPlayer()
    if args.file != "home":
        node.publish_path(data)
    goal = node.build_goal(data, args.speed, args.approach_time)
    total = goal.trajectory.points[-1].time_from_start
    print(f"Trajectory: {path}\n  {len(data['samples'])} samples, "
          f"{data['duration']:.1f}s recorded, {total.sec + total.nanosec * 1e-9:.1f}s to play "
          f"at speed x{args.speed} (incl. {args.approach_time:.1f}s approach)")

    try:
        if not args.yes:
            answer = input("The robot will move to the start pose and replay. Proceed? [y/N] ")
            if answer.strip().lower() != "y":
                print("Aborted.")
                return
        if not activate_controller(node, TRAJECTORY_CONTROLLER):
            return
        node.execute(goal)
    except KeyboardInterrupt:
        pass
    finally:
        if not args.no_restore_teleop and rclpy.ok():
            activate_controller(node, TELEOP_CONTROLLER)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
