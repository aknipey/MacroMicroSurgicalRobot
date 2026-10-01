#!/usr/bin/env python3
"""Records UR5e trajectories (joint angles + tool pose) while you teleoperate.

Services (std_srvs/Trigger):
    /trajectory_recorder/start   begin a new recording
    /trajectory_recorder/stop    stop and save it to <output_dir>/<name>.yaml (+ .csv)

The keyboard teleop ('[' / ']') and Touch teleop (grey button) call these for you.
The tool path being recorded is published as nav_msgs/Path on
/trajectory_recorder/path for RViz. Saved files can be replayed with
trajectory_player.py.
"""

import csv
import os
from datetime import datetime

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        self.declare_parameter("output_dir", os.path.expanduser("~/ros2_ws/trajectories"))
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("sample_rate", 25.0)     # Hz

        self.output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.joint_positions = None
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, "~/path", latched)

        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(Trigger, "~/stop", self.on_stop)

        self.recording = False
        self.samples = []
        self.start_time = None
        self.path = Path()
        self.path.header.frame_id = self.base_frame

        self.create_timer(1.0 / self.get_parameter("sample_rate").value, self.sample)
        self.get_logger().info(f"Trajectory recorder ready, saving to {self.output_dir}")

    def on_joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in UR_JOINTS):
            self.joint_positions = [positions[j] for j in UR_JOINTS]

    def on_start(self, _request, response):
        if self.recording:
            response.success = False
            response.message = "Already recording"
            return response
        self.samples = []
        self.path.poses = []
        self.start_time = self.get_clock().now()
        self.recording = True
        response.success = True
        response.message = "Recording trajectory..."
        return response

    def on_stop(self, _request, response):
        if not self.recording:
            response.success = False
            response.message = "Not recording"
            return response
        self.recording = False
        if len(self.samples) < 2:
            response.success = False
            response.message = "Recording stopped - too few samples to save (did the robot move?)"
            return response
        try:
            path = self.save()
        except OSError as exc:
            response.success = False
            response.message = f"Failed to save trajectory: {exc}"
            return response
        response.success = True
        response.message = f"Saved {len(self.samples)} samples to {path}"
        return response

    def sample(self):
        if not self.recording or self.joint_positions is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        except TransformException as exc:
            self.get_logger().warn(f"No {self.base_frame}->{self.ee_frame} transform: {exc}",
                                   throttle_duration_sec=1.0)
            return

        now = self.get_clock().now()
        t, r = tf.transform.translation, tf.transform.rotation
        self.samples.append({
            "time": (now - self.start_time).nanoseconds * 1e-9,
            "positions": list(self.joint_positions),
            "ee_pose": [t.x, t.y, t.z, r.x, r.y, r.z, r.w],
        })

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = now.to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
        pose.pose.orientation = r
        self.path.poses.append(pose)
        self.path.header.stamp = now.to_msg()
        self.path_pub.publish(self.path)

    def save(self):
        os.makedirs(self.output_dir, exist_ok=True)
        name = datetime.now().strftime("trajectory_%Y%m%d_%H%M%S")
        yaml_path = os.path.join(self.output_dir, name + ".yaml")
        csv_path = os.path.join(self.output_dir, name + ".csv")

        # Re-base time so the first saved sample is t = 0
        t0 = self.samples[0]["time"]
        samples = [dict(s, time=round(s["time"] - t0, 4)) for s in self.samples]

        with open(yaml_path, "w") as f:
            yaml.safe_dump({
                "joint_names": UR_JOINTS,
                "base_frame": self.base_frame,
                "ee_frame": self.ee_frame,
                "duration": samples[-1]["time"],
                "samples": samples,
            }, f, default_flow_style=None, sort_keys=False)

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time"] + UR_JOINTS + ["x", "y", "z", "qx", "qy", "qz", "qw"])
            for s in samples:
                writer.writerow([s["time"]] + s["positions"] + s["ee_pose"])

        return yaml_path


def main():
    rclpy.init()
    node = TrajectoryRecorder()
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
